#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace gz::sim::systems {
namespace {

double SdfDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                 const std::string &_key, const double _defaultValue) {
  return _sdf->Get<double>(_key, _defaultValue).first;
}

bool SdfBool(const std::shared_ptr<const sdf::Element> &_sdf,
             const std::string &_key, const bool _defaultValue) {
  return _sdf->Get<bool>(_key, _defaultValue).first;
}

}  // namespace

class HybridAirPropellerModel : public System,
                                public ISystemConfigure,
                                public ISystemPreUpdate {
 public:
  HybridAirPropellerModel() = default;
  ~HybridAirPropellerModel() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override {
    this->model_ = Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "[HybridAirPropellerModel] Invalid model entity" << std::endl;
      return;
    }

    this->jointName_ = _sdf->Get<std::string>("jointName", "").first;
    this->linkName_ = _sdf->Get<std::string>("linkName", "").first;
    this->axisFrameLinkName_ =
        _sdf->Get<std::string>("axisFrameLinkName", this->linkName_).first;
    this->turningDirection_ =
        _sdf->Get<std::string>("turningDirection", "ccw").first;
    this->commandTopic_ =
        _sdf->Get<std::string>("commandSubTopic", this->commandTopic_).first;
    this->diagnosticTopic_ =
        _sdf->Get<std::string>("diagnosticTopic", this->diagnosticTopic_).first;
    this->motorNumber_ = _sdf->Get<int>("motorNumber", this->motorNumber_).first;

    this->normalizedCommand_ =
        SdfBool(_sdf, "normalizedCommand", this->normalizedCommand_);
    this->bidirectional_ = SdfBool(_sdf, "bidirectional", this->bidirectional_);
    this->timeConstantUp_ =
        SdfDouble(_sdf, "timeConstantUp", this->timeConstantUp_);
    this->timeConstantDown_ =
        SdfDouble(_sdf, "timeConstantDown", this->timeConstantDown_);
    this->maxRotVelocity_ =
        SdfDouble(_sdf, "maxRotVelocity", this->maxRotVelocity_);
    this->motorConstantAir_ =
        SdfDouble(_sdf, "motorConstantAir", this->motorConstantAir_);
    this->momentConstantAir_ =
        std::abs(SdfDouble(_sdf, "momentConstantAir", this->momentConstantAir_));
    this->motorConstantWater_ =
        SdfDouble(_sdf, "motorConstantWater", this->motorConstantWater_);
    this->momentConstantWater_ =
        std::abs(SdfDouble(_sdf, "momentConstantWater", this->momentConstantWater_));
    this->maxThrust_ = SdfDouble(_sdf, "maxThrust", this->maxThrust_);
    this->maxTorque_ = SdfDouble(_sdf, "maxTorque", this->maxTorque_);
    this->surfaceZ_ = SdfDouble(_sdf, "surface_z", this->surfaceZ_);
    this->transitionBand_ =
        SdfDouble(_sdf, "transition_band", this->transitionBand_);
    this->submergenceFilterAlpha_ =
        std::clamp(SdfDouble(_sdf, "submergenceFilterAlpha",
                             this->submergenceFilterAlpha_),
                   0.0, 0.999999);

    if (_sdf->HasElement("thrustAxis")) {
      this->thrustAxis_ = _sdf->Get<gz::math::Vector3d>("thrustAxis");
      if (this->thrustAxis_.Length() > 1e-9) {
        this->thrustAxis_.Normalize();
      } else {
        this->thrustAxis_ = gz::math::Vector3d(0, 0, 1);
      }
    }

    auto mutableSdf = _sdf ? _sdf->Clone() : sdf::ElementPtr();
    if (mutableSdf && mutableSdf->HasElement("sample_point")) {
      for (auto pointElement = mutableSdf->GetElement("sample_point");
           pointElement != nullptr;
           pointElement = pointElement->GetNextElement("sample_point")) {
        this->samplePoints_.push_back(pointElement->Get<gz::math::Vector3d>());
      }
    }
    if (this->samplePoints_.empty()) {
      this->samplePoints_.push_back(gz::math::Vector3d::Zero);
    }

    if (this->turningDirection_ == "ccw" || this->turningDirection_ == "CCW") {
      this->turnDirSign_ = 1.0;
    } else if (this->turningDirection_ == "cw" ||
               this->turningDirection_ == "CW") {
      this->turnDirSign_ = -1.0;
    } else {
      gzerr << "[HybridAirPropellerModel] Unknown turningDirection: "
            << this->turningDirection_ << ", defaulting to ccw" << std::endl;
      this->turnDirSign_ = 1.0;
    }

    if (this->timeConstantUp_ <= 0.0 || this->timeConstantDown_ <= 0.0) {
      throw std::invalid_argument(
          "HybridAirPropellerModel time constants must be positive");
    }
    if (this->maxRotVelocity_ <= 0.0) {
      throw std::invalid_argument(
          "HybridAirPropellerModel maxRotVelocity must be positive");
    }

    const auto jointEntity = this->model_.JointByName(_ecm, this->jointName_);
    if (jointEntity == kNullEntity) {
      throw std::runtime_error("Joint not found: " + this->jointName_);
    }
    this->joint_ = Joint(jointEntity);

    const auto linkEntity = this->model_.LinkByName(_ecm, this->linkName_);
    if (linkEntity == kNullEntity) {
      throw std::runtime_error("Link not found: " + this->linkName_);
    }
    this->link_ = Link(linkEntity);

    const auto axisFrameEntity =
        this->model_.LinkByName(_ecm, this->axisFrameLinkName_);
    if (axisFrameEntity == kNullEntity) {
      throw std::runtime_error("Axis frame link not found: " +
                               this->axisFrameLinkName_);
    }
    this->axisFrameLink_ = Link(axisFrameEntity);

    this->node_ = std::make_unique<gz::transport::Node>();
    std::string commandTopic = this->commandTopic_;
    if (!commandTopic.empty() && commandTopic[0] != '/') {
      commandTopic = "/" + this->model_.Name(_ecm) + "/" + commandTopic;
    }
    if (!this->node_->Subscribe(commandTopic,
                                &HybridAirPropellerModel::OnActuators, this)) {
      throw std::runtime_error("Failed to subscribe: " + commandTopic);
    }

    if (!this->diagnosticTopic_.empty()) {
      std::string topic = this->diagnosticTopic_;
      if (topic[0] != '/') {
        topic = "/model/" + this->model_.Name(_ecm) + "/" + topic;
      }
      this->diagnosticPub_ = this->node_->Advertise<gz::msgs::Float_V>(topic);
      this->diagnosticsEnabled_ = true;
    }

    this->lastCmd_.assign(
        static_cast<std::size_t>(std::max(1, this->motorNumber_ + 1)), 0.0);
    this->filteredOmega_.store(0.0);
    this->filteredSubmergence_ = 0.0;
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override {
    GZ_PROFILE("HybridAirPropellerModel::PreUpdate");

    if (_info.paused || !this->link_.Valid(_ecm) ||
        !this->axisFrameLink_.Valid(_ecm) || !this->joint_.Valid(_ecm)) {
      return;
    }

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0) {
      return;
    }

    double rawCmd = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->cmdMutex_);
      if (!this->hasCommand_ || this->lastCmd_.empty()) {
        return;
      }
      const std::size_t index = static_cast<std::size_t>(std::clamp(
          this->motorNumber_, 0, static_cast<int>(this->lastCmd_.size() - 1)));
      rawCmd = this->lastCmd_[index];
    }
    if (!std::isfinite(rawCmd)) {
      return;
    }

    const double omegaRef = this->CommandToOmega(rawCmd);
    const double currentOmega = this->filteredOmega_.load();
    const double tau =
        (omegaRef >= currentOmega) ? this->timeConstantUp_ : this->timeConstantDown_;
    const double alpha = tau > 1e-9 ? 1.0 - std::exp(-dt / tau) : 1.0;
    const double omega = currentOmega + (omegaRef - currentOmega) * alpha;
    this->filteredOmega_.store(omega);
    this->joint_.SetVelocity(_ecm, {this->turnDirSign_ * omega});

    const double submergence = this->ComputeSubmergence(_ecm);
    if (!this->submergenceInitialized_) {
      this->filteredSubmergence_ = submergence;
      this->submergenceInitialized_ = true;
    } else {
      this->filteredSubmergence_ =
          this->submergenceFilterAlpha_ * this->filteredSubmergence_ +
          (1.0 - this->submergenceFilterAlpha_) * submergence;
    }

    const double signedOmega2 =
        (this->bidirectional_ && omega < 0.0 ? -1.0 : 1.0) * omega * omega;
    const double thrustAir = this->motorConstantAir_ * signedOmega2;
    const double thrustWater = this->motorConstantWater_ * signedOmega2;
    double thrust =
        this->filteredSubmergence_ * thrustWater +
        (1.0 - this->filteredSubmergence_) * thrustAir;

    const double torqueAir =
        -this->turnDirSign_ * this->momentConstantAir_ * signedOmega2;
    const double torqueWater =
        -this->turnDirSign_ * this->momentConstantWater_ * signedOmega2;
    double torque =
        this->filteredSubmergence_ * torqueWater +
        (1.0 - this->filteredSubmergence_) * torqueAir;

    thrust = std::clamp(thrust, -this->maxThrust_, this->maxThrust_);
    torque = std::clamp(torque, -this->maxTorque_, this->maxTorque_);
    if (!std::isfinite(thrust) || !std::isfinite(torque)) {
      return;
    }

    const auto axisFramePose = this->axisFrameLink_.WorldPose(_ecm);
    if (!axisFramePose.has_value()) {
      return;
    }
    const auto axisWorld = axisFramePose->Rot().RotateVector(this->thrustAxis_);
    if (!axisWorld.IsFinite()) {
      return;
    }

    this->link_.AddWorldWrench(_ecm, axisWorld * thrust, axisWorld * torque);
    this->PublishDiagnostics(rawCmd, omega, thrust, torque,
                             this->filteredSubmergence_);
  }

 private:
  void OnActuators(const gz::msgs::Actuators &_msg) {
    std::lock_guard<std::mutex> lock(this->cmdMutex_);
    this->lastCmd_.resize(_msg.velocity_size(), 0.0);
    for (int i = 0; i < _msg.velocity_size(); ++i) {
      this->lastCmd_[static_cast<std::size_t>(i)] = _msg.velocity(i);
    }
    this->hasCommand_ = true;
  }

  double CommandToOmega(const double _cmd) const {
    if (this->normalizedCommand_) {
      const double lo = this->bidirectional_ ? -1.0 : 0.0;
      return std::clamp(_cmd, lo, 1.0) * this->maxRotVelocity_;
    }
    const double lo = this->bidirectional_ ? -this->maxRotVelocity_ : 0.0;
    return std::clamp(_cmd, lo, this->maxRotVelocity_);
  }

  double SampleSubmergence(const double _worldZ) const {
    if (this->transitionBand_ <= 1e-6) {
      return _worldZ <= this->surfaceZ_ ? 1.0 : 0.0;
    }

    const double lower = this->surfaceZ_ - 0.5 * this->transitionBand_;
    const double upper = this->surfaceZ_ + 0.5 * this->transitionBand_;
    if (_worldZ <= lower) {
      return 1.0;
    }
    if (_worldZ >= upper) {
      return 0.0;
    }
    return (upper - _worldZ) / (upper - lower);
  }

  double ComputeSubmergence(const EntityComponentManager &_ecm) const {
    const auto pose = this->link_.WorldPose(_ecm);
    if (!pose.has_value()) {
      return 0.0;
    }

    double ratio = 0.0;
    for (const auto &point : this->samplePoints_) {
      const auto worldPoint = pose->Pos() + pose->Rot().RotateVector(point);
      ratio += this->SampleSubmergence(worldPoint.Z());
    }
    return std::clamp(ratio / static_cast<double>(this->samplePoints_.size()),
                      0.0, 1.0);
  }

  void PublishDiagnostics(const double _cmd, const double _omega,
                          const double _thrust, const double _torque,
                          const double _submergence) {
    if (!this->diagnosticsEnabled_) {
      return;
    }
    gz::msgs::Float_V msg;
    msg.add_data(static_cast<float>(_cmd));
    msg.add_data(static_cast<float>(_omega));
    msg.add_data(static_cast<float>(_thrust));
    msg.add_data(static_cast<float>(_torque));
    msg.add_data(static_cast<float>(_submergence));
    msg.add_data(static_cast<float>(this->DomainCode(_submergence)));
    this->diagnosticPub_.Publish(msg);
  }

  int DomainCode(const double _submergence) const {
    if (_submergence <= 0.05) {
      return 0;
    }
    if (_submergence >= 0.95) {
      return 2;
    }
    return 1;
  }

  Model model_{kNullEntity};
  Link link_{kNullEntity};
  Link axisFrameLink_{kNullEntity};
  Joint joint_{kNullEntity};

  std::string jointName_;
  std::string linkName_;
  std::string axisFrameLinkName_;
  std::string turningDirection_{"ccw"};
  std::string commandTopic_{"/teleh4z_zaxis/command/motor_speed"};
  std::string diagnosticTopic_{"hybrid_air_propeller_0/state"};
  gz::math::Vector3d thrustAxis_{0, 0, 1};
  std::vector<gz::math::Vector3d> samplePoints_;

  int motorNumber_{0};
  double turnDirSign_{1.0};
  bool normalizedCommand_{false};
  bool bidirectional_{true};
  double timeConstantUp_{0.015};
  double timeConstantDown_{0.03};
  double maxRotVelocity_{1585.0};
  double motorConstantAir_{1.64e-5};
  double momentConstantAir_{0.018};
  double motorConstantWater_{1.0e-4};
  double momentConstantWater_{0.018};
  double maxThrust_{50.0};
  double maxTorque_{5.0};
  double surfaceZ_{0.0};
  double transitionBand_{0.08};
  double submergenceFilterAlpha_{0.95};

  std::atomic<double> filteredOmega_{0.0};
  double filteredSubmergence_{0.0};
  bool submergenceInitialized_{false};
  std::vector<double> lastCmd_;
  std::unique_ptr<gz::transport::Node> node_;
  gz::transport::Node::Publisher diagnosticPub_;
  bool diagnosticsEnabled_{false};
  std::mutex cmdMutex_;
  bool hasCommand_{false};
};

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::HybridAirPropellerModel, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::HybridAirPropellerModel,
                    "teleaqua_gz_plugins::HybridAirPropellerModel")
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::HybridAirPropellerModel,
                    "gz::sim::systems::HybridAirPropellerModel")
