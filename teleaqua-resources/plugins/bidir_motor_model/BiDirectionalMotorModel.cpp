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
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace gz::sim::systems {

class BiDirectionalMotorModel : public System,
                                public ISystemConfigure,
                                public ISystemPreUpdate {
 public:
  BiDirectionalMotorModel() = default;
  ~BiDirectionalMotorModel() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override {
    this->model_ = Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "[BiDirectionalMotorModel] Invalid model entity" << std::endl;
      return;
    }

    this->jointName_ = _sdf->Get<std::string>("jointName", "").first;
    this->linkName_ = _sdf->Get<std::string>("linkName", "").first;
    this->turningDirection_ =
        _sdf->Get<std::string>("turningDirection", "ccw").first;
    this->commandTopic_ =
        _sdf->Get<std::string>(commandTopicKey, this->commandTopic_).first;
    this->motorNumber_ =
        _sdf->Get<int>(motorNumberKey, this->motorNumber_).first;

    this->commandGain_ = _sdf->Get<double>("commandGain", this->commandGain_).first;
    this->deadbandOmega_ =
        _sdf->Get<double>("deadbandOmega", this->deadbandOmega_).first;
    this->maxOmega_ = _sdf->Get<double>("maxOmega", this->maxOmega_).first;
    this->maxThrust_ = _sdf->Get<double>("maxThrust", this->maxThrust_).first;
    this->maxTorque_ = _sdf->Get<double>("maxTorque", this->maxTorque_).first;
    this->SIM_GZ_SV_TRIM_ =
        _sdf->Get<double>("SIM_GZ_SV_TRIM", this->SIM_GZ_SV_TRIM_).first;

    this->surface_z_ = _sdf->Get<double>("surface_z", this->surface_z_).first;
    this->transition_band_ =
        _sdf->Get<double>("transition_band", this->transition_band_).first;
    this->fluid_density_water_ =
        _sdf->Get<double>("fluid_density_water", this->fluid_density_water_)
            .first;
    this->fluid_density_air_ =
        _sdf->Get<double>("fluid_density_air", this->fluid_density_air_).first;

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
      gzerr << "[BiDirectionalMotorModel] Unknown turningDirection: "
            << this->turningDirection_ << ", defaulting to ccw" << std::endl;
      this->turnDirSign_ = 1.0;
    }

    this->timeConstantUp_ =
        _sdf->Get<double>(timeConstantUpKey, this->timeConstantUp_).first;
    this->timeConstantDown_ =
        _sdf->Get<double>(timeConstantDownKey, this->timeConstantDown_).first;
    if (this->timeConstantUp_ <= 0.0 || this->timeConstantDown_ <= 0.0) {
      throw std::invalid_argument(
          "Time constants for BiDirectionalMotorModel must be positive");
    }

    this->motorConstant_ =
        std::abs(_sdf->Get<double>(motorConstantKey, this->motorConstant_).first);
    this->momentConstant_ =
        std::abs(_sdf->Get<double>(momentConstantKey, this->momentConstant_).first);
    if (this->motorConstant_ <= 0.0 || this->momentConstant_ <= 0.0) {
      throw std::invalid_argument(
          "Motor and moment constants for BiDirectionalMotorModel must be non-zero");
    }

    this->forwardCoeff_ =
        _sdf->Get<double>(forwardCoeffKey, this->forwardCoeff_).first;
    this->reverseCoeff_ =
        _sdf->Get<double>(reverseCoeffKey, this->reverseCoeff_).first;
    if (this->forwardCoeff_ <= 0.0 || this->reverseCoeff_ <= 0.0) {
      throw std::invalid_argument(
          "Forward and reverse coefficients must be positive");
    }

    auto jointEntity = this->model_.JointByName(_ecm, this->jointName_);
    if (jointEntity == kNullEntity) {
      throw std::runtime_error("Joint not found: " + this->jointName_);
    }
    this->joint_ = Joint(jointEntity);

    auto linkEntity = this->model_.LinkByName(_ecm, this->linkName_);
    if (linkEntity == kNullEntity) {
      throw std::runtime_error("Link not found: " + this->linkName_);
    }
    this->link_ = Link(linkEntity);

    if (_sdf->HasElement("thrustAxis")) {
      this->thrustAxis_ =
          _sdf->Get<gz::math::Vector3d>("thrustAxis").Normalized();
    } else {
      this->thrustAxis_ = gz::math::Vector3d(0, 0, 1);
    }

    this->node_ = std::make_unique<gz::transport::Node>();
    std::string topic = this->commandTopic_;
    if (!topic.empty() && topic[0] != '/') {
      topic = "/" + this->model_.Name(_ecm) + "/" + topic;
    }

    if (!this->node_->Subscribe(topic, &BiDirectionalMotorModel::OnActuators,
                                this)) {
      throw std::runtime_error("Failed to subscribe: " + topic);
    }

    this->filteredCmd_.store(0.0);
    this->hasCommand_ = false;
    this->commandStateInitialized_ = false;
    this->lastCmd_.assign(static_cast<std::size_t>(
                              std::max(1, this->motorNumber_ + 1)),
                          0.0);
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override {
    GZ_PROFILE("BiDirectionalMotorModel::PreUpdate");

    if (!this->link_.Valid(_ecm) || !this->joint_.Valid(_ecm)) {
      return;
    }

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0) {
      return;
    }

    double rawCmd = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->cmdMutex_);
      if (this->lastCmd_.empty() || !this->hasCommand_) {
        return;
      }
      const std::size_t index =
          static_cast<std::size_t>(std::clamp(this->motorNumber_, 0,
                                              static_cast<int>(this->lastCmd_.size() - 1)));
      rawCmd = this->lastCmd_[index];
    }

    const double pwmRef =
        (this->SIM_GZ_SV_TRIM_ == 0.0)
            ? rawCmd * this->commandGain_
            : (rawCmd - this->SIM_GZ_SV_TRIM_) * this->commandGain_;
    if (!std::isfinite(pwmRef)) {
      return;
    }

    if (!this->commandStateInitialized_) {
      // Seed the filter from the first real servo command to avoid startup thrust transients.
      this->filteredCmd_.store(pwmRef);
      this->commandStateInitialized_ = true;
    } else {
      const double currentCmd = this->filteredCmd_.load();
      const double tau =
          (pwmRef >= currentCmd) ? this->timeConstantUp_ : this->timeConstantDown_;
      if (tau > 1e-6) {
        const double alpha = 1.0 - std::exp(-dt / tau);
        this->filteredCmd_.store(currentCmd + (pwmRef - currentCmd) * alpha);
      } else {
        this->filteredCmd_.store(pwmRef);
      }
    }

    const double pwmNow = this->filteredCmd_.load();
    if (!std::isfinite(pwmNow)) {
      return;
    }

    double thrustMag = this->ComputeThrustFromPwm(pwmNow);
    double torqueMag = 0.0;

    const double submergence = this->ComputeSubmergence(_ecm);
    const double effectiveDensity =
        this->fluid_density_air_ +
        submergence * (this->fluid_density_water_ - this->fluid_density_air_);
    const double densityScale =
        this->fluid_density_water_ > 1e-6
            ? effectiveDensity / this->fluid_density_water_
            : 1.0;
    const double mediumScale =
        std::clamp(submergence * densityScale, 0.0, 1.0);

    thrustMag *= mediumScale;
    torqueMag *= mediumScale;

    if (!std::isfinite(thrustMag) || !std::isfinite(torqueMag)) {
      return;
    }

    thrustMag = std::clamp(thrustMag, -this->maxThrust_, this->maxThrust_);
    torqueMag = std::clamp(torqueMag, -this->maxTorque_, this->maxTorque_);
    if (std::abs(thrustMag) < 1e-6 && std::abs(torqueMag) < 1e-6) {
      return;
    }

    const auto pose = this->link_.WorldPose(_ecm);
    if (!pose.has_value()) {
      return;
    }

    const auto axis = pose->Rot().RotateVector(this->thrustAxis_);
    const auto thrustWorld = axis * thrustMag;
    const auto torqueWorld = axis * torqueMag;
    if (!thrustWorld.IsFinite() || !torqueWorld.IsFinite()) {
      return;
    }

    this->link_.AddWorldWrench(_ecm, thrustWorld, torqueWorld);
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

  double ComputeThrustFromPwm(double _pwm) const {
    if (!std::isfinite(_pwm)) {
      return 0.0;
    }

    // Calibrated asymmetric PWM-to-thrust map with a central deadband.
    if (_pwm > 1435.0 && _pwm < 1530.0) {
      return 0.0;
    }

    if (_pwm >= 1000.0 && _pwm <= 1435.0) {
      const double x = (1500.0 - _pwm) / 500.0;
      return 5.75 * x * x + 3.92 * x - 0.63;
    }

    if (_pwm >= 1530.0 && _pwm <= 2000.0) {
      const double x = (_pwm - 1500.0) / 500.0;
      return -(2.53 * x * x + 4.25 * x - 0.31);
    }

    return 0.0;
  }

  double SampleSubmergence(double _worldZ) const {
    if (this->transition_band_ <= 1e-6) {
      return _worldZ <= this->surface_z_ ? 1.0 : 0.0;
    }

    const double lower = this->surface_z_ - 0.5 * this->transition_band_;
    const double upper = this->surface_z_ + 0.5 * this->transition_band_;
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

    return std::clamp(ratio / this->samplePoints_.size(), 0.0, 1.0);
  }

  static constexpr char timeConstantUpKey[] = "timeConstantUp";
  static constexpr char timeConstantDownKey[] = "timeConstantDown";
  static constexpr char motorConstantKey[] = "motorConstant";
  static constexpr char momentConstantKey[] = "momentConstant";
  static constexpr char commandTopicKey[] = "commandSubTopic";
  static constexpr char motorNumberKey[] = "motorNumber";
  static constexpr char forwardCoeffKey[] = "forwardCoeff";
  static constexpr char reverseCoeffKey[] = "reverseCoeff";

  Model model_{kNullEntity};
  Link link_{kNullEntity};
  Joint joint_{kNullEntity};

  std::string jointName_;
  std::string linkName_;
  std::string turningDirection_{"ccw"};
  std::string commandTopic_{"command/motor_speed"};
  gz::math::Vector3d thrustAxis_{0, 0, 1};
  std::vector<gz::math::Vector3d> samplePoints_;

  double timeConstantUp_{0.0427}; // 加速时间常数
  double timeConstantDown_{0.0427}; // 减速时间常数
  double motorConstant_{2e-5};
  double momentConstant_{0.016};
  double forwardCoeff_{1.0};
  double reverseCoeff_{0.7};
  int motorNumber_{0};
  double turnDirSign_{1.0};
  double commandGain_{1.0};
  double deadbandOmega_{1e-3};
  double maxOmega_{5000.0};
  double maxThrust_{1e3};
  double maxTorque_{1e3};
  double SIM_GZ_SV_TRIM_{0.0};

  double surface_z_{0.0};
  double transition_band_{0.1};
  double fluid_density_water_{1000.0};
  double fluid_density_air_{1.225};

  std::atomic<double> filteredCmd_{0.0};
  std::vector<double> lastCmd_;
  std::unique_ptr<gz::transport::Node> node_;
  std::mutex cmdMutex_;
  bool hasCommand_{false};
  bool commandStateInitialized_{false};
};

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::BiDirectionalMotorModel, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::BiDirectionalMotorModel,
                    "px4_gz_plugins::BiDirectionalMotorModel")
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::BiDirectionalMotorModel,
                    "gz::sim::v8::systems::BiDirectionalMotorModel")
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::BiDirectionalMotorModel,
                    "gz::sim::systems::BiDirectionalMotorModel")
