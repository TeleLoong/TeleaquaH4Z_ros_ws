import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleAirWaterStatus, VehicleAirWaterStatusSetpoint
from ros_gz_interfaces.msg import Float32Array
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class State(Enum):
    IDLE_WATER = auto()
    DEPLOYING_ARMS = auto()
    WAITING_FOR_AIR_CLEARANCE = auto()
    SWITCHING_TO_AIR = auto()
    IDLE_AIR = auto()
    SWITCHING_TO_WATER = auto()
    RETRACTING_ARMS = auto()


ARM_RETRACTED = 0.0
ARM_DEPLOYED = math.pi / 2

ARM_POSITION_TOLERANCE_RAD = 0.02
ARM_VELOCITY_TOLERANCE_RAD_S = 0.05
ARM_SETTLE_DURATION_SEC = 0.25
ARM_MOTION_TIMEOUT_SEC = 8.0
PX4_SWITCH_TIMEOUT_SEC = 5.0
SETPOINT_PUBLISH_RATE_HZ = 5.0
ARM_TRAJECTORY_TOPIC = '/arm_trajectory'


class TeleH4ZModeManager(Node):

    def __init__(self):
        super().__init__('teleh4z_mode_manager')

        self.declare_parameter('arm_move_duration', 2.0)
        self.declare_parameter('arm_settle_duration', ARM_SETTLE_DURATION_SEC)
        self.declare_parameter('arm_motion_timeout', ARM_MOTION_TIMEOUT_SEC)
        self.declare_parameter('px4_switch_timeout', PX4_SWITCH_TIMEOUT_SEC)
        self.declare_parameter('model_name', 'teleh4z_0')
        self.declare_parameter('world_name', 'playground')

        self._arm_move_duration = float(self.get_parameter('arm_move_duration').value)
        self._arm_settle_duration = float(self.get_parameter('arm_settle_duration').value)
        self._arm_motion_timeout = float(self.get_parameter('arm_motion_timeout').value)
        self._px4_switch_timeout = float(self.get_parameter('px4_switch_timeout').value)
        self._model_name = str(self.get_parameter('model_name').value)
        self._world_name = str(self.get_parameter('world_name').value)
        self._joint_state_topic = f'/world/{self._world_name}/model/{self._model_name}/joint_state'
        self._transition_state_topic = f'/model/{self._model_name}/transition_state'

        self._state = State.IDLE_WATER
        self._px4_mode: int = 1
        self._px4_status_received = False
        self._target_px4_mode: int = 1
        self._arm_target = None
        self._arm_settle_start_ns = None

        self._arm_left_pos = None
        self._arm_left_vel = None
        self._arm_right_pos = None
        self._arm_right_vel = None

        self._transition_state = [0.0] * 6
        self._transition_state_received = False

        self._setpoint_timer = None
        self._timeout_timer = None
        self._arm_timeout_timer = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub_status = self.create_subscription(
            VehicleAirWaterStatus,
            '/fmu/out/vehicle_air_water_status',
            self._on_px4_status,
            px4_qos,
        )

        self._sub_request = self.create_subscription(
            String,
            '/teleh4z/mode_request',
            self._on_mode_request,
            10,
        )

        self._sub_joint_state = self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._on_joint_state,
            10,
        )

        self._sub_transition_state = self.create_subscription(
            Float32Array,
            self._transition_state_topic,
            self._on_transition_state,
            10,
        )

        self._pub_setpoint = self.create_publisher(
            VehicleAirWaterStatusSetpoint,
            '/fmu/in/vehicle_air_water_status_setpoint',
            px4_qos,
        )

        self._pub_arm_trajectory = self.create_publisher(
            JointTrajectory,
            ARM_TRAJECTORY_TOPIC,
            10,
        )

        self.get_logger().info(
            'TeleH4Z Mode Manager started. '
            f'Arm trajectory topic: {ARM_TRAJECTORY_TOPIC}, '
            f'Joint state topic: {self._joint_state_topic}, '
            f'Transition state topic: {self._transition_state_topic}, '
            f'Initial state: {self._state.name}'
        )

    def _on_px4_status(self, msg: VehicleAirWaterStatus):
        prev = self._px4_mode
        self._px4_mode = msg.uw_transit_mode
        self._px4_status_received = True

        if prev != self._px4_mode:
            mode_str = 'WATER' if self._px4_mode == 1 else 'AIR'
            self.get_logger().info(f'PX4 mode changed: {mode_str} (uw_transit_mode={self._px4_mode})')

        if self._state == State.SWITCHING_TO_AIR and self._px4_mode == 0:
            self._on_px4_confirmed_air()
        elif self._state == State.SWITCHING_TO_WATER and self._px4_mode == 1:
            self._on_px4_confirmed_water()

    def _on_joint_state(self, msg: JointState):
        names = list(msg.name)

        def value_for(joint_name, values):
            if joint_name in names:
                idx = names.index(joint_name)
                if idx < len(values):
                    return values[idx]
            return None

        self._arm_left_pos = value_for('arm_left_joint', list(msg.position))
        self._arm_right_pos = value_for('arm_right_joint', list(msg.position))
        self._arm_left_vel = value_for('arm_left_joint', list(msg.velocity))
        self._arm_right_vel = value_for('arm_right_joint', list(msg.velocity))

        if self._state in (State.DEPLOYING_ARMS, State.RETRACTING_ARMS):
            self._check_arm_completion()

    def _on_transition_state(self, msg: Float32Array):
        data = list(msg.data)
        if len(data) < 6:
            data.extend([0.0] * (6 - len(data)))
        self._transition_state = data[:6]
        self._transition_state_received = True

        if self._state == State.WAITING_FOR_AIR_CLEARANCE and self._air_clearance_ok():
            self.get_logger().info(
                'Air propellers are clear of the water surface. Requesting PX4 AIR mode...'
            )
            self._request_px4_air_mode()

    def _on_mode_request(self, msg: String):
        request = msg.data.strip().lower()

        if request not in ('air', 'water'):
            self.get_logger().warn(f'Invalid mode request: {msg.data}. Use air or water.')
            return

        if self._state not in (State.IDLE_WATER, State.IDLE_AIR):
            self.get_logger().warn(
                f'Transition in progress ({self._state.name}), ignoring {request} request.'
            )
            return

        if request == 'air' and self._state == State.IDLE_AIR:
            self.get_logger().info('Already in AIR mode, ignoring.')
            return
        if request == 'water' and self._state == State.IDLE_WATER:
            self.get_logger().info('Already in WATER mode, ignoring.')
            return

        if request == 'air':
            self._begin_water_to_air()
        else:
            self._begin_air_to_water()

    def _begin_water_to_air(self):
        self.get_logger().info('=== Transition: WATER -> AIR ===')
        self._state = State.DEPLOYING_ARMS
        self._arm_target = ARM_DEPLOYED
        self._arm_settle_start_ns = None
        self.get_logger().info(
            f'DEPLOYING_ARMS: publish arm trajectory to {ARM_DEPLOYED:.4f} rad '
            f'over {self._arm_move_duration:.2f}s'
        )
        self._publish_arm_trajectory(ARM_DEPLOYED)
        self._start_arm_timeout()

    def _on_arms_deployed(self):
        self._cancel_timer('_arm_timeout_timer')
        self._arm_target = None
        self._arm_settle_start_ns = None

        self._state = State.WAITING_FOR_AIR_CLEARANCE
        self._target_px4_mode = 0

        if self._air_clearance_ok():
            self.get_logger().info('Arms deployed and air props already clear of the water surface.')
            self._request_px4_air_mode()
            return

        if not self._transition_state_received:
            self.get_logger().warn(
                'Arms deployed, but transition_state has not been received yet. Waiting for clearance telemetry...'
            )
        else:
            self.get_logger().info(
                'Arms deployed. Waiting for air propellers to clear the water surface before switching PX4 to AIR mode...'
            )

        self._cancel_timer('_timeout_timer')
        self._timeout_timer = self.create_timer(
            self._px4_switch_timeout,
            self._on_switch_timeout,
        )

    def _request_px4_air_mode(self):
        self._cancel_timer('_timeout_timer')
        self._state = State.SWITCHING_TO_AIR
        self._target_px4_mode = 0
        self.get_logger().info('Requesting PX4 switch to AIR mode...')

        self._publish_setpoint(uw_transit_mode=0)
        self._setpoint_timer = self.create_timer(
            1.0 / SETPOINT_PUBLISH_RATE_HZ,
            lambda: self._publish_setpoint(uw_transit_mode=0),
        )
        self._timeout_timer = self.create_timer(
            self._px4_switch_timeout,
            self._on_switch_timeout,
        )

        if self._px4_mode == 0:
            self._on_px4_confirmed_air()

    def _on_px4_confirmed_air(self):
        self._cancel_timer('_setpoint_timer')
        self._cancel_timer('_timeout_timer')
        self._state = State.IDLE_AIR
        self.get_logger().info('=== IDLE_AIR: transition complete ===')

    def _begin_air_to_water(self):
        self.get_logger().info('=== Transition: AIR -> WATER ===')
        self._state = State.SWITCHING_TO_WATER
        self._target_px4_mode = 1
        self.get_logger().info('Requesting PX4 switch to WATER mode (air propulsion will stop, water propulsion is immersion-gated)...')

        self._publish_setpoint(uw_transit_mode=1)
        self._setpoint_timer = self.create_timer(
            1.0 / SETPOINT_PUBLISH_RATE_HZ,
            lambda: self._publish_setpoint(uw_transit_mode=1),
        )
        self._timeout_timer = self.create_timer(
            self._px4_switch_timeout,
            self._on_switch_timeout,
        )

        if self._px4_mode == 1:
            self._on_px4_confirmed_water()

    def _on_px4_confirmed_water(self):
        self._cancel_timer('_setpoint_timer')
        self._cancel_timer('_timeout_timer')

        self._state = State.RETRACTING_ARMS
        self._arm_target = ARM_RETRACTED
        self._arm_settle_start_ns = None
        self.get_logger().info(
            f'PX4 in WATER mode. RETRACTING_ARMS: publish arm trajectory to {ARM_RETRACTED:.4f} rad '
            f'over {self._arm_move_duration:.2f}s'
        )
        self._publish_arm_trajectory(ARM_RETRACTED)
        self._start_arm_timeout()

    def _on_arms_retracted(self):
        self._cancel_timer('_arm_timeout_timer')
        self._arm_target = None
        self._arm_settle_start_ns = None
        self._state = State.IDLE_WATER
        self.get_logger().info('=== IDLE_WATER: transition complete ===')

    def _check_arm_completion(self):
        if self._arm_target is None:
            return
        if None in (
            self._arm_left_pos,
            self._arm_right_pos,
            self._arm_left_vel,
            self._arm_right_vel,
        ):
            return

        pos_ok = (
            abs(self._arm_left_pos - self._arm_target) < ARM_POSITION_TOLERANCE_RAD and
            abs(self._arm_right_pos - self._arm_target) < ARM_POSITION_TOLERANCE_RAD
        )
        vel_ok = (
            abs(self._arm_left_vel) < ARM_VELOCITY_TOLERANCE_RAD_S and
            abs(self._arm_right_vel) < ARM_VELOCITY_TOLERANCE_RAD_S
        )

        if pos_ok and vel_ok:
            now_ns = self.get_clock().now().nanoseconds
            if self._arm_settle_start_ns is None:
                self._arm_settle_start_ns = now_ns
                return

            settled_sec = (now_ns - self._arm_settle_start_ns) / 1e9
            if settled_sec >= self._arm_settle_duration:
                if self._state == State.DEPLOYING_ARMS:
                    self._on_arms_deployed()
                elif self._state == State.RETRACTING_ARMS:
                    self._on_arms_retracted()
        else:
            self._arm_settle_start_ns = None

    def _start_arm_timeout(self):
        self._cancel_timer('_arm_timeout_timer')
        self._arm_timeout_timer = self.create_timer(
            self._arm_motion_timeout,
            self._on_arm_motion_timeout,
        )

    def _on_arm_motion_timeout(self):
        self._cancel_timer('_arm_timeout_timer')
        target_str = 'DEPLOYED' if self._arm_target == ARM_DEPLOYED else 'RETRACTED'
        self.get_logger().error(f'TIMEOUT waiting for arm trajectory to reach {target_str}.')

        if self._state == State.DEPLOYING_ARMS:
            self.get_logger().warn('Arm deploy failed; returning to IDLE_WATER without requesting AIR mode.')
            self._state = State.IDLE_WATER
        elif self._state == State.RETRACTING_ARMS:
            self.get_logger().warn('Arm retract failed after PX4 entered WATER mode; keeping IDLE_WATER with warning.')
            self._state = State.IDLE_WATER

        self._arm_target = None
        self._arm_settle_start_ns = None

    def _on_switch_timeout(self):
        self._cancel_timer('_setpoint_timer')
        self._cancel_timer('_timeout_timer')

        if self._state == State.WAITING_FOR_AIR_CLEARANCE:
            self.get_logger().error(
                'TIMEOUT waiting for air propellers to clear the water surface. Rolling back to WATER configuration.'
            )
            self._state = State.RETRACTING_ARMS
            self._arm_target = ARM_RETRACTED
            self._arm_settle_start_ns = None
            self._publish_arm_trajectory(ARM_RETRACTED)
            self._start_arm_timeout()
            return

        target_str = 'AIR' if self._target_px4_mode == 0 else 'WATER'
        self.get_logger().error(
            f'TIMEOUT waiting for PX4 to confirm {target_str} mode! Rolling back to safe state.'
        )

        if self._state == State.SWITCHING_TO_AIR:
            self.get_logger().warn('Rolling back: retracting arms, staying in WATER mode.')
            self._state = State.RETRACTING_ARMS
            self._arm_target = ARM_RETRACTED
            self._arm_settle_start_ns = None
            self._publish_arm_trajectory(ARM_RETRACTED)
            self._start_arm_timeout()
        elif self._state == State.SWITCHING_TO_WATER:
            self.get_logger().warn('Rolling back: staying in AIR mode.')
            self._state = State.IDLE_AIR

    def _publish_arm_trajectory(self, target: float):
        msg = JointTrajectory()
        msg.joint_names = ['arm_left_joint', 'arm_right_joint']

        point = JointTrajectoryPoint()
        point.positions = [target, target]
        sec = int(self._arm_move_duration)
        nanosec = int((self._arm_move_duration - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        msg.points = [point]

        self._pub_arm_trajectory.publish(msg)

    def _publish_setpoint(self, uw_transit_mode: int):
        msg = VehicleAirWaterStatusSetpoint()
        msg.timestamp = self._get_timestamp()
        msg.uw_transit_mode = uw_transit_mode
        msg.transform_status = 0
        self._pub_setpoint.publish(msg)

    def _air_clearance_ok(self) -> bool:
        return self._transition_state_received and len(self._transition_state) >= 6 and self._transition_state[5] >= 0.5

    def _get_timestamp(self) -> int:
        now = self.get_clock().now()
        return now.nanoseconds // 1000

    def _cancel_timer(self, attr_name: str):
        timer = getattr(self, attr_name, None)
        if timer is not None:
            timer.cancel()
            timer.destroy()
            setattr(self, attr_name, None)


def main(args=None):
    rclpy.init(args=args)
    node = TeleH4ZModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
