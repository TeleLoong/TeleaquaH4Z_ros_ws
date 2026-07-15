from math import isfinite

import rclpy
from px4_msgs.msg import ManualControlSetpoint, VehicleAirWaterStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


PWM_NEUTRAL_US = 1500.0


class TeleH4ZManualWaterThrusterMapper(Node):

    def __init__(self):
        super().__init__('teleh4z_manual_water_thruster_mapper')

        self.declare_parameter('manual_control_topic', '/fmu/out/manual_control_setpoint')
        self.declare_parameter('vehicle_air_water_status_topic', '/fmu/out/vehicle_air_water_status')
        self.declare_parameter('water_thruster_pwm_topic', '/teleh4z/water_thruster_pwm')
        self.declare_parameter('require_water_mode', True)
        self.declare_parameter('require_valid_manual_control', True)
        self.declare_parameter('deadband', 0.08)
        self.declare_parameter('surge_axis', 'pitch')
        self.declare_parameter('yaw_axis', 'yaw')
        self.declare_parameter('heave_axis', 'throttle')
        self.declare_parameter('pitch_axis', 'roll')
        self.declare_parameter('surge_pwm_scale', -280.0)
        self.declare_parameter('yaw_pwm_scale', 180.0)
        self.declare_parameter('heave_pwm_scale', 260.0)
        self.declare_parameter('pitch_pwm_scale', 160.0)
        self.declare_parameter('min_pwm_us', 1000.0)
        self.declare_parameter('max_pwm_us', 2000.0)

        self._manual_control_topic = str(self.get_parameter('manual_control_topic').value)
        self._status_topic = str(self.get_parameter('vehicle_air_water_status_topic').value)
        self._water_thruster_pwm_topic = str(self.get_parameter('water_thruster_pwm_topic').value)
        self._require_water_mode = bool(self.get_parameter('require_water_mode').value)
        self._require_valid_manual_control = bool(
            self.get_parameter('require_valid_manual_control').value
        )
        self._deadband = max(0.0, float(self.get_parameter('deadband').value))
        self._surge_axis = str(self.get_parameter('surge_axis').value)
        self._yaw_axis = str(self.get_parameter('yaw_axis').value)
        self._heave_axis = str(self.get_parameter('heave_axis').value)
        self._pitch_axis = str(self.get_parameter('pitch_axis').value)
        self._surge_pwm_scale = float(self.get_parameter('surge_pwm_scale').value)
        self._yaw_pwm_scale = float(self.get_parameter('yaw_pwm_scale').value)
        self._heave_pwm_scale = float(self.get_parameter('heave_pwm_scale').value)
        self._pitch_pwm_scale = float(self.get_parameter('pitch_pwm_scale').value)
        self._min_pwm_us = float(self.get_parameter('min_pwm_us').value)
        self._max_pwm_us = float(self.get_parameter('max_pwm_us').value)

        self._in_water = False
        self._last_pwm = [PWM_NEUTRAL_US] * 4
        self._warned_unknown_axes = set()

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_pwm = self.create_publisher(
            Float64MultiArray,
            self._water_thruster_pwm_topic,
            10,
        )
        self._sub_manual = self.create_subscription(
            ManualControlSetpoint,
            self._manual_control_topic,
            self._on_manual_control,
            px4_qos,
        )
        self._sub_status = self.create_subscription(
            VehicleAirWaterStatus,
            self._status_topic,
            self._on_status,
            px4_qos,
        )

        self.get_logger().info(
            'TeleH4Z manual water thruster mapper started. '
            f'manual={self._manual_control_topic}, status={self._status_topic}, '
            f'pwm={self._water_thruster_pwm_topic}, require_water_mode={self._require_water_mode}, '
            f'axes=(surge:{self._surge_axis}, yaw:{self._yaw_axis}, '
            f'heave:{self._heave_axis}, pitch:{self._pitch_axis})'
        )

    def _on_status(self, msg: VehicleAirWaterStatus):
        was_in_water = self._in_water
        self._in_water = int(msg.uw_transit_mode) == 1 and int(msg.transform_status) == 0
        if was_in_water and not self._in_water:
            self._publish_pwm([PWM_NEUTRAL_US] * 4)

    def _on_manual_control(self, msg: ManualControlSetpoint):
        if self._require_valid_manual_control and not msg.valid:
            self._publish_if_changed([PWM_NEUTRAL_US] * 4)
            return

        if self._require_water_mode and not self._in_water:
            return

        surge = self._axis_value(msg, self._surge_axis)
        yaw = self._axis_value(msg, self._yaw_axis)
        heave = self._axis_value(msg, self._heave_axis)
        pitch = self._axis_value(msg, self._pitch_axis)

        # Thruster order: [right, front, left, back].
        # Side pair handles surge + yaw; vertical pair handles heave + pitch trim.
        pwm = [
            PWM_NEUTRAL_US + self._surge_pwm_scale * surge + self._yaw_pwm_scale * yaw,
            PWM_NEUTRAL_US + self._heave_pwm_scale * heave + self._pitch_pwm_scale * pitch,
            PWM_NEUTRAL_US + self._surge_pwm_scale * surge - self._yaw_pwm_scale * yaw,
            PWM_NEUTRAL_US + self._heave_pwm_scale * heave - self._pitch_pwm_scale * pitch,
        ]
        self._publish_if_changed([self._clamp(value) for value in pwm])

    def _axis(self, value: float) -> float:
        if value is None or not isfinite(float(value)):
            return 0.0
        value = max(-1.0, min(1.0, float(value)))
        if abs(value) < self._deadband:
            return 0.0
        if value > 0.0:
            return (value - self._deadband) / (1.0 - self._deadband)
        return (value + self._deadband) / (1.0 - self._deadband)

    def _axis_value(self, msg: ManualControlSetpoint, axis_name: str) -> float:
        name = axis_name.strip().lower()
        sign = 1.0
        if name.startswith('-'):
            sign = -1.0
            name = name[1:]
        elif name.startswith('+'):
            name = name[1:]

        if name in ('none', 'disabled', 'off', ''):
            return 0.0
        if name not in ('roll', 'pitch', 'yaw', 'throttle'):
            if axis_name not in self._warned_unknown_axes:
                self.get_logger().warn(
                    f'Unknown manual control axis "{axis_name}", using 0. '
                    'Valid axes: roll, pitch, yaw, throttle, none; prefix with - to invert.'
                )
                self._warned_unknown_axes.add(axis_name)
            return 0.0
        return sign * self._axis(getattr(msg, name))

    def _publish_if_changed(self, pwm):
        if any(abs(a - b) >= 1.0 for a, b in zip(pwm, self._last_pwm)):
            self._publish_pwm(pwm)

    def _publish_pwm(self, pwm):
        msg = Float64MultiArray()
        msg.data = list(pwm)
        self._pub_pwm.publish(msg)
        self._last_pwm = list(pwm)

    def _clamp(self, pwm_us: float) -> float:
        return min(max(float(pwm_us), self._min_pwm_us), self._max_pwm_us)


def main(args=None):
    rclpy.init(args=args)
    node = TeleH4ZManualWaterThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
