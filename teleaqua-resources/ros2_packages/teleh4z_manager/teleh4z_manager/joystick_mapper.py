from typing import Optional

import rclpy
from px4_msgs.msg import ManualControlSetpoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String


PWM_NEUTRAL_US = 1500.0


class TeleH4ZJoystickMapper(Node):

    def __init__(self):
        super().__init__('teleh4z_joystick_mapper')

        self.declare_parameter('manual_control_topic', '/fmu/out/manual_control_setpoint')
        self.declare_parameter('mode_request_topic', '/teleh4z/mode_request')
        self.declare_parameter('water_thruster_pwm_topic', '/teleh4z/water_thruster_pwm')
        self.declare_parameter('air_button_bit', 3)
        self.declare_parameter('water_button_bit', 0)
        self.declare_parameter('neutral_button_bit', 1)
        self.declare_parameter('enable_neutral_button', True)
        self.declare_parameter('cooldown_sec', 1.0)
        self.declare_parameter('require_valid_manual_control', True)

        self._manual_control_topic = str(self.get_parameter('manual_control_topic').value)
        self._mode_request_topic = str(self.get_parameter('mode_request_topic').value)
        self._water_thruster_pwm_topic = str(self.get_parameter('water_thruster_pwm_topic').value)
        self._air_button_bit = int(self.get_parameter('air_button_bit').value)
        self._water_button_bit = int(self.get_parameter('water_button_bit').value)
        self._neutral_button_bit = int(self.get_parameter('neutral_button_bit').value)
        self._enable_neutral_button = bool(self.get_parameter('enable_neutral_button').value)
        self._cooldown_sec = max(0.0, float(self.get_parameter('cooldown_sec').value))
        self._require_valid_manual_control = bool(
            self.get_parameter('require_valid_manual_control').value
        )

        self._previous_buttons = 0
        self._last_action_time = None
        self._warned_invalid_bits = set()

        manual_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_mode_request = self.create_publisher(String, self._mode_request_topic, 10)
        self._pub_water_pwm = self.create_publisher(
            Float64MultiArray,
            self._water_thruster_pwm_topic,
            10,
        )
        self._sub_manual = self.create_subscription(
            ManualControlSetpoint,
            self._manual_control_topic,
            self._on_manual_control,
            manual_qos,
        )

        self.get_logger().info(
            'TeleH4Z joystick mapper started. '
            f'manual={self._manual_control_topic}, mode={self._mode_request_topic}, '
            f'air_bit={self._air_button_bit}, water_bit={self._water_button_bit}, '
            f'neutral_bit={self._neutral_button_bit}, cooldown={self._cooldown_sec:.2f}s'
        )

    def _on_manual_control(self, msg: ManualControlSetpoint):
        buttons = int(msg.buttons) & 0xFFFF

        if self._require_valid_manual_control and not msg.valid:
            self._previous_buttons = buttons
            return

        if self._edge_pressed(buttons, self._neutral_button_bit) and self._enable_neutral_button:
            if self._cooldown_ready():
                self._publish_water_neutral()
                self._mark_action_time()
            self._previous_buttons = buttons
            return

        if self._edge_pressed(buttons, self._air_button_bit):
            if self._cooldown_ready():
                self._publish_mode_request('air')
                self._mark_action_time()
            self._previous_buttons = buttons
            return

        if self._edge_pressed(buttons, self._water_button_bit):
            if self._cooldown_ready():
                self._publish_mode_request('water')
                self._mark_action_time()
            self._previous_buttons = buttons
            return

        self._previous_buttons = buttons

    def _edge_pressed(self, buttons: int, bit_index: int) -> bool:
        mask = self._button_mask(bit_index)
        if mask is None:
            return False
        return bool(buttons & mask) and not bool(self._previous_buttons & mask)

    def _button_mask(self, bit_index: int) -> Optional[int]:
        if bit_index < 0:
            return None
        if bit_index > 15:
            if bit_index not in self._warned_invalid_bits:
                self.get_logger().warn(
                    f'Ignoring invalid button bit {bit_index}; valid range is 0..15 or -1 to disable.'
                )
                self._warned_invalid_bits.add(bit_index)
            return None
        return 1 << bit_index

    def _cooldown_ready(self) -> bool:
        if self._last_action_time is None:
            return True
        elapsed = (self.get_clock().now() - self._last_action_time).nanoseconds / 1e9
        return elapsed >= self._cooldown_sec

    def _mark_action_time(self):
        self._last_action_time = self.get_clock().now()

    def _publish_mode_request(self, mode: str):
        msg = String()
        msg.data = mode
        self._pub_mode_request.publish(msg)
        self.get_logger().info(f'Joystick requested {mode.upper()} mode.')

    def _publish_water_neutral(self):
        msg = Float64MultiArray()
        msg.data = [PWM_NEUTRAL_US] * 4
        self._pub_water_pwm.publish(msg)
        self.get_logger().warn('Joystick neutral command published for water thrusters.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleH4ZJoystickMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
