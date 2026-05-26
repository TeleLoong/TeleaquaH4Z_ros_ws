from typing import List

import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


THRUSTER_NAMES = ('right', 'front', 'left', 'back')
PWM_NEUTRAL_US = 1500.0


class TeleH4ZWaterThrusterCommander(Node):

    def __init__(self):
        super().__init__('teleh4z_water_thruster_commander')

        self.declare_parameter('model_name', 'teleh4z_0')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('command_gain', 57.29)
        self.declare_parameter('min_pwm_us', 1000.0)
        self.declare_parameter('max_pwm_us', 2000.0)
        self.declare_parameter('initial_pwm_us', [PWM_NEUTRAL_US] * 4)

        self._model_name = str(self.get_parameter('model_name').value)
        self._rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))
        self._command_gain = float(self.get_parameter('command_gain').value)
        self._min_pwm_us = float(self.get_parameter('min_pwm_us').value)
        self._max_pwm_us = float(self.get_parameter('max_pwm_us').value)
        self._pwm_us = self._coerce_four_values(
            list(self.get_parameter('initial_pwm_us').value),
            'initial_pwm_us',
        )

        self._pubs = [
            self.create_publisher(
                Actuators,
                f'/{self._model_name}/servo_{index}',
                10,
            )
            for index in range(4)
        ]

        self._sub_pwm = self.create_subscription(
            Float64MultiArray,
            '/teleh4z/water_thruster_pwm',
            self._on_pwm_command,
            10,
        )
        self._sub_raw = self.create_subscription(
            Float64MultiArray,
            '/teleh4z/water_thruster_raw',
            self._on_raw_command,
            10,
        )

        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish)

        self.get_logger().info(
            'TeleH4Z water thruster commander started. '
            f'Model: {self._model_name}, publish topics: '
            + ', '.join(f'/{self._model_name}/servo_{i}' for i in range(4))
            + '. Input topics: /teleh4z/water_thruster_pwm, /teleh4z/water_thruster_raw'
        )

    def _on_pwm_command(self, msg: Float64MultiArray):
        self._pwm_us = self._coerce_four_values(list(msg.data), '/teleh4z/water_thruster_pwm')

    def _on_raw_command(self, msg: Float64MultiArray):
        raw = self._coerce_four_raw_values(list(msg.data), '/teleh4z/water_thruster_raw')
        self._pwm_us = [self._raw_to_pwm(value) for value in raw]

    def _publish(self):
        now = self.get_clock().now().to_msg()

        for index, pwm_us in enumerate(self._pwm_us):
            msg = Actuators()
            msg.header.stamp = now
            msg.header.frame_id = THRUSTER_NAMES[index]
            msg.velocity = [self._pwm_to_raw(pwm_us)]
            self._pubs[index].publish(msg)

    def _coerce_four_values(self, values: List[float], source: str) -> List[float]:
        if len(values) != 4:
            self.get_logger().warn(
                f'{source} expected 4 values [right, front, left, back], got {len(values)}. '
                'Missing values use neutral 1500us; extras are ignored.'
            )

        padded = (values + [PWM_NEUTRAL_US] * 4)[:4]
        return [self._clamp_pwm(float(value)) for value in padded]

    def _coerce_four_raw_values(self, values: List[float], source: str) -> List[float]:
        if len(values) != 4:
            self.get_logger().warn(
                f'{source} expected 4 values [right, front, left, back], got {len(values)}. '
                'Missing values use neutral raw command; extras are ignored.'
            )

        neutral_raw = self._pwm_to_raw(PWM_NEUTRAL_US)
        padded = (values + [neutral_raw] * 4)[:4]
        return [float(value) for value in padded]

    def _pwm_to_raw(self, pwm_us: float) -> float:
        if abs(self._command_gain) < 1e-6:
            return 0.0
        return self._clamp_pwm(pwm_us) / self._command_gain

    def _raw_to_pwm(self, raw: float) -> float:
        return self._clamp_pwm(raw * self._command_gain)

    def _clamp_pwm(self, pwm_us: float) -> float:
        return min(max(pwm_us, self._min_pwm_us), self._max_pwm_us)


def main(args=None):
    rclpy.init(args=args)
    node = TeleH4ZWaterThrusterCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
