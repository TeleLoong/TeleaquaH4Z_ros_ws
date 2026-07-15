import csv
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


CSV_COLUMNS = ('time', 'pwm_1', 'pwm_2', 'pwm_3', 'pwm_4')
THRUSTER_TO_BOARD_INDEX = {
    'left': 0,
    'right': 1,
    'front': 2,
    'back': 3,
}
SERVO_TO_THRUSTER = ('right', 'front', 'left', 'back')


class ReplayRow:

    def __init__(self, relative_time_s: float, board_pwm: Sequence[float]):
        self.relative_time_s = relative_time_s
        self.board_pwm = list(board_pwm)


class TeleH4ZWaterThrusterReplay(Node):

    def __init__(self):
        super().__init__('teleh4z_water_thruster_replay')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('model_name', 'teleh4z_0')
        self.declare_parameter('mirror_sum', 3000.0)
        self.declare_parameter('start_delay_s', 0.0)
        self.declare_parameter('output_csv_path', '/tmp/teleh4z_gazebo_replay_log.csv')
        self.declare_parameter('timer_rate_hz', 200.0)

        self._csv_path = str(self.get_parameter('csv_path').value)
        self._model_name = str(self.get_parameter('model_name').value)
        self._mirror_sum = float(self.get_parameter('mirror_sum').value)
        self._start_delay_s = max(0.0, float(self.get_parameter('start_delay_s').value))
        self._output_csv_path = str(self.get_parameter('output_csv_path').value)
        timer_rate_hz = max(1.0, float(self.get_parameter('timer_rate_hz').value))

        self._rows = self._load_replay_csv(self._csv_path)
        self._next_row_index = 0
        self._clock_s: Optional[float] = None
        self._replay_start_clock_s: Optional[float] = None
        self._replay_done = False

        self._current_board_pwm = list(self._rows[0].board_pwm)
        self._current_plugin_by_thruster = self._mirror_by_thruster(self._current_board_pwm)

        self._pubs = [
            self.create_publisher(Actuators, f'/{self._model_name}/servo_{index}', 10)
            for index in range(4)
        ]

        self._clock_sub = self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            f'/model/{self._model_name}/odometry',
            self._on_odometry,
            50,
        )

        self._log_file = self._open_output_csv(self._output_csv_path)
        self._log_writer = csv.DictWriter(
            self._log_file,
            fieldnames=[
                't_sim',
                't_rel',
                'x_world',
                'y_world',
                'z_world',
                'roll',
                'pitch',
                'yaw',
                'u_pos',
                'u_vel',
                'u_acc',
                'pwm_1',
                'pwm_2',
                'pwm_3',
                'pwm_4',
                'plugin_left',
                'plugin_right',
                'plugin_front',
                'plugin_back',
            ],
        )
        self._log_writer.writeheader()
        self._log_file.flush()

        self._initial_position: Optional[List[float]] = None
        self._initial_body_x_world: Optional[List[float]] = None
        self._last_odom_t_s: Optional[float] = None
        self._last_u_vel: Optional[float] = None

        self._timer = self.create_timer(1.0 / timer_rate_hz, self._tick)

        self.get_logger().info(
            f'Loaded {len(self._rows)} replay rows from {self._csv_path}. '
            f'Publishing mirrored PWM to /{self._model_name}/servo_0..3 and '
            f'logging odometry to {self._output_csv_path}.'
        )

    def destroy_node(self):
        if hasattr(self, '_log_file') and not self._log_file.closed:
            self._log_file.flush()
            self._log_file.close()
        super().destroy_node()

    def _load_replay_csv(self, csv_path: str) -> List[ReplayRow]:
        if not csv_path:
            raise ValueError('csv_path parameter is required')

        path = Path(csv_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Replay CSV not found: {path}')

        rows: List[ReplayRow] = []
        with path.open('r', newline='') as handle:
            reader = csv.DictReader(handle)
            missing_columns = [name for name in CSV_COLUMNS if name not in (reader.fieldnames or [])]
            if missing_columns:
                raise ValueError(
                    f'Replay CSV missing required columns: {", ".join(missing_columns)}'
                )

            first_time: Optional[float] = None
            last_relative_time = -math.inf
            for line_number, row in enumerate(reader, start=2):
                time_s = self._parse_float(row['time'], 'time', line_number)
                if first_time is None:
                    first_time = time_s
                relative_time_s = time_s - first_time
                if relative_time_s < last_relative_time:
                    raise ValueError(
                        f'Replay CSV time must be monotonically increasing; '
                        f'line {line_number} is earlier than the previous row.'
                    )
                last_relative_time = relative_time_s

                board_pwm = [
                    self._parse_float(row[column], column, line_number)
                    for column in CSV_COLUMNS[1:]
                ]
                rows.append(ReplayRow(relative_time_s, board_pwm))

        if not rows:
            raise ValueError(f'Replay CSV has no data rows: {path}')

        return rows

    def _parse_float(self, value: str, column: str, line_number: int) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'Invalid numeric value for {column} at line {line_number}: {value!r}'
            ) from exc

        if not math.isfinite(number):
            raise ValueError(f'Non-finite value for {column} at line {line_number}: {value!r}')
        return number

    def _open_output_csv(self, output_csv_path: str):
        path = Path(output_csv_path).expanduser()
        if path.parent and str(path.parent) != '.':
            path.parent.mkdir(parents=True, exist_ok=True)
        return path.open('w', newline='')

    def _on_clock(self, msg: Clock):
        self._clock_s = self._stamp_to_seconds(msg.clock)

    def _tick(self):
        if self._clock_s is None or self._replay_done:
            return

        if self._replay_start_clock_s is None:
            self._replay_start_clock_s = self._clock_s + self._start_delay_s
            self.get_logger().info(
                f'Replay will start at sim time {self._replay_start_clock_s:.6f}s'
            )

        replay_time_s = self._clock_s - self._replay_start_clock_s
        if replay_time_s < 0.0:
            return

        published = False
        while self._next_row_index < len(self._rows):
            row = self._rows[self._next_row_index]
            if row.relative_time_s > replay_time_s:
                break

            self._current_board_pwm = list(row.board_pwm)
            self._current_plugin_by_thruster = self._mirror_by_thruster(row.board_pwm)
            self._publish_current_command()
            self._next_row_index += 1
            published = True

        if self._next_row_index >= len(self._rows):
            self._replay_done = True
            self.get_logger().info(
                f'Replay complete at sim time {self._clock_s:.6f}s; '
                f'published {len(self._rows)} rows.'
            )
        elif published:
            self.get_logger().debug(
                f'Replay advanced to row {self._next_row_index} at sim time {self._clock_s:.6f}s'
            )

    def _publish_current_command(self):
        now = self.get_clock().now().to_msg()

        for servo_index, thruster_name in enumerate(SERVO_TO_THRUSTER):
            msg = Actuators()
            msg.header.stamp = now
            msg.header.frame_id = thruster_name
            msg.velocity = [self._current_plugin_by_thruster[thruster_name]]
            self._pubs[servo_index].publish(msg)

    def _mirror_by_thruster(self, board_pwm: Sequence[float]) -> Dict[str, float]:
        return {
            thruster: self._mirror_sum - board_pwm[board_index]
            for thruster, board_index in THRUSTER_TO_BOARD_INDEX.items()
        }

    def _on_odometry(self, msg: Odometry):
        if self._clock_s is None or self._replay_start_clock_s is None:
            return

        t_sim = self._clock_s
        t_rel = t_sim - self._replay_start_clock_s
        if t_rel < 0.0:
            return

        position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

        roll, pitch, yaw = self._quaternion_to_euler(quaternion)
        body_x_world = self._body_x_axis_world(quaternion)

        if self._initial_position is None:
            self._initial_position = list(position)
            self._initial_body_x_world = list(body_x_world)

        u_pos = self._dot(
            [position[i] - self._initial_position[i] for i in range(3)],
            self._initial_body_x_world,
        )
        u_vel = msg.twist.twist.linear.x

        u_acc = 0.0
        if self._last_odom_t_s is not None and self._last_u_vel is not None:
            dt = t_sim - self._last_odom_t_s
            if dt > 0.0:
                u_acc = (u_vel - self._last_u_vel) / dt

        self._last_odom_t_s = t_sim
        self._last_u_vel = u_vel

        self._log_writer.writerow({
            't_sim': self._format_float(t_sim),
            't_rel': self._format_float(t_rel),
            'x_world': self._format_float(position[0]),
            'y_world': self._format_float(position[1]),
            'z_world': self._format_float(position[2]),
            'roll': self._format_float(roll),
            'pitch': self._format_float(pitch),
            'yaw': self._format_float(yaw),
            'u_pos': self._format_float(u_pos),
            'u_vel': self._format_float(u_vel),
            'u_acc': self._format_float(u_acc),
            'pwm_1': self._format_float(self._current_board_pwm[0]),
            'pwm_2': self._format_float(self._current_board_pwm[1]),
            'pwm_3': self._format_float(self._current_board_pwm[2]),
            'pwm_4': self._format_float(self._current_board_pwm[3]),
            'plugin_left': self._format_float(self._current_plugin_by_thruster['left']),
            'plugin_right': self._format_float(self._current_plugin_by_thruster['right']),
            'plugin_front': self._format_float(self._current_plugin_by_thruster['front']),
            'plugin_back': self._format_float(self._current_plugin_by_thruster['back']),
        })
        self._log_file.flush()

    def _stamp_to_seconds(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _quaternion_to_euler(self, q: Sequence[float]):
        x, y, z, w = q

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _body_x_axis_world(self, q: Sequence[float]) -> List[float]:
        x, y, z, w = q
        return [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ]

    def _dot(self, left: Sequence[float], right: Sequence[float]) -> float:
        return sum(left[i] * right[i] for i in range(3))

    def _format_float(self, value) -> str:
        return f'{float(value):.9g}'


def main(args=None):
    rclpy.init(args=args)
    node = TeleH4ZWaterThrusterReplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
