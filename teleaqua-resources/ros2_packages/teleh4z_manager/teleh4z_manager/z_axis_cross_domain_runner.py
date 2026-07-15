import csv
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    ConfigOverrides,
    HealthReport,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAirWaterStatus,
    VehicleAirWaterStatusSetpoint,
    VehicleCommandAck,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Float32Array
from rosgraph_msgs.msg import Clock


class TeleH4ZZAxisCrossDomainRunner(Node):

    def __init__(self):
        super().__init__('teleh4z_z_axis_cross_domain_runner')

        self.declare_parameter('model_name', 'teleh4z_zaxis_0')
        self.declare_parameter('output_csv_path', '/tmp/teleh4z_z_axis_cross_domain_gazebo.csv')
        self.declare_parameter('trial_id', 0)
        self.declare_parameter('condition', 'px4_offboard')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('start_delay_s', 0.0)
        self.declare_parameter('offboard_arm_delay_s', 0.1)
        self.declare_parameter('prearm_settle_s', 2.0)
        self.declare_parameter('offboard_confirm_settle_s', 0.5)
        self.declare_parameter('require_preflight_checks_before_arm', True)
        self.declare_parameter('preflight_ready_timeout_s', 30.0)
        self.declare_parameter('require_prop_diag_before_arm', True)
        self.declare_parameter('prop_diag_ready_timeout_s', 12.0)
        self.declare_parameter('require_air_mode', True)
        self.declare_parameter('air_mode_timeout_s', 5.0)
        self.declare_parameter('force_arm', True)
        self.declare_parameter('disable_auto_disarm', True)
        self.declare_parameter('water_depth_m', 1.0)
        self.declare_parameter('air_height_m', 0.70)
        self.declare_parameter('underwater_hover_s', 10.0)
        self.declare_parameter('underwater_ascent_s', 5.0)
        self.declare_parameter('underwater_ascent_target_z_m', -0.10)
        self.declare_parameter('surface_exit_s', 5.0)
        self.declare_parameter('surface_exit_guard_enabled', True)
        self.declare_parameter('surface_exit_guard_height_m', 0.40)
        self.declare_parameter('surface_exit_guard_timeout_s', 0.0)
        self.declare_parameter('surface_exit_guard_thrust_fraction', 0.8)
        self.declare_parameter('air_hover_thrust_N', 16.7)
        self.declare_parameter('surface_z_m', 0.0)
        self.declare_parameter('duration_s', 65.0)
        self.declare_parameter('clock_stall_timeout_s', 1.0)

        self._model_name = str(self.get_parameter('model_name').value)
        self._output_csv_path = str(self.get_parameter('output_csv_path').value)
        self._trial_id = int(self.get_parameter('trial_id').value)
        self._condition = str(self.get_parameter('condition').value)
        self._rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))
        self._start_delay_s = max(0.0, float(self.get_parameter('start_delay_s').value))
        self._offboard_arm_delay_s = max(
            0.0, float(self.get_parameter('offboard_arm_delay_s').value)
        )
        self._prearm_settle_s = max(
            0.0, float(self.get_parameter('prearm_settle_s').value)
        )
        self._offboard_confirm_settle_s = max(
            0.0, float(self.get_parameter('offboard_confirm_settle_s').value)
        )
        self._require_preflight_checks_before_arm = bool(
            self.get_parameter('require_preflight_checks_before_arm').value
        )
        self._preflight_ready_timeout_s = max(
            0.0, float(self.get_parameter('preflight_ready_timeout_s').value)
        )
        self._require_prop_diag_before_arm = bool(
            self.get_parameter('require_prop_diag_before_arm').value
        )
        self._prop_diag_ready_timeout_s = max(
            0.0, float(self.get_parameter('prop_diag_ready_timeout_s').value)
        )
        self._require_air_mode = bool(self.get_parameter('require_air_mode').value)
        self._air_mode_timeout_s = max(
            0.0, float(self.get_parameter('air_mode_timeout_s').value)
        )
        self._force_arm = bool(self.get_parameter('force_arm').value)
        self._disable_auto_disarm = bool(
            self.get_parameter('disable_auto_disarm').value
        )
        self._water_depth = max(0.0, float(self.get_parameter('water_depth_m').value))
        self._air_height = max(0.0, float(self.get_parameter('air_height_m').value))
        self._underwater_hover_s = max(
            0.0, float(self.get_parameter('underwater_hover_s').value)
        )
        self._underwater_ascent_s = max(
            0.1, float(self.get_parameter('underwater_ascent_s').value)
        )
        self._underwater_ascent_target_z_m = float(
            self.get_parameter('underwater_ascent_target_z_m').value
        )
        self._surface_exit_s = max(0.1, float(self.get_parameter('surface_exit_s').value))
        self._surface_exit_guard_enabled = bool(
            self.get_parameter('surface_exit_guard_enabled').value
        )
        self._surface_exit_guard_height = max(
            0.0, float(self.get_parameter('surface_exit_guard_height_m').value)
        )
        self._surface_exit_guard_timeout_s = max(
            0.0, float(self.get_parameter('surface_exit_guard_timeout_s').value)
        )
        self._surface_exit_guard_thrust_fraction = max(
            0.0, float(self.get_parameter('surface_exit_guard_thrust_fraction').value)
        )
        self._air_hover_thrust_N = max(
            0.0, float(self.get_parameter('air_hover_thrust_N').value)
        )
        self._surface_exit_guard_min_thrust = (
            self._surface_exit_guard_thrust_fraction * self._air_hover_thrust_N
        )
        self._surface_z = float(self.get_parameter('surface_z_m').value)
        self._duration_s = max(1.0, float(self.get_parameter('duration_s').value))
        self._clock_stall_timeout_s = max(
            0.0, float(self.get_parameter('clock_stall_timeout_s').value)
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        prop_diag_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._clock_s: Optional[float] = None
        self._start_clock_s: Optional[float] = None
        self._latest_odom: Optional[Odometry] = None
        self._latest_local_position: Optional[VehicleLocalPosition] = None
        self._latest_status: Optional[VehicleStatus] = None
        self._latest_health_report: Optional[HealthReport] = None
        self._latest_air_water_status: Optional[VehicleAirWaterStatus] = None
        self._latest_command_ack: Optional[VehicleCommandAck] = None
        self._last_event_labels = set()
        self._finished = False
        self._air_mode_log_done = False
        self._last_offboard_request_s: Optional[float] = None
        self._last_arm_request_s: Optional[float] = None
        self._offboard_ready_since_s: Optional[float] = None
        self._last_logged_clock_s: Optional[float] = None
        self._clock_stall_wall_start_s: Optional[float] = None
        self._waiting_for_prop_diag_logged = False
        self._prop_diag_timeout_logged = False
        self._waiting_for_preflight_logged = False
        self._preflight_timeout_logged = False

        self._local_x0: Optional[float] = None
        self._local_y0: Optional[float] = None
        self._local_z0: Optional[float] = None
        self._yaw0: Optional[float] = None
        self._world_z0: Optional[float] = None

        self._mission_t0: Optional[float] = None
        self._ascent_start_z_m: Optional[float] = None
        self._effective_underwater_ascent_s: Optional[float] = None
        self._vz_ref_mps = 0.0
        self._surface_exit_guard_start_s: Optional[float] = None
        self._surface_exit_guard_release_s: Optional[float] = None
        self._surface_exit_guard_release_reason = ''
        self._surface_exit_guard_state = 'inactive'
        self._surface_exit_guard_elapsed_s = 0.0

        self._prop_diag: Dict[int, List[float]] = {index: [0.0] * 6 for index in range(4)}
        self._prop_diag_stamp_s: Dict[int, Optional[float]] = {
            index: None for index in range(4)
        }

        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self._pub_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos
        )
        self._pub_command = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )
        self._pub_air_water_setpoint = self.create_publisher(
            VehicleAirWaterStatusSetpoint,
            '/fmu/in/vehicle_air_water_status_setpoint',
            px4_qos,
        )
        self._pub_config_overrides = self.create_publisher(
            ConfigOverrides,
            '/fmu/in/config_overrides',
            px4_qos,
        )

        self._sub_clock = self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._sub_odom = self.create_subscription(
            Odometry,
            f'/model/{self._model_name}/odometry',
            self._on_odometry,
            50,
        )
        self._sub_local_position = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._on_local_position,
            px4_qos,
        )
        self._sub_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._on_status,
            px4_qos,
        )
        self._sub_health_report = self.create_subscription(
            HealthReport,
            '/fmu/out/health_report',
            self._on_health_report,
            px4_qos,
        )
        self._sub_air_water_status = self.create_subscription(
            VehicleAirWaterStatus,
            '/fmu/out/vehicle_air_water_status',
            self._on_air_water_status,
            px4_qos,
        )
        self._sub_command_ack = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self._on_command_ack,
            px4_qos,
        )
        for index in range(4):
            self.create_subscription(
                Float32Array,
                f'/model/{self._model_name}/hybrid_air_propeller_{index}/state',
                lambda msg, i=index: self._on_prop_diag(i, msg),
                prop_diag_qos,
            )

        self._log_file = self._open_output_csv(self._output_csv_path)
        self._writer = csv.DictWriter(
            self._log_file,
            fieldnames=[
                't', 'pos_x', 'pos_y', 'pos_z', 'roll', 'pitch', 'yaw',
                'lin_vx', 'lin_vy', 'lin_vz', 'ang_vx', 'ang_vy', 'ang_vz',
                'T1', 'T2', 'T3', 'T4', 'T_sum', 'RPM1', 'RPM2', 'RPM3', 'RPM4',
                'cmd1', 'cmd2', 'cmd3', 'cmd4',
                'omega1', 'omega2', 'omega3', 'omega4',
                'domain_code1', 'domain_code2', 'domain_code3', 'domain_code4',
                'thrust_margin_avg_N', 'thrust_margin_min_N',
                'surface_exit_guard_state', 'surface_exit_guard_elapsed_s',
                'surface_exit_guard_min_thrust_N',
                'source', 'trial_id', 'condition', 'z_ref_m', 'vz_ref_mps', 'command',
                'domain_state', 'event_label', 'submersion_ratio',
                'control_source', 'notes',
            ],
        )
        self._writer.writeheader()
        self._log_file.flush()

        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            'TeleH4Z PX4 offboard Z-axis cross-domain runner started. '
            f'model={self._model_name}, log={self._output_csv_path}'
        )

    def destroy_node(self):
        if hasattr(self, '_log_file') and not self._log_file.closed:
            self._log_file.flush()
            self._log_file.close()
        super().destroy_node()

    def _open_output_csv(self, output_csv_path: str):
        path = Path(output_csv_path).expanduser()
        if path.parent and str(path.parent) != '.':
            path.parent.mkdir(parents=True, exist_ok=True)
        return path.open('w', newline='')

    def _on_clock(self, msg: Clock):
        self._clock_s = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def _on_odometry(self, msg: Odometry):
        self._latest_odom = msg

    def _on_local_position(self, msg: VehicleLocalPosition):
        self._latest_local_position = msg

    def _on_status(self, msg: VehicleStatus):
        self._latest_status = msg

    def _on_health_report(self, msg: HealthReport):
        self._latest_health_report = msg

    def _on_air_water_status(self, msg: VehicleAirWaterStatus):
        self._latest_air_water_status = msg

    def _on_command_ack(self, msg: VehicleCommandAck):
        self._latest_command_ack = msg
        if int(msg.result) != int(VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED):
            self.get_logger().warn(
                f'PX4 command ack: command={int(msg.command)} result={int(msg.result)} '
                f'param1={int(msg.result_param1)} param2={int(msg.result_param2)}'
            )

    def _on_prop_diag(self, index: int, msg: Float32Array):
        values = list(msg.data)
        if len(values) < 6:
            values.extend([0.0] * (6 - len(values)))
        self._prop_diag[index] = values[:6]
        self._prop_diag_stamp_s[index] = self._clock_s

    def _tick(self):
        if self._finished or self._clock_s is None:
            return
        if self._latest_local_position is None:
            return

        if self._start_clock_s is None:
            self._start_clock_s = self._clock_s + self._start_delay_s
            self.get_logger().info(
                f'PX4 offboard Z-axis task will start at sim time {self._start_clock_s:.3f}s'
            )

        t_rel = self._clock_s - self._start_clock_s
        if t_rel < 0.0:
            self._publish_hold_setpoint()
            return

        self._capture_initial_reference()

        if t_rel > self._duration_s:
            self._finish_task(
                f'Z-axis PX4 offboard task complete at t={t_rel:.3f}s; '
                f'log={self._output_csv_path}'
            )
            return

        # --- Request AIR mode and OFFBOARD/ARM.  The mission timer only
        #     starts once the vehicle is actually tracking (OFFBOARD+ARMED).
        #     Until then the setpoint is held at the S0 underwater hover
        #     depth, preventing the robot from skipping phases. ---
        self._request_air_mode()
        self._publish_config_overrides()

        air_ready = self._air_mode_ready(t_rel)
        startup_prop_diag_ready = (
            True if self._mission_t0 is not None else self._prop_diag_ready(t_rel)
        )
        startup_ready = (
            t_rel >= self._offboard_arm_delay_s + self._prearm_settle_s and
            air_ready and
            startup_prop_diag_ready
        )
        if startup_ready and not self._offboard_ready():
            self._request_offboard_periodically(self._clock_s)

        if self._offboard_ready():
            if self._offboard_ready_since_s is None:
                self._offboard_ready_since_s = self._clock_s
                self.get_logger().info('PX4 confirmed OFFBOARD mode; waiting for arm readiness.')
        else:
            self._offboard_ready_since_s = None

        offboard_settled = (
            self._offboard_ready_since_s is not None and
            self._clock_s - self._offboard_ready_since_s >= self._offboard_confirm_settle_s
        )
        preflight_ready = (
            self._preflight_ready(t_rel) if startup_ready and offboard_settled else False
        )
        if startup_ready and offboard_settled and preflight_ready and not self._armed_ready():
            self._request_arm_periodically(self._clock_s)

        if self._offboard_ready() and self._armed_ready():
            if self._mission_t0 is None:
                self._capture_initial_reference(force=True, reason='PX4 armed mission hover reference')
                self._mission_t0 = t_rel
                self.get_logger().info(
                    f'Vehicle is OFFBOARD+ARMED; mission timer starts at '
                    f't_rel={t_rel:.3f}s'
                )
            t_mission = t_rel - self._mission_t0
        else:
            if self._mission_t0 is None:
                t_mission = 0.0
            else:
                t_mission = t_rel - self._mission_t0

        local_z_ref, local_vz_ref, z_ref_m, command_label = self._local_z_reference(
            t_mission
        )
        self._vz_ref_mps = local_vz_ref
        self._publish_offboard_control(local_z_ref, local_vz_ref)

        self._write_log_row_once(t_rel, z_ref_m, command_label)

    def _finish_task(self, message: str):
        if self._finished:
            return
        self._finished = True
        self.get_logger().info(message)
        if hasattr(self, '_log_file') and not self._log_file.closed:
            self._log_file.flush()
            self._log_file.close()

    def _write_log_row_once(self, t_rel: float, z_ref_m: float, command_label: str):
        if (
            self._last_logged_clock_s is not None and
            self._clock_s <= self._last_logged_clock_s + 1e-9
        ):
            self._handle_stalled_clock(t_rel)
            return

        self._clock_stall_wall_start_s = None
        self._write_log_row(t_rel, z_ref_m, command_label)
        self._last_logged_clock_s = self._clock_s

    def _handle_stalled_clock(self, t_rel: float):
        if self._clock_stall_wall_start_s is None:
            self._clock_stall_wall_start_s = time.monotonic()
            return
        elapsed_s = time.monotonic() - self._clock_stall_wall_start_s
        if elapsed_s >= self._clock_stall_timeout_s:
            self._finish_task(
                f'Sim clock stalled at t={t_rel:.6f}s for {elapsed_s:.3f}s; '
                f'stopping CSV logging at log={self._output_csv_path}'
            )

    def _capture_initial_reference(self, force: bool = False, reason: str = 'PX4 offboard hold reference'):
        if self._local_z0 is not None and not force:
            return
        local_position = self._latest_local_position
        odom = self._latest_odom
        self._local_x0 = float(local_position.x)
        self._local_y0 = float(local_position.y)
        self._local_z0 = float(local_position.z)
        self._yaw0 = float(local_position.heading) if math.isfinite(local_position.heading) else 0.0
        if odom is not None:
            self._world_z0 = float(odom.pose.pose.position.z)
        else:
            self._world_z0 = -float(local_position.z)
        self.get_logger().info(
            f'Captured {reason}: '
            f'local=({self._local_x0:.3f}, {self._local_y0:.3f}, {self._local_z0:.3f}), '
            f'world_z={self._world_z0:.3f}, odom={1 if odom is not None else 0}'
        )

    def _publish_hold_setpoint(self):
        local_position = self._latest_local_position
        if local_position is None:
            return
        self._publish_offboard_mode()
        self._publish_trajectory(
            float(local_position.x),
            float(local_position.y),
            float(local_position.z),
            float(local_position.heading) if math.isfinite(local_position.heading) else 0.0,
        )

    def _publish_offboard_control(self, local_z_ref: float, local_vz_ref: float):
        self._publish_offboard_mode()
        self._publish_trajectory(
            self._local_x0 if self._local_x0 is not None else 0.0,
            self._local_y0 if self._local_y0 is not None else 0.0,
            local_z_ref,
            self._yaw0 if self._yaw0 is not None else 0.0,
            vz=local_vz_ref,
        )

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self._pub_offboard.publish(msg)

    def _publish_trajectory(
        self, x: float, y: float, z: float, yaw: float, vz: float = math.nan
    ):
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan, math.nan, float(vz)]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        self._pub_trajectory.publish(msg)

    def _request_air_mode(self):
        if not self._require_air_mode:
            return
        msg = VehicleAirWaterStatusSetpoint()
        msg.timestamp = self._timestamp_us()
        msg.uw_transit_mode = 0
        msg.transform_status = 0
        self._pub_air_water_setpoint.publish(msg)

    def _publish_config_overrides(self):
        if not self._disable_auto_disarm:
            return
        msg = ConfigOverrides()
        msg.timestamp = self._timestamp_us()
        msg.disable_auto_disarm = True
        msg.defer_failsafes = False
        msg.defer_failsafes_timeout_s = 0
        msg.source_type = ConfigOverrides.SOURCE_TYPE_MODE
        msg.source_id = int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self._pub_config_overrides.publish(msg)

    def _air_mode_ready(self, t_rel: float) -> bool:
        if not self._require_air_mode:
            return True
        if (
            self._latest_air_water_status is not None and
            int(self._latest_air_water_status.uw_transit_mode) == 0
        ):
            if not self._air_mode_log_done:
                self.get_logger().info('PX4 confirmed AIR mode; enabling offboard arm sequence.')
                self._air_mode_log_done = True
            return True
        if t_rel > self._air_mode_timeout_s:
            self.get_logger().warn(
                f'PX4 has not confirmed AIR mode within {self._air_mode_timeout_s:.1f}s; '
                'proceeding with offboard arm sequence anyway.'
            )
            return True
        return False

    def _prop_diag_ready(self, t_rel: float) -> bool:
        if not self._require_prop_diag_before_arm:
            return True
        if self._clock_s is None:
            return False

        fresh = [
            stamp is not None and self._clock_s - stamp <= 1.0
            for stamp in self._prop_diag_stamp_s.values()
        ]
        if all(fresh):
            initial_sub = self._mean_prop_diag(4)
            initial_domain = self._mean_prop_diag(5)
            if initial_sub >= 0.5 or initial_domain >= 1.5:
                return True

        if not self._waiting_for_prop_diag_logged:
            self.get_logger().warn(
                'Waiting for all hybrid propeller diagnostics before OFFBOARD/ARM. '
                'If this stays stuck, check Gazebo plugin load and ros_gz_bridge topics.'
            )
            self._waiting_for_prop_diag_logged = True

        if (
            self._prop_diag_ready_timeout_s > 0.0 and
            t_rel >= self._prop_diag_ready_timeout_s and
            not self._prop_diag_timeout_logged
        ):
            stamps = ','.join('ok' if item else 'missing' for item in fresh)
            self.get_logger().error(
                'Hybrid propeller diagnostics are not ready; refusing to arm this trial. '
                f'fresh=[{stamps}], sub={self._mean_prop_diag(4):.3f}, '
                f'domain={self._mean_prop_diag(5):.3f}'
            )
            self._prop_diag_timeout_logged = True
        return False

    def _preflight_ready(self, t_rel: float) -> bool:
        if not self._require_preflight_checks_before_arm:
            return True
        status = self._latest_status
        if status is None:
            return False

        preflight_ok = bool(getattr(status, 'pre_flight_checks_pass', False))
        failsafe = bool(getattr(status, 'failsafe', False))
        offboard_can_arm = self._health_report_can_arm_offboard()
        if preflight_ok and not failsafe and offboard_can_arm:
            return True

        if not self._waiting_for_preflight_logged:
            self.get_logger().warn(
                'Waiting for PX4 preflight health before ARM. '
                f'{self._preflight_notes()}'
            )
            self._waiting_for_preflight_logged = True

        if (
            self._preflight_ready_timeout_s > 0.0 and
            t_rel >= self._preflight_ready_timeout_s and
            not self._preflight_timeout_logged
        ):
            self.get_logger().error(
                'PX4 preflight health is still not ready; refusing to arm this trial. '
                f'{self._preflight_notes()}'
            )
            self._preflight_timeout_logged = True
        return False

    def _health_report_can_arm_offboard(self) -> bool:
        report = self._latest_health_report
        if report is None:
            return True
        offboard_bit = 1 << int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        flags = int(getattr(report, 'can_arm_mode_flags', 0))
        return (flags & offboard_bit) != 0

    def _preflight_notes(self) -> str:
        status = self._latest_status
        report = self._latest_health_report
        if status is None:
            return (
                'preflight=; failsafe=; safety_off=; health_error=; '
                'arm_error=; can_arm_offboard='
            )

        health_error = ''
        arm_error = ''
        can_arm_offboard = ''
        if report is not None:
            health_error = str(int(getattr(report, 'health_error_flags', 0)))
            arm_error = str(int(getattr(report, 'arming_check_error_flags', 0)))
            can_arm_offboard = str(int(self._health_report_can_arm_offboard()))

        return (
            f'preflight={int(bool(getattr(status, "pre_flight_checks_pass", False)))}; '
            f'failsafe={int(bool(getattr(status, "failsafe", False)))}; '
            f'safety_off={int(bool(getattr(status, "safety_off", False)))}; '
            f'health_error={health_error}; arm_error={arm_error}; '
            f'can_arm_offboard={can_arm_offboard}'
        )

    def _set_offboard_mode(self):
        self.get_logger().info('Requesting PX4 OFFBOARD mode')
        self._publish_vehicle_command(
            getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176),
            param1=1.0,
            param2=6.0,
        )

    def _arm(self):
        force_text = ' with force-arm magic' if self._force_arm else ''
        self.get_logger().info(f'Requesting PX4 arm{force_text}')
        self._publish_vehicle_command(
            getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400),
            param1=1.0,
            param2=21196.0 if self._force_arm else 0.0,
        )

    def _publish_vehicle_command(self, command: int, **params):
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_command.publish(msg)

    def _request_offboard_periodically(self, t_rel: float):
        if (
            self._last_offboard_request_s is None or
            t_rel - self._last_offboard_request_s >= 1.0
        ):
            self._set_offboard_mode()
            self._last_offboard_request_s = t_rel

    def _request_arm_periodically(self, t_rel: float):
        if self._last_arm_request_s is None or t_rel - self._last_arm_request_s >= 1.0:
            self._arm()
            self._last_arm_request_s = t_rel

    def _offboard_ready(self) -> bool:
        return (
            self._latest_status is not None and
            int(self._latest_status.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        )

    def _armed_ready(self) -> bool:
        return (
            self._latest_status is not None and
            int(self._latest_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )

    def _local_z_reference(self, t: float):
        # Use the actual captured depth as the S0 hover target so the robot
        # hovers where it is, rather than jumping to a pre-configured depth
        # that may be far from its current position.
        start_depth = -self._world_z0 if self._world_z0 is not None else self._water_depth

        t0 = self._underwater_hover_s

        if t < t0:
            self._surface_exit_guard_state = 'inactive'
            self._surface_exit_guard_elapsed_s = 0.0
            z_ref_m = start_depth
            label = 'S0_UNDERWATER_HOVER'
            return self._local_z_from_z_m(z_ref_m), 0.0, z_ref_m, label

        if self._ascent_start_z_m is None:
            self._capture_ascent_start(start_depth)

        ascent_start_z_m = self._ascent_start_z_m
        ascent_duration_s = self._effective_underwater_ascent_s
        t1 = t0 + ascent_duration_s

        if t < t1:
            self._surface_exit_guard_state = 'inactive'
            self._surface_exit_guard_elapsed_s = 0.0
            z_ref_m = self._lerp(
                ascent_start_z_m,
                self._underwater_ascent_target_z_m,
                (t - t0) / ascent_duration_s,
            )
            label = 'S1_UNDERWATER_ASCENT'
            vz_ref_mps = (
                self._underwater_ascent_target_z_m - ascent_start_z_m
            ) / ascent_duration_s
        elif self._surface_exit_guard_enabled and self._surface_exit_guard_timeout_s > 0.0:
            z_ref_m, label = self._guarded_surface_exit_reference(t, t1)
            if label == 'S2_SURFACE_EXIT':
                vz_ref_mps = (
                    -self._air_height + self._surface_exit_guard_height
                ) / self._surface_exit_s
            else:
                vz_ref_mps = 0.0
        else:
            self._surface_exit_guard_state = 'disabled'
            self._surface_exit_guard_elapsed_s = 0.0
            t2 = t1 + self._surface_exit_s
            if t < t2:
                z_ref_m = self._lerp(
                    self._underwater_ascent_target_z_m,
                    -self._air_height,
                    (t - t1) / self._surface_exit_s,
                )
                label = 'S2_SURFACE_EXIT'
                vz_ref_mps = (
                    -self._air_height - self._underwater_ascent_target_z_m
                ) / self._surface_exit_s
            else:
                z_ref_m = -self._air_height
                label = 'S5_AIR_HOVER'
                vz_ref_mps = 0.0
        return self._local_z_from_z_m(z_ref_m), vz_ref_mps, z_ref_m, label

    def _capture_ascent_start(self, planned_start_z_m: float):
        if self._latest_odom is not None:
            ascent_start_z_m = -float(self._latest_odom.pose.pose.position.z)
        elif self._latest_local_position is not None:
            _, _, world_z = self._local_position_fallback_xyz(self._latest_local_position)
            ascent_start_z_m = -float(world_z)
        else:
            ascent_start_z_m = planned_start_z_m

        planned_distance_m = abs(
            planned_start_z_m - self._underwater_ascent_target_z_m
        )
        remaining_distance_m = abs(
            ascent_start_z_m - self._underwater_ascent_target_z_m
        )
        nominal_speed_mps = planned_distance_m / self._underwater_ascent_s
        if nominal_speed_mps > 1e-6:
            ascent_duration_s = max(0.1, remaining_distance_m / nominal_speed_mps)
        else:
            ascent_duration_s = 0.1

        self._ascent_start_z_m = ascent_start_z_m
        self._effective_underwater_ascent_s = ascent_duration_s
        self.get_logger().info(
            'Captured measured S1 ascent start: '
            f'z_m={ascent_start_z_m:.3f}, target={self._underwater_ascent_target_z_m:.3f}, '
            f'duration={ascent_duration_s:.3f}s, nominal_speed={nominal_speed_mps:.3f}m/s'
        )

    def _guarded_surface_exit_reference(self, t: float, surface_exit_start_s: float):
        guard_z_ref_m = -self._surface_exit_guard_height
        if self._surface_exit_guard_start_s is None:
            self._surface_exit_guard_start_s = t
            self._surface_exit_guard_release_s = None
            self._surface_exit_guard_release_reason = ''

        if self._surface_exit_guard_release_s is None:
            elapsed_s = max(0.0, t - self._surface_exit_guard_start_s)
            thrust_ready = self._total_prop_thrust() >= self._surface_exit_guard_min_thrust
            timed_out = elapsed_s >= self._surface_exit_guard_timeout_s
            if thrust_ready or timed_out:
                self._surface_exit_guard_release_s = t
                self._surface_exit_guard_release_reason = 'thrust' if thrust_ready else 'timeout'
                self._surface_exit_guard_state = f'released_{self._surface_exit_guard_release_reason}'
                self._surface_exit_guard_elapsed_s = elapsed_s
                self.get_logger().info(
                    'Surface exit guard released by '
                    f'{self._surface_exit_guard_release_reason}: '
                    f'thrust={self._total_prop_thrust():.3f}N, '
                    f'threshold={self._surface_exit_guard_min_thrust:.3f}N, '
                    f'elapsed={elapsed_s:.3f}s'
                )
            else:
                self._surface_exit_guard_state = 'holding'
                self._surface_exit_guard_elapsed_s = elapsed_s
                return guard_z_ref_m, 'S2_SURFACE_EXIT_GUARD'

        release_s = self._surface_exit_guard_release_s or surface_exit_start_s
        climb_elapsed_s = max(0.0, t - release_s)
        if climb_elapsed_s < self._surface_exit_s:
            self._surface_exit_guard_elapsed_s = max(
                0.0, release_s - (self._surface_exit_guard_start_s or release_s)
            )
            z_ref_m = self._lerp(guard_z_ref_m, -self._air_height,
                                 climb_elapsed_s / self._surface_exit_s)
            return z_ref_m, 'S2_SURFACE_EXIT'

        self._surface_exit_guard_state = 'done'
        return -self._air_height, 'S5_AIR_HOVER'

    def _local_z_from_z_m(self, z_m: float) -> float:
        if self._world_z0 is None or self._local_z0 is None:
            return 0.0
        world_z_ref = -float(z_m)
        return self._local_z0 + (self._world_z0 - world_z_ref)

    def _write_log_row(self, t_rel: float, z_ref_m: float, command_label: str):
        odom = self._latest_odom
        local_position = self._latest_local_position
        odom_available = odom is not None
        if odom_available:
            pos_x = odom.pose.pose.position.x
            pos_y = odom.pose.pose.position.y
            pos_z = odom.pose.pose.position.z
            lin_vx = odom.twist.twist.linear.x
            lin_vy = odom.twist.twist.linear.y
            lin_vz = odom.twist.twist.linear.z
            ang_vx = odom.twist.twist.angular.x
            ang_vy = odom.twist.twist.angular.y
            ang_vz = odom.twist.twist.angular.z
            roll, pitch, yaw = self._quaternion_to_euler([
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ])
        else:
            pos_x, pos_y, pos_z = self._local_position_fallback_xyz(local_position)
            lin_vx = getattr(local_position, 'vx', None)
            lin_vy = getattr(local_position, 'vy', None)
            lin_vz = getattr(local_position, 'vz', None)
            ang_vx = None
            ang_vy = None
            ang_vz = None
            roll = None
            pitch = None
            yaw = (
                float(local_position.heading)
                if local_position is not None and math.isfinite(local_position.heading)
                else None
            )

        # Per-propeller diagnostics: [cmd, omega, thrust, torque, sub, domain].
        cmd = [self._prop_diag[i][0] for i in range(4)]
        omega = [self._prop_diag[i][1] for i in range(4)]
        T = [self._prop_diag[i][2] for i in range(4)]
        T_sum = sum(T)
        RPM = [value * 30.0 / math.pi for value in omega]
        domain_code = [self._prop_diag[i][5] for i in range(4)]
        per_prop_guard_thrust = self._surface_exit_guard_min_thrust / 4.0
        thrust_margin = [value - per_prop_guard_thrust for value in T]
        thrust_margin_avg = sum(thrust_margin) / 4.0
        thrust_margin_min = min(thrust_margin)

        sub = self._mean_prop_diag(4)
        domain_state = self._domain_state(sub)
        event_label = self._event_label(t_rel, pos_z, sub)
        nav_state = ''
        arming_state = ''
        arming_reason = ''
        disarming_reason = ''
        air_water_mode = ''
        command_ack = ''
        if self._latest_status is not None:
            nav_state = str(int(self._latest_status.nav_state))
            arming_state = str(int(self._latest_status.arming_state))
            arming_reason = str(int(self._latest_status.latest_arming_reason))
            disarming_reason = str(int(self._latest_status.latest_disarming_reason))
        if self._latest_air_water_status is not None:
            air_water_mode = str(int(self._latest_air_water_status.uw_transit_mode))
        if self._latest_command_ack is not None:
            command_ack = (
                f'{int(self._latest_command_ack.command)}:'
                f'{int(self._latest_command_ack.result)}'
            )
        notes = (
            f'odom={1 if odom_available else 0}; '
            'pos_x/y/z Gazebo world frame Z-up when odom=1, PX4 local fallback when odom=0; '
            f'nav={nav_state}; arm={arming_state}; '
            f'arming_reason={arming_reason}; disarming_reason={disarming_reason}; '
            f'uw_transit_mode={air_water_mode}; ack={command_ack}; '
            f'{self._preflight_notes()}'
        )

        self._writer.writerow({
            't': self._fmt(t_rel),
            'pos_x': self._fmt(pos_x),
            'pos_y': self._fmt(pos_y),
            'pos_z': self._fmt(pos_z),
            'roll': self._fmt(roll),
            'pitch': self._fmt(pitch),
            'yaw': self._fmt(yaw),
            'lin_vx': self._fmt(lin_vx),
            'lin_vy': self._fmt(lin_vy),
            'lin_vz': self._fmt(lin_vz),
            'ang_vx': self._fmt(ang_vx),
            'ang_vy': self._fmt(ang_vy),
            'ang_vz': self._fmt(ang_vz),
            'T1': self._fmt(T[0]),
            'T2': self._fmt(T[1]),
            'T3': self._fmt(T[2]),
            'T4': self._fmt(T[3]),
            'T_sum': self._fmt(T_sum),
            'RPM1': self._fmt(RPM[0]),
            'RPM2': self._fmt(RPM[1]),
            'RPM3': self._fmt(RPM[2]),
            'RPM4': self._fmt(RPM[3]),
            'cmd1': self._fmt(cmd[0]),
            'cmd2': self._fmt(cmd[1]),
            'cmd3': self._fmt(cmd[2]),
            'cmd4': self._fmt(cmd[3]),
            'omega1': self._fmt(omega[0]),
            'omega2': self._fmt(omega[1]),
            'omega3': self._fmt(omega[2]),
            'omega4': self._fmt(omega[3]),
            'domain_code1': self._fmt(domain_code[0]),
            'domain_code2': self._fmt(domain_code[1]),
            'domain_code3': self._fmt(domain_code[2]),
            'domain_code4': self._fmt(domain_code[3]),
            'thrust_margin_avg_N': self._fmt(thrust_margin_avg),
            'thrust_margin_min_N': self._fmt(thrust_margin_min),
            'surface_exit_guard_state': self._surface_exit_guard_state,
            'surface_exit_guard_elapsed_s': self._fmt(self._surface_exit_guard_elapsed_s),
            'surface_exit_guard_min_thrust_N': self._fmt(self._surface_exit_guard_min_thrust),
            'source': 'gazebo_px4',
            'trial_id': self._trial_id,
            'condition': self._condition,
            'z_ref_m': self._fmt(z_ref_m),
            'vz_ref_mps': self._fmt(self._vz_ref_mps),
            'command': command_label,
            'domain_state': domain_state,
            'event_label': event_label,
            'submersion_ratio': self._fmt(sub),
            'control_source': 'px4_offboard_position_velocity_ff',
            'notes': notes,
        })
        self._log_file.flush()

    def _local_position_fallback_xyz(self, local_position: Optional[VehicleLocalPosition]):
        if local_position is None:
            return None, None, None
        if self._local_x0 is None or self._local_y0 is None or self._local_z0 is None:
            return float(local_position.x), float(local_position.y), -float(local_position.z)
        x = float(local_position.x) - self._local_x0
        y = float(local_position.y) - self._local_y0
        z = self._world_z0 - (float(local_position.z) - self._local_z0)
        return x, y, z

    def _mean_prop_diag(self, index: int) -> float:
        return sum(self._prop_diag[i][index] for i in range(4)) / 4.0

    def _total_prop_thrust(self) -> float:
        return sum(self._prop_diag[i][2] for i in range(4))

    def _domain_state(self, sub: float) -> str:
        if sub <= 0.05:
            return 'air'
        if sub >= 0.95:
            return 'water'
        return 'transition'

    def _event_label(self, t: float, pos_z: float, sub: float) -> str:
        candidates = []
        if t <= 0.05:
            candidates.append('start')
        if pos_z <= 0.0 and 'contact_surface' not in self._last_event_labels:
            candidates.append('contact_surface')
        if sub >= 0.95 and 'fully_submerged' not in self._last_event_labels:
            candidates.append('fully_submerged')
        if sub <= 0.05 and 'exit_surface' not in self._last_event_labels:
            candidates.append('exit_surface')
        if t >= self._duration_s - 0.05 and 'end' not in self._last_event_labels:
            candidates.append('end')
        if not candidates:
            return ''
        label = candidates[0]
        self._last_event_labels.add(label)
        return label

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

    def _timestamp_us(self) -> int:
        # PX4 uXRCE-DDS timesync is wall/steady-clock based.  Gazebo /clock can
        # pause, stall, or jump when the simulator hiccups, so keep it only for
        # experiment timing / CSV and never stamp PX4 input messages with it.
        return int(self.get_clock().now().nanoseconds / 1000)

    def _lerp(self, a: float, b: float, u: float) -> float:
        return a + (b - a) * max(0.0, min(1.0, u))

    def _fmt(self, value) -> str:
        if value is None:
            return ''
        value = float(value)
        if not math.isfinite(value):
            return ''
        return f'{value:.6f}'


def main(args=None):
    rclpy.init(args=args)
    node = TeleH4ZZAxisCrossDomainRunner()
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
