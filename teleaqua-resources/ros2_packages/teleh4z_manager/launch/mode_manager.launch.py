from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="teleh4z_0",
        description="Gazebo model instance name (with _0 suffix)",
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="playground",
        description="Gazebo world name used in joint state topic path",
    )

    arm_move_duration_arg = DeclareLaunchArgument(
        "arm_move_duration",
        default_value="2.0",
        description="Arm trajectory duration in seconds",
    )

    arm_settle_duration_arg = DeclareLaunchArgument(
        "arm_settle_duration",
        default_value="0.25",
        description="Seconds joints must remain within tolerance before completion",
    )

    arm_motion_timeout_arg = DeclareLaunchArgument(
        "arm_motion_timeout",
        default_value="8.0",
        description="Timeout waiting for arm trajectory completion",
    )

    px4_switch_timeout_arg = DeclareLaunchArgument(
        "px4_switch_timeout",
        default_value="5.0",
        description="Timeout in seconds waiting for PX4 mode confirmation",
    )

    enable_joystick_mapper_arg = DeclareLaunchArgument(
        "enable_joystick_mapper",
        default_value="true",
        description="Start joystick button mapper for TeleH4Z mode requests",
    )

    joystick_air_button_bit_arg = DeclareLaunchArgument(
        "joystick_air_button_bit",
        default_value="3",
        description="ManualControlSetpoint.buttons bit that requests air mode",
    )

    joystick_water_button_bit_arg = DeclareLaunchArgument(
        "joystick_water_button_bit",
        default_value="0",
        description="ManualControlSetpoint.buttons bit that requests water mode",
    )

    joystick_neutral_button_bit_arg = DeclareLaunchArgument(
        "joystick_neutral_button_bit",
        default_value="1",
        description="ManualControlSetpoint.buttons bit that publishes neutral water-thruster PWM",
    )

    joystick_enable_neutral_arg = DeclareLaunchArgument(
        "joystick_enable_neutral",
        default_value="true",
        description="Enable neutral water-thruster PWM button action",
    )

    joystick_cooldown_sec_arg = DeclareLaunchArgument(
        "joystick_cooldown_sec",
        default_value="1.0",
        description="Minimum seconds between joystick button actions",
    )

    enable_water_thruster_joystick_arg = DeclareLaunchArgument(
        "enable_water_thruster_joystick",
        default_value="true",
        description="Start sim-only manual-control to water-thruster PWM mapper",
    )

    water_thruster_command_rate_hz_arg = DeclareLaunchArgument(
        "water_thruster_command_rate_hz",
        default_value="30.0",
        description="Water thruster commander publish rate",
    )

    water_thruster_command_model_name_arg = DeclareLaunchArgument(
        "water_thruster_command_model_name",
        default_value="teleh4z_manual",
        description="ROS/Gazebo topic prefix for sim-only water-thruster bypass commands",
    )

    water_thruster_surge_axis_arg = DeclareLaunchArgument(
        "water_thruster_surge_axis",
        default_value="pitch",
        description="ManualControlSetpoint axis for forward/back water-thruster command",
    )

    water_thruster_yaw_axis_arg = DeclareLaunchArgument(
        "water_thruster_yaw_axis",
        default_value="yaw",
        description="ManualControlSetpoint axis for differential yaw water-thruster command",
    )

    water_thruster_heave_axis_arg = DeclareLaunchArgument(
        "water_thruster_heave_axis",
        default_value="throttle",
        description="ManualControlSetpoint axis for vertical water-thruster command",
    )

    water_thruster_pitch_axis_arg = DeclareLaunchArgument(
        "water_thruster_pitch_axis",
        default_value="roll",
        description="ManualControlSetpoint axis for front/back pitch-trim water-thruster command",
    )

    water_thruster_surge_pwm_scale_arg = DeclareLaunchArgument(
        "water_thruster_surge_pwm_scale",
        default_value="-280.0",
        description="PWM delta for full forward/back manual pitch input",
    )

    water_thruster_yaw_pwm_scale_arg = DeclareLaunchArgument(
        "water_thruster_yaw_pwm_scale",
        default_value="180.0",
        description="PWM delta for full yaw manual input",
    )

    water_thruster_heave_pwm_scale_arg = DeclareLaunchArgument(
        "water_thruster_heave_pwm_scale",
        default_value="260.0",
        description="PWM delta for full vertical manual throttle input",
    )

    water_thruster_pitch_pwm_scale_arg = DeclareLaunchArgument(
        "water_thruster_pitch_pwm_scale",
        default_value="160.0",
        description="PWM delta for full pitch-trim manual roll input",
    )

    model_name = LaunchConfiguration("model_name")
    world_name = LaunchConfiguration("world_name")
    arm_move_duration = LaunchConfiguration("arm_move_duration")
    arm_settle_duration = LaunchConfiguration("arm_settle_duration")
    arm_motion_timeout = LaunchConfiguration("arm_motion_timeout")
    px4_switch_timeout = LaunchConfiguration("px4_switch_timeout")
    enable_joystick_mapper = LaunchConfiguration("enable_joystick_mapper")
    joystick_air_button_bit = LaunchConfiguration("joystick_air_button_bit")
    joystick_water_button_bit = LaunchConfiguration("joystick_water_button_bit")
    joystick_neutral_button_bit = LaunchConfiguration("joystick_neutral_button_bit")
    joystick_enable_neutral = LaunchConfiguration("joystick_enable_neutral")
    joystick_cooldown_sec = LaunchConfiguration("joystick_cooldown_sec")
    enable_water_thruster_joystick = LaunchConfiguration("enable_water_thruster_joystick")
    water_thruster_command_rate_hz = LaunchConfiguration("water_thruster_command_rate_hz")
    water_thruster_command_model_name = LaunchConfiguration("water_thruster_command_model_name")
    water_thruster_surge_axis = LaunchConfiguration("water_thruster_surge_axis")
    water_thruster_yaw_axis = LaunchConfiguration("water_thruster_yaw_axis")
    water_thruster_heave_axis = LaunchConfiguration("water_thruster_heave_axis")
    water_thruster_pitch_axis = LaunchConfiguration("water_thruster_pitch_axis")
    water_thruster_surge_pwm_scale = LaunchConfiguration("water_thruster_surge_pwm_scale")
    water_thruster_yaw_pwm_scale = LaunchConfiguration("water_thruster_yaw_pwm_scale")
    water_thruster_heave_pwm_scale = LaunchConfiguration("water_thruster_heave_pwm_scale")
    water_thruster_pitch_pwm_scale = LaunchConfiguration("water_thruster_pitch_pwm_scale")

    trajectory_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="teleh4z_arm_trajectory_bridge",
        arguments=[
            "/arm_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
        ],
        output="screen",
    )

    joint_state_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="teleh4z_joint_state_bridge",
        arguments=[
            [
                "/world/",
                world_name,
                "/model/",
                model_name,
                "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            ],
        ],
        output="screen",
    )

    transition_state_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="teleh4z_transition_state_bridge",
        arguments=[
            [
                "/model/",
                model_name,
                "/transition_state@ros_gz_interfaces/msg/Float32Array[gz.msgs.Float_V",
            ],
        ],
        output="screen",
    )

    camera_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="teleh4z_camera_bridge",
        arguments=[
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    pose_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="teleh4z_pose_bridge",
        arguments=[
            [
                "/model/",
                model_name,
                "/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            ],
        ],
        output="screen",
    )

    manager_node = Node(
        package="teleh4z_manager",
        executable="mode_manager",
        name="teleh4z_mode_manager",
        parameters=[{
            "model_name": model_name,
            "world_name": world_name,
            "arm_move_duration": arm_move_duration,
            "arm_settle_duration": arm_settle_duration,
            "arm_motion_timeout": arm_motion_timeout,
            "px4_switch_timeout": px4_switch_timeout,
        }],
        output="screen",
    )

    joystick_mapper_node = Node(
        package="teleh4z_manager",
        executable="joystick_mapper",
        name="teleh4z_joystick_mapper",
        condition=IfCondition(enable_joystick_mapper),
        parameters=[{
            "air_button_bit": ParameterValue(joystick_air_button_bit, value_type=int),
            "water_button_bit": ParameterValue(joystick_water_button_bit, value_type=int),
            "neutral_button_bit": ParameterValue(joystick_neutral_button_bit, value_type=int),
            "enable_neutral_button": ParameterValue(joystick_enable_neutral, value_type=bool),
            "cooldown_sec": ParameterValue(joystick_cooldown_sec, value_type=float),
        }],
        output="screen",
    )

    water_thruster_bridge_nodes = []
    for index in range(4):
        water_thruster_bridge_nodes.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"teleh4z_water_thruster_{index}_bridge",
            condition=IfCondition(enable_water_thruster_joystick),
            arguments=[
                [
                    "/",
                    water_thruster_command_model_name,
                    f"/servo_{index}@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
                ],
            ],
            output="screen",
        ))

    water_thruster_commander_node = Node(
        package="teleh4z_manager",
        executable="water_thruster_commander",
        name="teleh4z_water_thruster_commander",
        condition=IfCondition(enable_water_thruster_joystick),
        parameters=[{
            "model_name": water_thruster_command_model_name,
            "rate_hz": ParameterValue(water_thruster_command_rate_hz, value_type=float),
        }],
        output="screen",
    )

    manual_water_thruster_mapper_node = Node(
        package="teleh4z_manager",
        executable="manual_water_thruster_mapper",
        name="teleh4z_manual_water_thruster_mapper",
        condition=IfCondition(enable_water_thruster_joystick),
        parameters=[{
            "surge_axis": water_thruster_surge_axis,
            "yaw_axis": water_thruster_yaw_axis,
            "heave_axis": water_thruster_heave_axis,
            "pitch_axis": water_thruster_pitch_axis,
            "surge_pwm_scale": ParameterValue(water_thruster_surge_pwm_scale, value_type=float),
            "yaw_pwm_scale": ParameterValue(water_thruster_yaw_pwm_scale, value_type=float),
            "heave_pwm_scale": ParameterValue(water_thruster_heave_pwm_scale, value_type=float),
            "pitch_pwm_scale": ParameterValue(water_thruster_pitch_pwm_scale, value_type=float),
        }],
        output="screen",
    )

    return LaunchDescription([
        model_name_arg,
        world_name_arg,
        arm_move_duration_arg,
        arm_settle_duration_arg,
        arm_motion_timeout_arg,
        px4_switch_timeout_arg,
        enable_joystick_mapper_arg,
        joystick_air_button_bit_arg,
        joystick_water_button_bit_arg,
        joystick_neutral_button_bit_arg,
        joystick_enable_neutral_arg,
        joystick_cooldown_sec_arg,
        enable_water_thruster_joystick_arg,
        water_thruster_command_rate_hz_arg,
        water_thruster_command_model_name_arg,
        water_thruster_surge_axis_arg,
        water_thruster_yaw_axis_arg,
        water_thruster_heave_axis_arg,
        water_thruster_pitch_axis_arg,
        water_thruster_surge_pwm_scale_arg,
        water_thruster_yaw_pwm_scale_arg,
        water_thruster_heave_pwm_scale_arg,
        water_thruster_pitch_pwm_scale_arg,
        trajectory_bridge_node,
        joint_state_bridge_node,
        transition_state_bridge_node,
        camera_bridge_node,
        pose_bridge_node,
        manager_node,
        joystick_mapper_node,
        *water_thruster_bridge_nodes,
        water_thruster_commander_node,
        manual_water_thruster_mapper_node,
    ])
