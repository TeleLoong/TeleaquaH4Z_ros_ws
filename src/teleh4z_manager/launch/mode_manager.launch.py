from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    model_name = LaunchConfiguration("model_name")
    world_name = LaunchConfiguration("world_name")
    arm_move_duration = LaunchConfiguration("arm_move_duration")
    arm_settle_duration = LaunchConfiguration("arm_settle_duration")
    arm_motion_timeout = LaunchConfiguration("arm_motion_timeout")
    px4_switch_timeout = LaunchConfiguration("px4_switch_timeout")

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

    return LaunchDescription([
        model_name_arg,
        world_name_arg,
        arm_move_duration_arg,
        arm_settle_duration_arg,
        arm_motion_timeout_arg,
        px4_switch_timeout_arg,
        trajectory_bridge_node,
        joint_state_bridge_node,
        transition_state_bridge_node,
        camera_bridge_node,
        pose_bridge_node,
        manager_node,
    ])
