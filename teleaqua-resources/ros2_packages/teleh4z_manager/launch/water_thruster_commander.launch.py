from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='teleh4z_0',
        description='Gazebo model instance name',
    )
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='20.0',
        description='Water thruster command publish rate',
    )

    model_name = LaunchConfiguration('model_name')
    rate_hz = LaunchConfiguration('rate_hz')

    bridges = []
    for index in range(4):
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'teleh4z_water_thruster_{index}_bridge',
            arguments=[
                [
                    '/',
                    model_name,
                    f'/servo_{index}@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
                ],
            ],
            output='screen',
        ))

    commander = Node(
        package='teleh4z_manager',
        executable='water_thruster_commander',
        name='teleh4z_water_thruster_commander',
        parameters=[{
            'model_name': model_name,
            'rate_hz': rate_hz,
        }],
        output='screen',
    )

    return LaunchDescription([
        model_name_arg,
        rate_hz_arg,
        *bridges,
        commander,
    ])
