from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value='',
        description='Experiment CSV path with columns time,pwm_1,pwm_2,pwm_3,pwm_4',
    )
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='teleh4z_0',
        description='Gazebo model instance name',
    )
    mirror_sum_arg = DeclareLaunchArgument(
        'mirror_sum',
        default_value='3000.0',
        description='Board-to-plugin PWM mirror sum',
    )
    start_delay_s_arg = DeclareLaunchArgument(
        'start_delay_s',
        default_value='0.0',
        description='Replay start delay in Gazebo simulation seconds',
    )
    output_csv_path_arg = DeclareLaunchArgument(
        'output_csv_path',
        default_value='/tmp/teleh4z_gazebo_replay_log.csv',
        description='Output CSV path for Gazebo u-axis replay log',
    )

    csv_path = LaunchConfiguration('csv_path')
    model_name = LaunchConfiguration('model_name')
    mirror_sum = LaunchConfiguration('mirror_sum')
    start_delay_s = LaunchConfiguration('start_delay_s')
    output_csv_path = LaunchConfiguration('output_csv_path')

    bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='teleh4z_replay_clock_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='teleh4z_replay_odometry_bridge',
            arguments=[
                [
                    '/model/',
                    model_name,
                    '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                ],
            ],
            output='screen',
        ),
    ]

    for index in range(4):
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'teleh4z_replay_servo_{index}_bridge',
            arguments=[
                [
                    '/',
                    model_name,
                    f'/servo_{index}@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
                ],
            ],
            output='screen',
        ))

    replay = Node(
        package='teleh4z_manager',
        executable='water_thruster_replay',
        name='teleh4z_water_thruster_replay',
        parameters=[{
            'csv_path': csv_path,
            'model_name': model_name,
            'mirror_sum': mirror_sum,
            'start_delay_s': start_delay_s,
            'output_csv_path': output_csv_path,
        }],
        output='screen',
    )

    return LaunchDescription([
        csv_path_arg,
        model_name_arg,
        mirror_sum_arg,
        start_delay_s_arg,
        output_csv_path_arg,
        *bridges,
        replay,
    ])
