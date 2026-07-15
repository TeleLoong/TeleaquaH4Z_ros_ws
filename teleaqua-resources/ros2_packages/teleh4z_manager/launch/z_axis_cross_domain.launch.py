from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='teleh4z_zaxis_0',
        description='Gazebo model instance name for the Z-axis experiment',
    )
    output_csv_path_arg = DeclareLaunchArgument(
        'output_csv_path',
        default_value='/tmp/teleh4z_z_axis_cross_domain_gazebo.csv',
        description='Output CSV path for Gazebo Z-axis cross-domain log',
    )
    trial_id_arg = DeclareLaunchArgument(
        'trial_id',
        default_value='0',
        description='Trial id written to the output CSV',
    )
    condition_arg = DeclareLaunchArgument(
        'condition',
        default_value='nominal',
        description='Condition label written to the output CSV',
    )
    start_delay_s_arg = DeclareLaunchArgument(
        'start_delay_s',
        default_value='1.0',
        description='Delay after /clock starts before the scripted task begins',
    )
    offboard_arm_delay_s_arg = DeclareLaunchArgument(
        'offboard_arm_delay_s',
        default_value='0.1',
        description='Delay after task start before requesting OFFBOARD and ARM',
    )
    prearm_settle_s_arg = DeclareLaunchArgument(
        'prearm_settle_s',
        default_value='2.0',
        description='Extra startup settle time before OFFBOARD and ARM requests',
    )
    offboard_confirm_settle_s_arg = DeclareLaunchArgument(
        'offboard_confirm_settle_s',
        default_value='0.5',
        description='Time to wait after OFFBOARD is confirmed before requesting ARM',
    )
    require_preflight_checks_before_arm_arg = DeclareLaunchArgument(
        'require_preflight_checks_before_arm',
        default_value='true',
        description='Require PX4 preflight health checks before arming',
    )
    preflight_ready_timeout_s_arg = DeclareLaunchArgument(
        'preflight_ready_timeout_s',
        default_value='30.0',
        description='Time before reporting that PX4 preflight health is not ready',
    )
    disable_auto_disarm_arg = DeclareLaunchArgument(
        'disable_auto_disarm',
        default_value='true',
        description='Publish PX4 config override to prevent automatic landing disarm',
    )
    require_prop_diag_before_arm_arg = DeclareLaunchArgument(
        'require_prop_diag_before_arm',
        default_value='true',
        description='Require fresh hybrid propeller diagnostics before arming',
    )
    prop_diag_ready_timeout_s_arg = DeclareLaunchArgument(
        'prop_diag_ready_timeout_s',
        default_value='12.0',
        description='Time before reporting missing propeller diagnostics',
    )
    water_depth_m_arg = DeclareLaunchArgument(
        'water_depth_m',
        default_value='1.0',
        description='Underwater hold depth below the water surface, in meters',
    )
    air_height_m_arg = DeclareLaunchArgument(
        'air_height_m',
        default_value='0.7',
        description='Final hover height above the water surface, in meters',
    )
    underwater_hover_s_arg = DeclareLaunchArgument(
        'underwater_hover_s',
        default_value='5.0',
        description='Duration of the underwater hover segment',
    )
    underwater_ascent_s_arg = DeclareLaunchArgument(
        'underwater_ascent_s',
        default_value='5.0',
        description='Duration of the underwater ascent segment',
    )
    underwater_ascent_target_z_m_arg = DeclareLaunchArgument(
        'underwater_ascent_target_z_m',
        default_value='-0.10',
        description='Z reference at the end of underwater ascent; negative is above the surface',
    )
    surface_exit_s_arg = DeclareLaunchArgument(
        'surface_exit_s',
        default_value='5.0',
        description='Duration of the surface exit segment',
    )
    surface_exit_guard_enabled_arg = DeclareLaunchArgument(
        'surface_exit_guard_enabled',
        default_value='true',
        description='Hold a shallow above-surface target until air thrust is established',
    )
    surface_exit_guard_height_m_arg = DeclareLaunchArgument(
        'surface_exit_guard_height_m',
        default_value='0.40',
        description='Guard setpoint height above the water surface, in meters',
    )
    surface_exit_guard_timeout_s_arg = DeclareLaunchArgument(
        'surface_exit_guard_timeout_s',
        default_value='0.0',
        description='Maximum guard hold time before resuming the air climb',
    )
    surface_exit_guard_thrust_fraction_arg = DeclareLaunchArgument(
        'surface_exit_guard_thrust_fraction',
        default_value='0.8',
        description='Fraction of air hover thrust required before guard release',
    )
    air_hover_thrust_N_arg = DeclareLaunchArgument(
        'air_hover_thrust_N',
        default_value='16.7',
        description='Estimated total air hover thrust used by the surface exit guard',
    )

    model_name = LaunchConfiguration('model_name')
    output_csv_path = LaunchConfiguration('output_csv_path')
    trial_id = LaunchConfiguration('trial_id')
    condition = LaunchConfiguration('condition')
    start_delay_s = LaunchConfiguration('start_delay_s')
    offboard_arm_delay_s = LaunchConfiguration('offboard_arm_delay_s')
    prearm_settle_s = LaunchConfiguration('prearm_settle_s')
    offboard_confirm_settle_s = LaunchConfiguration('offboard_confirm_settle_s')
    require_preflight_checks_before_arm = LaunchConfiguration(
        'require_preflight_checks_before_arm'
    )
    preflight_ready_timeout_s = LaunchConfiguration('preflight_ready_timeout_s')
    disable_auto_disarm = LaunchConfiguration('disable_auto_disarm')
    require_prop_diag_before_arm = LaunchConfiguration('require_prop_diag_before_arm')
    prop_diag_ready_timeout_s = LaunchConfiguration('prop_diag_ready_timeout_s')
    water_depth_m = LaunchConfiguration('water_depth_m')
    air_height_m = LaunchConfiguration('air_height_m')
    underwater_hover_s = LaunchConfiguration('underwater_hover_s')
    underwater_ascent_s = LaunchConfiguration('underwater_ascent_s')
    underwater_ascent_target_z_m = LaunchConfiguration('underwater_ascent_target_z_m')
    surface_exit_s = LaunchConfiguration('surface_exit_s')
    surface_exit_guard_enabled = LaunchConfiguration('surface_exit_guard_enabled')
    surface_exit_guard_height_m = LaunchConfiguration('surface_exit_guard_height_m')
    surface_exit_guard_timeout_s = LaunchConfiguration('surface_exit_guard_timeout_s')
    surface_exit_guard_thrust_fraction = LaunchConfiguration('surface_exit_guard_thrust_fraction')
    air_hover_thrust_N = LaunchConfiguration('air_hover_thrust_N')

    bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='teleh4z_zaxis_clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='teleh4z_zaxis_odometry_bridge',
            arguments=[[
                '/model/',
                model_name,
                '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ]],
            output='screen',
        ),
    ]

    for index in range(4):
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'teleh4z_zaxis_prop_{index}_diag_bridge',
            arguments=[[
                '/model/',
                model_name,
                f'/hybrid_air_propeller_{index}/state@ros_gz_interfaces/msg/Float32Array[gz.msgs.Float_V',
            ]],
            output='screen',
        ))

    runner = Node(
        package='teleh4z_manager',
        executable='z_axis_cross_domain_runner',
        name='teleh4z_z_axis_cross_domain_runner',
        parameters=[{
            'model_name': model_name,
            'output_csv_path': output_csv_path,
            'trial_id': ParameterValue(trial_id, value_type=int),
            'condition': condition,
            'start_delay_s': ParameterValue(start_delay_s, value_type=float),
            'offboard_arm_delay_s': ParameterValue(offboard_arm_delay_s, value_type=float),
            'prearm_settle_s': ParameterValue(prearm_settle_s, value_type=float),
            'offboard_confirm_settle_s': ParameterValue(
                offboard_confirm_settle_s, value_type=float
            ),
            'require_preflight_checks_before_arm': ParameterValue(
                require_preflight_checks_before_arm, value_type=bool
            ),
            'preflight_ready_timeout_s': ParameterValue(
                preflight_ready_timeout_s, value_type=float
            ),
            'disable_auto_disarm': ParameterValue(disable_auto_disarm, value_type=bool),
            'require_prop_diag_before_arm': ParameterValue(
                require_prop_diag_before_arm, value_type=bool
            ),
            'prop_diag_ready_timeout_s': ParameterValue(
                prop_diag_ready_timeout_s, value_type=float
            ),
            'water_depth_m': ParameterValue(water_depth_m, value_type=float),
            'air_height_m': ParameterValue(air_height_m, value_type=float),
            'underwater_hover_s': ParameterValue(underwater_hover_s, value_type=float),
            'underwater_ascent_s': ParameterValue(underwater_ascent_s, value_type=float),
            'underwater_ascent_target_z_m': ParameterValue(
                underwater_ascent_target_z_m, value_type=float
            ),
            'surface_exit_s': ParameterValue(surface_exit_s, value_type=float),
            'surface_exit_guard_enabled': ParameterValue(
                surface_exit_guard_enabled, value_type=bool
            ),
            'surface_exit_guard_height_m': ParameterValue(
                surface_exit_guard_height_m, value_type=float
            ),
            'surface_exit_guard_timeout_s': ParameterValue(
                surface_exit_guard_timeout_s, value_type=float
            ),
            'surface_exit_guard_thrust_fraction': ParameterValue(
                surface_exit_guard_thrust_fraction, value_type=float
            ),
            'air_hover_thrust_N': ParameterValue(air_hover_thrust_N, value_type=float),
        }],
        output='screen',
    )

    return LaunchDescription([
        model_name_arg,
        output_csv_path_arg,
        trial_id_arg,
        condition_arg,
        start_delay_s_arg,
        offboard_arm_delay_s_arg,
        prearm_settle_s_arg,
        offboard_confirm_settle_s_arg,
        require_preflight_checks_before_arm_arg,
        preflight_ready_timeout_s_arg,
        disable_auto_disarm_arg,
        require_prop_diag_before_arm_arg,
        prop_diag_ready_timeout_s_arg,
        water_depth_m_arg,
        air_height_m_arg,
        underwater_hover_s_arg,
        underwater_ascent_s_arg,
        underwater_ascent_target_z_m_arg,
        surface_exit_s_arg,
        surface_exit_guard_enabled_arg,
        surface_exit_guard_height_m_arg,
        surface_exit_guard_timeout_s_arg,
        surface_exit_guard_thrust_fraction_arg,
        air_hover_thrust_N_arg,
        *bridges,
        runner,
    ])
