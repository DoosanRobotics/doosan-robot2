#
#  dsr_bringup2
#  Mobile Manipulator dual-arm MoveIt launcher (R100 + dual M1013)
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('model', default_value='r100_m1013_dual', description='ROBOT_MODEL'),
        DeclareLaunchArgument('name', default_value=LaunchConfiguration('model'), description='NAME_SPACE'),
        DeclareLaunchArgument('host', default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('left_host', default_value=LaunchConfiguration('host'), description='LEFT_ROBOT_IP'),
        DeclareLaunchArgument('right_host', default_value=LaunchConfiguration('host'), description='RIGHT_ROBOT_IP'),
        DeclareLaunchArgument('port', default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('left_port', default_value=LaunchConfiguration('port'), description='LEFT_ROBOT_PORT'),
        DeclareLaunchArgument('right_port', default_value='12348', description='RIGHT_ROBOT_PORT'),
        DeclareLaunchArgument('mode', default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotiq_2f85)'),
        DeclareLaunchArgument('gui', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('left_init_on_start', default_value='true', description='Move left arm J1 to 180deg once before MoveIt/RViz'),
        DeclareLaunchArgument('enable_whole_body_controller', default_value='true', description='Spawn whole_body_controller scaffold plugin'),
        DeclareLaunchArgument('enable_rviz_refresh', default_value='true', description='Auto-refresh planning scene after execute'),
        DeclareLaunchArgument('use_nav2', default_value='true', description='Start Nav2 navigation stack'),
        DeclareLaunchArgument('nav2_start_delay', default_value='8.0', description='Delay (sec) before starting Nav2 to wait for odom->base_link TF'),
        DeclareLaunchArgument('use_map', default_value='false', description='Use map_server + AMCL localization'),
        DeclareLaunchArgument('enable_nav2_fallback', default_value='false', description='Call lifecycle_manager startup fallback'),
        DeclareLaunchArgument('map', default_value='', description='Map yaml for map mode (use_map:=true)'),
        DeclareLaunchArgument(
            'nav2_params_file_no_map',
            default_value=PathJoinSubstitution([FindPackageShare('dsr_bringup2'), 'config', 'nav2_params_no_map.yaml']),
            description='Nav2 parameter file for no-map mode',
        ),
        DeclareLaunchArgument(
            'nav2_params_file_map',
            default_value=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml']),
            description='Nav2 parameter file for map mode',
        ),
    ]

    selected_nav2_params_file = PythonExpression([
        "'", LaunchConfiguration('nav2_params_file_map'),
        "' if '", LaunchConfiguration('use_map'),
        "'.lower() in ['true','1'] else '",
        LaunchConfiguration('nav2_params_file_no_map'), "'"
    ])

    nav2_no_map_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('use_nav2'),
        "'.lower() in ['true','1'] and '",
        LaunchConfiguration('use_map'),
        "'.lower() not in ['true','1']"
    ]))

    nav2_map_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('use_nav2'),
        "'.lower() in ['true','1'] and '",
        LaunchConfiguration('use_map'),
        "'.lower() in ['true','1']"
    ]))

    moveit_config_pkg = PythonExpression([
        "'dsr_moveit_config_' + '", LaunchConfiguration('model'), "'"
    ])

    start_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_pkg), 'launch', 'start.launch.py']
            )
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'host': LaunchConfiguration('host'),
            'left_host': LaunchConfiguration('left_host'),
            'right_host': LaunchConfiguration('right_host'),
            'port': LaunchConfiguration('port'),
            'left_port': LaunchConfiguration('left_port'),
            'right_port': LaunchConfiguration('right_port'),
            # Keep this launcher virtual-only to match current support scope.
            'mode': 'virtual',
            'model': LaunchConfiguration('model'),
            'color': LaunchConfiguration('color'),
            'rt_host': LaunchConfiguration('rt_host'),
            'gripper': LaunchConfiguration('gripper'),
            'gui': LaunchConfiguration('gui'),
            'left_init_on_start': LaunchConfiguration('left_init_on_start'),
            'enable_whole_body_controller': LaunchConfiguration('enable_whole_body_controller'),
            'enable_rviz_refresh': LaunchConfiguration('enable_rviz_refresh'),
            'dynamic_yaml': 'true',
        }.items(),
    )

    static_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        condition=UnlessCondition(LaunchConfiguration('use_nav2')),
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=nav2_no_map_condition,
    )

    static_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'namespace': '',
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'use_smoother': 'False',
            'params_file': selected_nav2_params_file,
            'log_level': 'info',
        }.items(),
        condition=nav2_map_condition,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'false',
            'autostart': 'false',
            'use_composition': 'False',
            'use_respawn': 'False',
            'use_smoother': 'False',
            'params_file': selected_nav2_params_file,
            'log_level': 'info',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    nav2_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        actions=[
            SetRemap(src='/cmd_vel', dst=PythonExpression(["'/' + '", LaunchConfiguration('name'), "' + '/diff_drive_controller/cmd_vel'"])),
            SetRemap(src='/cmd_vel_nav', dst=PythonExpression(["'/' + '", LaunchConfiguration('name'), "' + '/diff_drive_controller/cmd_vel'"])),
            SetRemap(src='/odom', dst=PythonExpression(["'/' + '", LaunchConfiguration('name'), "' + '/diff_drive_controller/odom'"])),
            localization_launch,
            nav2_launch,
        ],
    )
    delayed_nav2_group = TimerAction(
        period=LaunchConfiguration('nav2_start_delay'),
        actions=[nav2_group],
    )

    nav2_startup_fallback = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/lifecycle_manager_navigation/manage_nodes',
                    'nav2_msgs/srv/ManageLifecycleNodes',
                    '{command: 0}',
                ],
                output='screen',
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_nav2'),
            "'.lower() in ['true','1'] and '",
            LaunchConfiguration('enable_nav2_fallback'),
            "'.lower() in ['true','1']"
        ])),
    )

    return LaunchDescription(args + [start_launch, static_world_to_odom, static_map_to_odom, static_world_to_map, delayed_nav2_group, nav2_startup_fallback])
