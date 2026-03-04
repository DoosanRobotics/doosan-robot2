#
#  dsr_bringup2
#  Mobile Manipulator dedicated launcher (R100 + M1013)
#

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from dsr_bringup2.utils import read_update_rate, show_git_info


def generate_launch_description():
    args = [
        DeclareLaunchArgument('model', default_value='r100_m1013', description='ROBOT_MODEL'),
        DeclareLaunchArgument('name', default_value=LaunchConfiguration('model'), description='NAME_SPACE'),
        DeclareLaunchArgument('host', default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port', default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode', default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('gui', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('remap_tf', default_value='false', description='REMAP TF'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='false', description='Publish joint_states for visualization'),
        DeclareLaunchArgument('use_nav2', default_value='true', description='Start Nav2 navigation stack'),
        DeclareLaunchArgument('nav2_start_delay', default_value='6.0', description='Delay (sec) before starting Nav2 to wait for odom->base_link TF'),
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

    xacro_path = os.path.join(get_package_share_directory('dsr_description2'), 'xacro')
    mode = LaunchConfiguration("mode")
    arm_model = PythonExpression([
        "('", LaunchConfiguration('model'), "').replace('r100_','')"
    ])
    update_rate = int(read_update_rate())
    show_git_info()

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("dsr_description2"),
                "xacro",
                LaunchConfiguration('model'),
            ]),
            ".urdf.xacro",
            " ",
            "color:=", LaunchConfiguration('color'),
            " ",
            "host:=", LaunchConfiguration('host'),
            " ",
            "rt_host:=", LaunchConfiguration('rt_host'),
            " ",
            "port:=", LaunchConfiguration('port'),
            " ",
            "mode:=", LaunchConfiguration('mode'),
            " ",
            "model:=", arm_model,
            " ",
            "update_rate:=", str(update_rate),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    moveit_config_pkg = PythonExpression([
        "'dsr_moveit_config_' + '", LaunchConfiguration('model'), "'"
    ])
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(moveit_config_pkg),
        "config",
        "ros2_controllers.yaml",
    ])
    mobile_base_controllers = PathJoinSubstitution([
        FindPackageShare("dsr_controller2"),
        "config",
        "mobile_base_controller.yaml",
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "rviz", "default.rviz"
    ])

    set_config_node = Node(
        package="dsr_bringup2",
        executable="set_config",
        namespace=LaunchConfiguration('name'),
        name="set_config",
        parameters=[
            {"name": LaunchConfiguration('name')},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('host')},
            {"port": LaunchConfiguration('port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": arm_model},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
            {"update_rate": update_rate},
        ],
        output="screen",
    )

    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        name="run_emulator",
        parameters=[
            {"name": LaunchConfiguration('name')},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('host')},
            {"port": LaunchConfiguration('port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": arm_model},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description, robot_controllers, mobile_base_controllers],
        remappings=[("~/robot_description", "robot_description")],
        output="both",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        remappings=[
            ("goal_pose", "/goal_pose"),
            ("move_base_simple/goal", "/goal_pose"),
            ("initialpose", "/initialpose"),
        ],
    )

    original_tf_nodes = GroupAction(
        actions=[
            robot_state_pub_node,
            joint_state_pub_node,
            rviz_node
        ],
        condition=UnlessCondition(LaunchConfiguration('remap_tf'))
    )

    remapped_tf_nodes = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            robot_state_pub_node,
            joint_state_pub_node,
            rviz_node
        ],
        condition=IfCondition(LaunchConfiguration('remap_tf'))
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager", "--controller-manager-timeout", "120"],
    )

    moveit_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_moveit_controller", "-c", "controller_manager", "--activate", "--controller-manager-timeout", "120"],
    )

    mobile_base_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "controller_manager", "--controller-manager-timeout", "120"],
    )

    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_config_node,
            on_exit=[control_node],
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_config_node,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_mobile_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mobile_base_controller_spawner],
        )
    )

    delay_moveit_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[moveit_controller_spawner],
        )
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
            'autostart': 'true',
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

    return LaunchDescription(args + [
        set_config_node,
        run_emulator_node,
        original_tf_nodes,
        remapped_tf_nodes,
        delay_control_node,
        delay_joint_state_broadcaster,
        delay_robot_controller,
        delay_moveit_controller,
        delay_mobile_base_controller,
        static_world_to_odom,
        static_map_to_odom,
        static_world_to_map,
        delayed_nav2_group,
        nav2_startup_fallback,
    ])
