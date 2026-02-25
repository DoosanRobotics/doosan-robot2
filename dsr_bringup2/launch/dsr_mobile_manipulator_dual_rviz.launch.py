#
#  dsr_bringup2 - Dual Arm Launch (M1013 Dual Arm)
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  Modified for dual arm configuration
#
#  Copyright (c) 2025 Doosan Robotics
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription, ExecuteProcess
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
    ARGUMENTS = [
        DeclareLaunchArgument('model',      default_value='r100_m1013_dual', description='ROBOT_MODEL'),
        DeclareLaunchArgument('name',       default_value=LaunchConfiguration('model'), description='NAME_SPACE'),
        DeclareLaunchArgument('host',       default_value='127.0.0.1',      description='ROBOT_IP (applied to both arms)'),
        DeclareLaunchArgument('left_host',  default_value=LaunchConfiguration('host'),  description='LEFT_ROBOT_IP'),
        DeclareLaunchArgument('left_port',  default_value='12345',          description='LEFT_ROBOT_PORT'),
        DeclareLaunchArgument('right_host', default_value=LaunchConfiguration('host'), description='RIGHT_ROBOT_IP'),
        DeclareLaunchArgument('right_port', default_value='12348',          description='RIGHT_ROBOT_PORT'),
        DeclareLaunchArgument('mode',       default_value='virtual',        description='OPERATION MODE'),
        DeclareLaunchArgument('color',      default_value='white',          description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',        default_value='false',          description='Start RViz2'),
        DeclareLaunchArgument('gz',         default_value='false',          description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host',    default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('remap_tf',   default_value='false',          description='REMAP TF'),
        DeclareLaunchArgument('arm_spacing', default_value='0.6',           description='Distance between arms'),
        DeclareLaunchArgument('left_init_on_start', default_value='true', description='Move left arm J1 to 180deg once at startup (dual only)'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='false', description='Publish joint_states for visualization'),
        DeclareLaunchArgument('use_nav2', default_value='true', description='Start Nav2 navigation stack'),
        DeclareLaunchArgument('nav2_start_delay', default_value='3.0', description='Delay (sec) before starting Nav2 to wait for odom->base_link TF'),
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
        "('", LaunchConfiguration('model'), "').replace('r100_','').replace('_dual','')"
    ])
    left_robot_name = PythonExpression([
        "'left_' + ('", LaunchConfiguration('model'), "').replace('r100_','').replace('_dual','')"
    ])
    right_robot_name = PythonExpression([
        "'right_' + ('", LaunchConfiguration('model'), "').replace('r100_','').replace('_dual','')"
    ])
    xacro_model = PythonExpression([
        "('", LaunchConfiguration('model'), "').replace('r100_','')"
    ])
    moveit_config_pkg = PythonExpression([
        "'dsr_moveit_config_' + '", LaunchConfiguration('model'), "'"
    ])
    update_rate = int(read_update_rate())
    show_git_info()

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("dsr_description2"),
                "xacro",
                xacro_model,
            ]),
            ".urdf.xacro",
            " ",
            "color:=", LaunchConfiguration('color'),
            " ",
            "arm_spacing:=", LaunchConfiguration('arm_spacing'),
            " ",
            "enable_ros2_control:=", "true",
            " ",
            "host:=", LaunchConfiguration('left_host'),
            " ",
            "port:=", LaunchConfiguration('left_port'),
            " ",
            "rt_host:=", LaunchConfiguration('rt_host'),
            " ",
            "mode:=", LaunchConfiguration('mode'),
            " ",
            "model:=", arm_model,
            " ",
            "update_rate:=", str(update_rate),
            " ",
            "arm2_host:=", LaunchConfiguration('right_host'),
            " ",
            "arm2_port:=", LaunchConfiguration('right_port'),
            " ",
            "arm2_rt_host:=", LaunchConfiguration('rt_host'),
            " ",
            "arm2_mode:=", LaunchConfiguration('mode'),
            " ",
            "arm2_model:=", arm_model,
            " ",
            "arm2_update_rate:=", str(update_rate),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

    # Config nodes for both arms
    left_set_config_node = Node(
        package="dsr_bringup2",
        executable="set_config",
        namespace=LaunchConfiguration('name'),
        name="left_set_config",
        parameters=[
            {"name": left_robot_name},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('left_host')},
            {"port": LaunchConfiguration('left_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": arm_model},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
            {"update_rate": update_rate},
        ],
        output="screen",
    )

    right_set_config_node = Node(
        package="dsr_bringup2",
        executable="set_config",
        namespace=LaunchConfiguration('name'),
        name="right_set_config",
        parameters=[
            {"name": right_robot_name},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('right_host')},
            {"port": LaunchConfiguration('right_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": arm_model},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
            {"update_rate": update_rate},
        ],
        output="screen",
    )

    # Emulator nodes for both arms
    left_run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        name="left_run_emulator",
        parameters=[
            {"name": left_robot_name},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('left_host')},
            {"port": LaunchConfiguration('left_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": arm_model},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output="screen",
    )

    right_run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        name="right_run_emulator",
        parameters=[
            {"name": right_robot_name},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('right_host')},
            {"port": LaunchConfiguration('right_port')},
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
            'robot_description': ParameterValue(
                Command([
                    'xacro', ' ',
                    xacro_path, '/',
                    xacro_model,
                    '.urdf.xacro',
                    ' color:=', LaunchConfiguration('color'),
                    ' arm_spacing:=', LaunchConfiguration('arm_spacing'),
                    ' enable_ros2_control:=', 'true',
                    ' host:=', LaunchConfiguration('left_host'),
                    ' port:=', LaunchConfiguration('left_port'),
                    ' rt_host:=', LaunchConfiguration('rt_host'),
                    ' mode:=', LaunchConfiguration('mode'),
                    ' model:=', arm_model,
                    ' update_rate:=', str(update_rate),
                    ' arm2_host:=', LaunchConfiguration('right_host'),
                    ' arm2_port:=', LaunchConfiguration('right_port'),
                    ' arm2_rt_host:=', LaunchConfiguration('rt_host'),
                    ' arm2_mode:=', LaunchConfiguration('mode'),
                    ' arm2_model:=', arm_model,
                    ' arm2_update_rate:=', str(update_rate),
                ]),
                value_type=str
            )
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
        ],
        condition=UnlessCondition(LaunchConfiguration('remap_tf'))
    )

    remapped_tf_nodes = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            robot_state_pub_node,
            joint_state_pub_node,
        ],
        condition=IfCondition(LaunchConfiguration('remap_tf'))
    )

    original_rviz_node = GroupAction(
        actions=[rviz_node],
        condition=UnlessCondition(LaunchConfiguration('remap_tf'))
    )

    remapped_rviz_node = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            rviz_node,
        ],
        condition=IfCondition(LaunchConfiguration('remap_tf'))
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    # Dual-arm service controller
    dual_robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # Dual-arm MoveIt trajectory controller (FollowJointTrajectory action server)
    dual_moveit_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dual_dsr_moveit_controller", "-c", "controller_manager"],
    )

    mobile_base_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "controller_manager"],
    )

    # Delay control_node after both config nodes
    delay_right_set_config = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_set_config_node,
            on_exit=[right_set_config_node],
        )
    )

    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_set_config_node,
            on_exit=[control_node],
        )
    )

    # Delay joint_state_broadcaster after control_node
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_set_config_node,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    # Delay dual arm controller after joint_state_broadcaster
    delay_dual_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dual_robot_controller_spawner],
        )
    )

    delay_mobile_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mobile_base_controller_spawner],
        )
    )

    # Delay MoveIt trajectory controller after dsr_controller2
    delay_dual_moveit_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dual_robot_controller_spawner,
            on_exit=[dual_moveit_controller_spawner],
        )
    )

    left_init_motion_once = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            PythonExpression(["'/' + '", LaunchConfiguration('name'), "' + '/left/motion/move_joint'"]),
            'dsr_msgs2/srv/MoveJoint',
            '{pos: [180.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 60.0, time: 2.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 1}',
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('left_init_on_start')),
    )

    delay_left_init_motion_after_dual_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dual_robot_controller_spawner,
            on_exit=[
                TimerAction(period=1.0, actions=[left_init_motion_once]),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('left_init_on_start')),
    )

    delay_rviz_after_left_init_motion = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_init_motion_once,
            on_exit=[
                TimerAction(period=0.5, actions=[original_rviz_node, remapped_rviz_node]),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('left_init_on_start')),
    )

    delay_rviz_without_left_init_motion = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dual_robot_controller_spawner,
            on_exit=[
                TimerAction(period=0.5, actions=[original_rviz_node, remapped_rviz_node]),
            ],
        ),
        condition=UnlessCondition(LaunchConfiguration('left_init_on_start')),
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

    delay_nav2_after_mobile_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mobile_base_controller_spawner,
            on_exit=[
                TimerAction(
                    period=LaunchConfiguration('nav2_start_delay'),
                    actions=[
                        nav2_group,
                        nav2_startup_fallback,
                    ],
                )
            ],
        )
    )

    nodes = [
        left_set_config_node,
        delay_right_set_config,
        left_run_emulator_node,
        right_run_emulator_node,
        original_tf_nodes,
        remapped_tf_nodes,
        delay_control_node,
        delay_joint_state_broadcaster,
        delay_dual_controller,
        delay_mobile_base_controller,
        delay_dual_moveit_controller,
        delay_left_init_motion_after_dual_controller,
        delay_rviz_after_left_init_motion,
        delay_rviz_without_left_init_motion,
        delay_nav2_after_mobile_base_controller,
        static_world_to_odom,
        static_map_to_odom,
        static_world_to_map,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
