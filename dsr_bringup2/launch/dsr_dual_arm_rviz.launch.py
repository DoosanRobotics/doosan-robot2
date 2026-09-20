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
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction, GroupAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from dsr_bringup2.utils import read_update_rate, show_git_info

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',       default_value='dsr01',          description='NAME_SPACE'),
        DeclareLaunchArgument('left_host',  default_value='127.0.0.1',      description='LEFT_ROBOT_IP'),
        DeclareLaunchArgument('left_port',  default_value='12345',          description='LEFT_ROBOT_PORT'),
        DeclareLaunchArgument('right_host', default_value='127.0.0.1',      description='RIGHT_ROBOT_IP'),
        DeclareLaunchArgument('right_port', default_value='12348',          description='RIGHT_ROBOT_PORT'),
        DeclareLaunchArgument('mode',       default_value='virtual',        description='OPERATION MODE'),
        DeclareLaunchArgument('model',      default_value='m1013_dual',     description='ROBOT_MODEL'),
        DeclareLaunchArgument('color',      default_value='white',          description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',        default_value='true',           description='Start RViz2'),
        DeclareLaunchArgument('gz',         default_value='false',          description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host',    default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('remap_tf',   default_value='false',          description='REMAP TF'),
        DeclareLaunchArgument('arm_spacing', default_value='0.6',           description='Distance between arms'),
    ]

    xacro_path = os.path.join(get_package_share_directory('dsr_description2'), 'xacro')
    mode = LaunchConfiguration("mode")
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
                LaunchConfiguration('model'),
            ]),
            ".urdf.xacro",
            " ",
            "color:=", LaunchConfiguration('color'),
            " ",
            "arm_spacing:=", LaunchConfiguration('arm_spacing'),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("dsr_controller2"),
        "config",
        "dsr_controller2.yaml",
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
            {"name": "left_m1013"},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('left_host')},
            {"port": LaunchConfiguration('left_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": "m1013"},
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
            {"name": "right_m1013"},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('right_host')},
            {"port": LaunchConfiguration('right_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": "m1013"},
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
            {"name": "left_m1013"},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('left_host')},
            {"port": LaunchConfiguration('left_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": "m1013"},
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
            {"name": "right_m1013"},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('right_host')},
            {"port": LaunchConfiguration('right_port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": "m1013"},
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
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': Command([
                'xacro', ' ',
                xacro_path, '/',
                LaunchConfiguration('model'),
                '.urdf.xacro',
                ' mode:=', LaunchConfiguration('mode'),
                ' color:=', LaunchConfiguration('color'),
                ' arm_spacing:=', LaunchConfiguration('arm_spacing')
            ])
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    original_tf_nodes = GroupAction(
        actions=[
            robot_state_pub_node,
            rviz_node
        ],
        condition=UnlessCondition(LaunchConfiguration('remap_tf'))
    )

    remapped_tf_nodes = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            robot_state_pub_node,
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

    # Unified dual arm controller
    dual_robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_dual_controller2", "-c", "controller_manager"],
    )

    # Delay control_node after both config nodes
    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_set_config_node,
            on_exit=[control_node],
        )
    )

    # Delay joint_state_broadcaster after control_node
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_set_config_node,
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

    nodes = [
        left_set_config_node,
        right_set_config_node,
        left_run_emulator_node,
        right_run_emulator_node,
        original_tf_nodes,
        remapped_tf_nodes,
        delay_control_node,
        delay_joint_state_broadcaster,
        delay_dual_controller,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
