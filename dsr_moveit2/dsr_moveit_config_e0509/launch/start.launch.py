# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os
import yaml

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

from moveit_configs_utils import MoveItConfigsBuilder
from dsr_bringup2.utils import read_update_rate


def rviz_node_function(context):
    model_value = LaunchConfiguration('model').perform(context)

    model_value_str = f"{model_value}"
    package_name_str = f"dsr_moveit_config_{model_value}"

    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("Package name:", package_name_str)
    print("Package path:", package_path_str)

    moveit_config = (
        MoveItConfigsBuilder(model_value_str, "robot_description", package_name_str)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
            load_all=False
        )
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_base = os.path.join(
        get_package_share_directory(package_name_str), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return [run_move_group_node, rviz_node]


def generate_launch_description():

    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='m0617', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
    ]

    xacro_path = os.path.join(
        get_package_share_directory('dsr_description2'), 'xacro'
    )

    update_rate = str(read_update_rate())

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_description2"),
                    "xacro",
                    LaunchConfiguration('model'),
                ]
            ),
            ".urdf.xacro",
            " name:=", LaunchConfiguration('name'),
            " host:=", LaunchConfiguration('host'),
            " rt_host:=", LaunchConfiguration('rt_host'),
            " port:=", LaunchConfiguration('port'),
            " mode:=", LaunchConfiguration('mode'),
            " model:=", LaunchConfiguration('model'),
            " update_rate:=", update_rate,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = [
        PathJoinSubstitution([
            FindPackageShare("dsr_controller2"),
            "config",
            "dsr_controller2.yaml",
        ])
    ]

    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"name": LaunchConfiguration('name')},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration('host')},
            {"port": LaunchConfiguration('port')},
            {"mode": LaunchConfiguration('mode')},
            {"model": LaunchConfiguration('model')},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
        ],
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
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': Command([
                'xacro', ' ',
                xacro_path, '/',
                LaunchConfiguration('model'),
                '.urdf.xacro color:=',
                LaunchConfiguration('color')
            ])
        }]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_moveit_controller",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
    )

    rviz_node = OpaqueFunction(function=rviz_node_function)

    delay_jsb_after_control_node = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[dsr_moveit_controller_spawner],
        )
    )

    delay_rviz_after_dsr_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        control_node,
        delay_jsb_after_control_node,
        delay_robot_controller_after_joint_state,
        delay_dsr_moveit_controller_after_robot_controller,
        delay_rviz_after_dsr_moveit_controller,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
