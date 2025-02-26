# 
#  dsr_gazebo_visual_servoing.launch
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


ARGUMENTS =[ 
    DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
    DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
    DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
    DeclareLaunchArgument('gui',   default_value = 'false',     description = 'Start RViz2'    ),
    DeclareLaunchArgument('use_gazebo',   default_value = 'true',     description = 'Start Gazebo'    ),

    DeclareLaunchArgument('x',   default_value = '0',     description = 'Location x on Gazebo '    ),
    DeclareLaunchArgument('y',   default_value = '0',     description = 'Location y on Gazebo'    ),
    DeclareLaunchArgument('z',   default_value = '0',     description = 'Location z on Gazebo'    ),
    DeclareLaunchArgument('R',   default_value = '0',     description = 'Location Roll on Gazebo'    ),
    DeclareLaunchArgument('P',   default_value = '0',     description = 'Location Pitch on Gazebo'    ),
    DeclareLaunchArgument('Y',   default_value = '0',     description = 'Location Yaw on Gazebo'    ),
]

def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "gz"]),
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            LaunchConfiguration('model'),
            "-allow_renaming",
            "true",
            "-x",
            LaunchConfiguration('x'),
            "-y",
            LaunchConfiguration('y'),
            "-z",
            LaunchConfiguration('z'),
            "-R",
            LaunchConfiguration('R'),
            "-P",
            LaunchConfiguration('P'),
            "-Y",
            LaunchConfiguration('Y'),
        ],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_description2"),
                    "xacro",
                    LaunchConfiguration('model'),
                ]
            ),
            ".urdf.xacro",
            " ",
            "use_gazebo:=",
            LaunchConfiguration('use_gazebo'),
            " ",
            "color:=",
            LaunchConfiguration('color'),
            " ",
            "namespace:=",
            PathJoinSubstitution([LaunchConfiguration('name'), "gz"])
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "gz"]),
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "gz"]),
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )
    
    dsr_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "gz"]),
        arguments=["dsr_position_controller", "--controller-manager", "controller_manager"],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # launch issue position controller
    dsr_position_controller_spawner_action = TimerAction(
        period=2.0,
        actions=[dsr_position_controller_spawner]
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_dsr_position_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dsr_position_controller_spawner_action],
        )
    )

    nodes = [
        # gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        delay_dsr_position_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(ARGUMENTS + nodes)

