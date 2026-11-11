# 
#  dsr_mujoco
#  Author: Theo Choi (theo.choi@doosan.com)
#  
#  Copyright (c) 2025 Doosan Robotics
#  Use of this source code is governed by the Apache License 2.0, see LICENSE
# 

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('headless', default_value='false', description='Run MuJoCo without the viewer window'),
    ]

def generate_launch_description():

    mujoco_model_path = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "mujoco_models",
        LaunchConfiguration('model'),
        "scene.xml"]
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
            "use_mujoco:=true",
            " ",
            "color:=",
            LaunchConfiguration('color'),
            " ",
            "namespace:=",
            PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
            " ",
            "mujoco_model:=",
            mujoco_model_path,
            " ",
            "headless:=",
            LaunchConfiguration('headless'),
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controller_param_file= PathJoinSubstitution(
        [FindPackageShare("dsr_mujoco"), "config",
        "dsr_mujoco_controller.yaml"]
    )
        
    # Publishes robot_description for the MuJoCo controller manager; TF stays under the mj namespace
    mj_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        output='log',
        parameters=[robot_description, {"use_sim_time": True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # Mujoco node
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        output='screen',
        parameters=[
            controller_param_file,
            {"use_sim_time": True},
        ],
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        arguments=[
            "joint_state_broadcaster", 
            "-c", "controller_manager",  
        ],
        output="screen",
    )
    
    # Position controller
    dsr_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        arguments=[
            "dsr_position_controller", 
            "-c", "controller_manager", 
        ],
        output="screen",
    )
    
    # Delay mujoco's robot joint broadcaster after mujoco node start
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    # Delay mujoco's robot controller after mujoco node start
    delay_dsr_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[dsr_position_controller_spawner]
                )
            ]
        )
    )
    
    return LaunchDescription(ARGUMENTS + [
        mj_robot_state_publisher,
        node_mujoco,
        delay_joint_state_broadcaster,
        delay_dsr_position_controller,
    ])


