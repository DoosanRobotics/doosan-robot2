# 
#  dsr_bringup2
#  Author: Theo Choi (theo.choi@doosan.com)
#  
#  Copyright (c) 2025 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from dsr_bringup2.utils import read_update_rate, show_git_info

def generate_launch_description():
    ARGUMENTS = [ 
        DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE'),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value = 'true',      description = 'Start RViz2'),
        DeclareLaunchArgument('mj',    default_value = 'true',     description = 'USE MUJOCO SIM'    ),
        DeclareLaunchArgument('rt_host', default_value = '', description = 'ROBOT_RT_IP'),
        DeclareLaunchArgument('gripper', default_value='none', description='Use gripper'),
        DeclareLaunchArgument('scene_path', default_value='none', description='Relative path to MuJoCo scene XML file (demo/slope_demo_scene.xml)'),
        DeclareLaunchArgument('headless', default_value='false', description='Run MuJoCo without the viewer window'),
        DeclareLaunchArgument('tf_prefix', default_value='', description='PREFIX FOR TF FRAME NAMES (e.g. "dsr01/")'),
    ]

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    # mode = LaunchConfiguration("mode")
    update_rate = str(read_update_rate()) # get update_rate from yaml
    show_git_info() # print git info

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
            " color:=", LaunchConfiguration('color'),
            " name:=", LaunchConfiguration('name'),
            " host:=", LaunchConfiguration('host'),
            " rt_host:=", LaunchConfiguration('rt_host'),
            " port:=", LaunchConfiguration('port'),
            " mode:=", LaunchConfiguration('mode'),
            " model:=", LaunchConfiguration('model'),
            " update_rate:=", update_rate,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dsr_controller2"),
            "config",
            "dsr_controller2.yaml",
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )
    
    # Run emulator node for virtual mode
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"name":    LaunchConfiguration('name')},
            {"rate":    100},
            {"standby": 5000},
            {"command": True},
            {"host":    LaunchConfiguration('host')},
            {"port":    LaunchConfiguration('port')},
            {"mode":    LaunchConfiguration('mode')},
            {"model":   LaunchConfiguration('model')},
            {"gripper": "none"},
            {"mobile":  "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])),
        output="screen",
    )
    
    # Connect dsr joint topics to sim
    node_mujoco_bridge = Node(
        package="dsr_bringup2",
        executable="mujoco_bridge",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"model":   LaunchConfiguration('model') },
        ],
        output="log",
        condition=IfCondition(LaunchConfiguration('mj')),
    )

    # Ros2 Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description, robot_controllers],
        output="screen",
    )
    
    # Robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[
            {'frame_prefix': LaunchConfiguration('tf_prefix')},
            robot_description,
            {"use_sim_time": LaunchConfiguration('mj')},
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": LaunchConfiguration('mj')}],
    )

    # Joint state broadcaster spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )
    # DSR controller2 spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # Path of mujoco launch file
    included_launch_file_path = os.path.join(
        get_package_share_directory('dsr_mujoco'),
        'launch',
        'dsr_mujoco.launch.py'
    )
    
    # Include mujoco launch file
    mujoco_included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file_path),
        launch_arguments={
                          'name' : LaunchConfiguration('name'),
                          'model' : LaunchConfiguration('model'),
                          'color' : LaunchConfiguration('color'),
                          'gripper' : LaunchConfiguration('gripper'),
                          'scene_path' : LaunchConfiguration('scene_path'),
                          'headless' : LaunchConfiguration('headless'),
                          }.items(),
        condition=IfCondition(LaunchConfiguration('mj')),
    )
    
    # Event handlers for proper sequencing
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    mujoco_included_launch_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[mujoco_included_launch],
        )
    )
    
    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        control_node,
        # gazebo_connection_node, # to do : decide to remove or not
        mujoco_included_launch_after_robot_controller_spawner,
        node_mujoco_bridge,
    ]
    
    return LaunchDescription(ARGUMENTS + nodes)