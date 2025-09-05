# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,DeclareLaunchArgument, GroupAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch.launch_context import LaunchContext

def print_launch_configuration_value(context, *args, **kwargs):
    # LaunchConfiguration 값을 평가합니다.
    gz_value = LaunchConfiguration('gz').perform(context)
    # 평가된 값을 콘솔에 출력합니다.
    print(f'LaunchConfiguration gz: {gz_value}')
    return gz_value

def generate_launch_description():
    ARGUMENTS =[ 
        DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('gui',   default_value = 'false',     description = 'Start RViz2'    ),
        DeclareLaunchArgument('gz',    default_value = 'true',     description = 'USE GAZEBO SIM'    ),
        DeclareLaunchArgument('x',   default_value = '0',     description = 'Location x on Gazebo '    ),
        DeclareLaunchArgument('y',   default_value = '0',     description = 'Location y on Gazebo'    ),
        DeclareLaunchArgument('z',   default_value = '0',     description = 'Location z on Gazebo'    ),
        DeclareLaunchArgument('R',   default_value = '0',     description = 'Location Roll on Gazebo'    ),
        DeclareLaunchArgument('P',   default_value = '0',     description = 'Location Pitch on Gazebo'    ),
        DeclareLaunchArgument('Y',   default_value = '0',     description = 'Location Yaw on Gazebo'    ),
        DeclareLaunchArgument('rt_host',    default_value = '192.168.137.50',     description = 'ROBOT_RT_IP'    ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('remap_tf',   default_value = 'false',     description = 'REMAP TF'    ),
    ]
    set_use_sim_time = SetLaunchConfiguration(name='use_sim_time', value='true')
    
    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    mode = LaunchConfiguration("mode")
    
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
            " name:=", LaunchConfiguration('name'),
            " host:=", LaunchConfiguration('host'),
            " rt_host:=", LaunchConfiguration('rt_host'),
            " port:=", LaunchConfiguration('port'),
            " mode:=", LaunchConfiguration('mode'),
            " model:=", LaunchConfiguration('model'),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

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
    
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"name":    LaunchConfiguration('name')  }, 
            {"rate":    100         },
            {"standby": 5000        },
            {"command": True        },
            {"host":    LaunchConfiguration('host')  },
            {"port":    LaunchConfiguration('port')  },
            {"mode":    LaunchConfiguration('mode')  },
            {"model":   LaunchConfiguration('model') },
            {"gripper": "none"      },
            {"mobile":  "none"      },
            {"rt_host":  LaunchConfiguration('rt_host')      },
            #parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.    
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output="screen",
    )

    gazebo_connection_node = Node(
        package="dsr_bringup2",
        executable="gazebo_connection",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"model":   LaunchConfiguration('model') },
        ],
        output="log",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description, robot_controllers],
        # output="both",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])           
        }],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )
    
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
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

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_joint_trajectory", "-c", "controller_manager"],
    )



    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    

    # 두 번째 Launch 파일의 경로를 설정합니다.
    included_launch_file_path = os.path.join(
        get_package_share_directory('dsr_gazebo2'),
        'launch',
        'dsr_spawn_on_gazebo.launch.py'
    )
    
    # IncludeLaunchDescription 액션을 사용하여 두 번째 Launch 파일을 포함합니다.
    # launch_arguments를 사용하여 namespace를 설정합니다.
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file_path),
        launch_arguments={'use_gazebo': LaunchConfiguration('gz'), 
                          'name' : LaunchConfiguration('name'),
                          'color' : LaunchConfiguration('color'),
                          'x' :LaunchConfiguration('x'),
                          'y' :LaunchConfiguration('y'),
                          'z' :LaunchConfiguration('z'),
                          'R' :LaunchConfiguration('R'),
                          'P' :LaunchConfiguration('P'),
                          'Y' :LaunchConfiguration('Y'),
                          'use_sim_time' : LaunchConfiguration('use_sim_time'),
                          'remap_tf' : LaunchConfiguration('remap_tf'),
                          }.items(),
    )
    

    included_launch_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[included_launch],
        )
    )
    

    nodes = [
        set_use_sim_time,
        run_emulator_node,
        gazebo_connection_node,
        original_tf_nodes,
        remapped_tf_nodes,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        included_launch_after_robot_controller_spawner,
        control_node,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
