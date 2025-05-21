# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler,DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
    ARGUMENTS =[ 
        DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('rt_host',    default_value = '192.168.137.50',     description = 'ROBOT_RT_IP'    ),
        DeclareLaunchArgument('rviz',   default_value = 'false',     description = 'Start RViz2'    ),
        DeclareLaunchArgument('start_emulator',   default_value = 'false',     description = 'Start emulator'    ),
    ]
    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')
    rviz = LaunchConfiguration("rviz")
    start_emulator = LaunchConfiguration("start_emulator")
    
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
        output="screen",
        condition=IfCondition(start_emulator),
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
        'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])           
    }])
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        rviz_node,
        control_node,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
