# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ARGUMENTS =[ 
        DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',  default_value = '192.168.137.100', description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',  default_value = 'real',   description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('gz',    default_value = 'false',     description = 'USE GAZEBO SIM'    ),
        DeclareLaunchArgument('rt_host',    default_value = '192.168.137.50',     description = 'ROBOT_RT_IP'    ),
    ]
    package_name = PythonExpression(["'", LaunchConfiguration('model'), "_moveit_config'"])

    included_launch_file_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'launch',
        'start.launch.py'
    ])

    # IncludeLaunchDescription 액션을 사용하여 두 번째 Launch 파일을 포함합니다.
    # launch_arguments를 사용하여 namespace를 설정합니다.
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file_path),
        launch_arguments={'mode': LaunchConfiguration('mode'), 
                        'name' : LaunchConfiguration('name'),
                        'color' : LaunchConfiguration('color'),
                        'model' :LaunchConfiguration('model'),
                        'host' :LaunchConfiguration('host'),
                        'port' :LaunchConfiguration('port'),
                        "rt_host":  LaunchConfiguration('rt_host'),
                        }.items(),
    )

    return LaunchDescription(ARGUMENTS + [included_launch])
