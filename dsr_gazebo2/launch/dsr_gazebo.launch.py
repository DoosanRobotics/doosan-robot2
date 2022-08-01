#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'model',
        default_value='m1013',
        description='Robot Model'
    ),
    DeclareLaunchArgument(
        'color',
        default_value='white',
        description='Robot Color'
    )
]


def generate_launch_description():
    #use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    xacro_path = os.path.join(get_package_share_directory('dsr_description2'), 'xacro')

    return LaunchDescription(ARGUMENTS + [
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])}],
            ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'm1013'],
            output='screen'
            ),
    ])
