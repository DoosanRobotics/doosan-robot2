# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def include_launch_description(context):
    model_value = LaunchConfiguration('model').perform(context)

    # Make pacakage name
    package_name_str = f"dsr_moveit_config_{model_value}"

    # Evaluate FindPackageShare
    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("Package:", package_name_str)
    print("Package Path:", package_path_str)

    # launch file path
    included_launch_file_path = os.path.join(package_path_str, 'launch', 'start.launch.py')

    # Return IncludeLaunchDescription 
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file_path),
        launch_arguments={
            'mode': LaunchConfiguration('mode'), 
            'name': LaunchConfiguration('name'),
            'color': LaunchConfiguration('color'),
            'model': LaunchConfiguration('model'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'rt_host': LaunchConfiguration('rt_host'),
            'gripper': LaunchConfiguration('gripper'), # [modified]
        }.items(),
    )]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='m1013', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotique)'), # [modified]
    ]

    # Use OpaqueFunction to dynamically compute the path at launch time and include launch
    included_launch = OpaqueFunction(function=include_launch_description)

    return LaunchDescription(ARGUMENTS + [included_launch])
