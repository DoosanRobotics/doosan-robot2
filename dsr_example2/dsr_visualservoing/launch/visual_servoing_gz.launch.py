# 
#  visual_servoing_gz.launch.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dsr_visualservoing',
            executable='detect_marker_gz',
            name='detect_marker_gz',
            output='screen'
        ),
        
        Node(
            package='dsr_visualservoing',
            executable='send_pose_servol_gz',
            name='send_pose_servol_gz',
            output='screen'
        )
    ])
