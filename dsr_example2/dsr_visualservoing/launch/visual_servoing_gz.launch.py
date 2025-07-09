# 
#  visual_servoing_gz.launch.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
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
