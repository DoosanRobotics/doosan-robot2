# 
#  camera_calibration.launch.py
#  Modified by: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    descp_path = get_package_share_directory('dsr_visualservoing')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + descp_path + '/description/visual_servoing.sdf'


        }.items(),
    )

    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=[LaunchConfiguration('image_topic')],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )


    bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['rgbd_camera/image', 'rgbd_camera/depth_image'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Open RQt.'),

        DeclareLaunchArgument('image_topic', default_value='/camera',
                              description='Topic to start viewing in RQt.'),
        bridge,
        rqt,

    ])
