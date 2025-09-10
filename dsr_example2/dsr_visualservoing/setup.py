
#
# dsr_example2
# Author: Chemin Ahn (chemx3937@gmail.com)
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

import glob
import os
from setuptools import find_packages, setup

package_name = 'dsr_visualservoing'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*launch.py'))),
        (share_dir + '/config', glob.glob(os.path.join('config', '*yaml'))),
        (share_dir + '/description', glob.glob(os.path.join('description', '*sdf'))),
        (share_dir + '/description/textures', glob.glob(os.path.join('description/textures', '*png'))),

    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chemin Ahn',
    maintainer_email='chemx3937@gmail.com',
    description='Visual Servoing',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [                
                'camera_publisher = dsr_visualservoing.camera_publisher:main',
                'joint90 = dsr_visualservoing.joint90:main',
                'detect_marker_gz = dsr_visualservoing.detect_marker_gz:main',
                'send_pose_servol_gz = dsr_visualservoing.send_pose_servol_gz:main',


        ],
    },
)
