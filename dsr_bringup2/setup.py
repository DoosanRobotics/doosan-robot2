from setuptools import find_packages, setup
from glob import glob

package_name = 'dsr_bringup2'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minsoo Song',
    maintainer_email='minsoo.song@doosan.com',
    description='dsr_bringup2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_connection = dsr_bringup2.moveit_connection:main',
            'gazebo_connection = dsr_bringup2.gazebo_connection:main',
            'run_emulator = dsr_bringup2.run_emulator:main',
            'mujoco_bridge = dsr_bringup2.dsr_mujoco_bridge:main',
        ],
    },
)
