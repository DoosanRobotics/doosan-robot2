from setuptools import find_packages, setup

package_name = 'dsr_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minsoo.song',
    maintainer_email='minsoo.song@doosan.com',
    description='This package is example code for Doosan Robot ROS2 packages.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dance = dsr_example.demo.dance_m1013:main',
                'single_robot_simple = dsr_example.simple.single_robot_simple:main',
                'slope_demo = dsr_example.demo.slope_demo:main',
                'test = dsr_example.simple.test:main',
                'test2 = dsr_example.simple.test2:main',
                'test_action_jog_h2r = dsr_example.simple.test_action_jog_h2r:main',
                'test_action_movej_h2r = dsr_example.simple.test_action_movej_h2r:main',
                'test_action_movel_h2r = dsr_example.simple.test_action_movel_h2r:main',
        ],
    },
)
