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
    maintainer='gossi',
    maintainer_email='mincheol710313@gmail.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dance = dsr_example.demo.dance_m1013:main',
                'single_robot_simple = dsr_example.simple.single_robot_simple:main',
                'slope_demo = dsr_example.demo.slope_demo:main',
                'test_servoj = dsr_example.simple.test_servoj:main',
                'test_servoj_rt = dsr_example.simple.test_servoj_rt:main',
                'test_movel = dsr_example.simple.test_movel:main',
                'test_movel_amovel = dsr_example.simple.test_movel_amovel:main',
                'test_servol = dsr_example.simple.test_servol:main',
                'test_servol_rt = dsr_example.simple.test_servol_rt:main',
                'test_speedj = dsr_example.simple.test_speedj:main',
                'test_speedj_rt = dsr_example.simple.test_speedj_rt:main',
                'test_speedl = dsr_example.simple.test_speedl:main',
                'test_speedl_rt = dsr_example.simple.test_speedl_rt:main',
                'test_torque_rt = dsr_example.simple.test_torque_rt:main',
                'test_rt_service = dsr_example.simple.test_rt_service:main',
        ],
    },
)
