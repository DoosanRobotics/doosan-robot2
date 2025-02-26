import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def include_launch_description(context):
    """런치 시점에서 model 값을 평가하고, 패키지 경로를 찾은 후 launch 파일 실행"""
    model_value = LaunchConfiguration('model').perform(context)

    # 패키지 이름 생성
    package_name_str = f"dsr_moveit_config_{model_value}"

    # FindPackageShare 평가
    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("패키지 이름:", package_name_str)
    print("패키지 경로:", package_path_str)

    # launch 파일 경로
    included_launch_file_path = os.path.join(package_path_str, 'launch', 'start.launch.py')

    # IncludeLaunchDescription 반환
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
    ]

    # OpaqueFunction을 사용하여 런치 시점에서 동적으로 경로를 계산하고 launch 포함
    included_launch = OpaqueFunction(function=include_launch_description)

    return LaunchDescription(ARGUMENTS + [included_launch])
