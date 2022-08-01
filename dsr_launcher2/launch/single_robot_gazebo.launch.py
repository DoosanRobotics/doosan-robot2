import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition

import xacro

args =[ 
    DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'     ),
    DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
    DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
    DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
    DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
    DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
]
context = LaunchContext()

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')
    drcf_path = os.path.join( get_package_share_directory('common2'), 'bin/DRCF')
    # RViz2
    rviz_config_file = get_package_share_directory('dsr_description2') + "/rviz/default.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])

    #change_permission = ExecuteProcess(
    #    cmd=['chmod', '-R', '775', drcf_path],
    #    output='screen'
    #)

    # Run DRCF Emulator
    DRCF_node = ExecuteProcess(
        cmd=['sh', [drcf_path, '/run_drcf.sh'], LaunchConfiguration('port'), LaunchConfiguration('model')],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"]))
    )

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_0'])

    # robot_state_publisher
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 #output='both',
                                 output='screen',
                                 parameters=[{
                                    'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])           
                                 }])


    # dsr_control2 
    dsr_control2 = Node(package='dsr_control2', 
                        executable='dsr_control_node2', 
                        name='dsr_control_node2', 
                        output='screen',
                        parameters=[
		                    {"name":    LaunchConfiguration('name')  }, 
		                    {"rate":    100         },
                            {"standby": 5000        },
		                    {"command": True        },
		                    {"host":    LaunchConfiguration('host')  },
		                    {"port":    LaunchConfiguration('port')  },
		                    {"mode":    LaunchConfiguration('mode')  },
		                    {"model":   LaunchConfiguration('model') },
                            {"gripper": "none"      },
	                        {"mobile":  "none"      },
                            #parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.    
                        ]
                    )

    # gazebo2
    gazebo = ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                output='screen'
                )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'm1013'],
                        output='screen'
                    )


    return LaunchDescription(args + [
        #change_permission,
        DRCF_node,
        static_tf,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        dsr_control2,
    ])
