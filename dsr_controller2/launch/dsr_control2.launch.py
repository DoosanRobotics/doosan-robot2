import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.launch_context import LaunchContext

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import pathlib

parameters_file_name = 'default.yaml'

def generate_launch_description():

    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/config/' + parameters_file_name

    #parameters_file_path = Path(get_package_share_directory('dsr_control_node2'), 'config', 'default.yaml')

    print(parameters_file_path)

    return LaunchDescription([

        Node(
            package='dsr_control2', 
            node_executable='dsr_control_node2', 
            output='screen',

            # arguments=[
            #     LaunchConfiguration('test_arg'),
            #     LaunchConfiguration('test_arg2')           
            # ],

            parameters=[
		        {"name":    "dsr01"     },  #??? 추후 확인 필요! 
		        {"rate":    100         },
                {"standby": 5000        },
		        {"command": True        },  #??? 추후 확인 필요!
		        {"host":    "127.0.0.1" },
		        {"port":    12345       },
		        {"mode":    "virtual"   },
		        {"model":   "m1013"     },
                {"gripper": "none"      },
	            {"mobile":  "none"      },
                #parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.    
            ],
        )    
    ])
