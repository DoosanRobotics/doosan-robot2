# 
#  dsr_moveit2
#  Author: Minsoo Song (minsoo.song@doosan.com)
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

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,DeclareLaunchArgument, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction, SetLaunchConfiguration

# Moveit2
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
import subprocess
from urdf_parser_py.urdf import URDF
from dsr_bringup2.utils.controller_config import adjust_dsr_controller_yaml, parse_joints_from_urdf

# Generate robot_description and select controller YAML based on the URDF model.
def generate_robot_description_action(context, *args, **kwargs):

    # Retrieve 'dynamic_yaml'and 'model', 'color' arguments from the launch context
    dynamic_yaml = LaunchConfiguration('dynamic_yaml').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)

    # Parse URDF to extract active and passive joints
    urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color)

    # If `dynamic_yaml` is true, generates a temporary controller.yaml with active joints only.
    if dynamic_yaml:
        original_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            "dsr_controller2.yaml"
        )
        adjusted_yaml = adjust_dsr_controller_yaml(original_yaml, active_joints, passive_joints)
        print(f"[INFO] Using dynamically generated controller.yaml: {adjusted_yaml}")

    # If 'dynamic_yaml' is false, attempts to use a model-specific static controller.yaml (e.g., dsr_controller2_<model>.yaml).
    else:
        static_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            f"dsr_controller2_{model}.yaml"
        )
        if os.path.exists(static_yaml):
            adjusted_yaml = static_yaml
            print(f"[INFO] Using static controller.yaml: {adjusted_yaml}")
        # Falls back to the default dsr_controller2.yaml if a model-specific YAML is not found
        else:
            adjusted_yaml = os.path.join(
                get_package_share_directory("dsr_controller2"),
                "config",
                "dsr_controller2.yaml"
            )
            print(f"[WARN] Model-specific YAML not found. Using default: {adjusted_yaml}")

    return [
        SetLaunchConfiguration('robot_description', urdf_xml),
        SetLaunchConfiguration('controller_yaml', adjusted_yaml)
    ]

def rviz_node_function(context):
    model_value = LaunchConfiguration('model').perform(context)

    model_value_str = f"{model_value}"
    package_name_str = f"dsr_moveit_config_{model_value}"

    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("Package:", package_name_str)
    print("Package Path:", package_path_str)

    moveit_config = (
        MoveItConfigsBuilder(model_value_str, "robot_description", package_name_str)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"],      # List of planning pipelines to load (each loaded from config/<name>_planning.yaml)
                            default_planning_pipeline="ompl", # Name of the default planning pipeline (used if none is explicitly selected)
                            load_all= False                   # If pipelines is None: True loads all from config/default packages; False loads only from config package
                            )
        .to_moveit_configs()
    )
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory(package_name_str), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    
    return [run_move_group_node, 
        Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace=LaunchConfiguration('name'),
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[moveit_config.to_dict()],
    )]

def generate_launch_description():
    ARGUMENTS =[ 
        DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model', default_value = 'a0509',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('gui',   default_value = 'false',     description = 'Start RViz2'    ),
        DeclareLaunchArgument('gz',    default_value = 'false',     description = 'USE GAZEBO SIM'    ),
        DeclareLaunchArgument('rt_host',    default_value = '192.168.137.50',     description = 'ROBOT_RT_IP'    ),
        DeclareLaunchArgument('dynamic_yaml', default_value = 'false', description='Use dynamic generation of controller.yaml (true/false)')
    ]

    # Generate dynamic controller YAML and robot_description
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

    set_config_node = Node(
        package="dsr_bringup2",
        executable="set_config",
        namespace=LaunchConfiguration('name'),
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
            {"rt_host":  LaunchConfiguration('rt_host')      },
            #parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.    
        ],
        output="screen",
    )
    
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
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
            {"rt_host":  LaunchConfiguration('rt_host')      },
            #parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.    
        ],
        output="screen",
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"robot_description": LaunchConfiguration('robot_description')},
            LaunchConfiguration('controller_yaml')     # using static or dynamic yaml file
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        # remappings=[
        #     (
        #         "/joint_states",
        #         "/dsr/joint_states",
        #     ),
        # ],
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}], # using robot_description from generate_robot_description_action function
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "dsr_moveit_controller",
            "-c",
            "controller_manager",
        ],
    )

    rviz_node = OpaqueFunction(function=rviz_node_function)

    # Delay ros2_control_node until set_config_node finishes
    delay_control_node_after_set_config = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_config_node,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] set_config_node finished. Starting ros2_control_node..."),
                control_node
            ],
        )
    )

    # Delay robot_controller (dsr_controller2) until joint_state_broadcaster is active
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    )

    #  Delay dsr_moveit_controller until robot_controller is active
    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    # Delay RViz and move_group until dsr_moveit_controller is active
    delay_rviz_after_moveit_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup + RViz2..."),
                rviz_node
            ],
        )
    )

    nodes = [
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),  # Launch start log
        robot_description_action,                                                  # Generate robot_description and adjusted controller YAML dynamically
        set_config_node,                                                           # robot parameter configuration node
        run_emulator_node,                                                         # Emulator node for virtual mode
        robot_state_pub_node,                                                      # Robot state publisher (publishes /robot_description)
        delay_control_node_after_set_config,                                       # Wait for set_config_node before starting ros2_control_node
        joint_state_broadcaster_spawner,                                           # Broadcast /joint_states
        delay_robot_controller_after_joint_state,                                  # Wait for joint_state_broadcaster before dsr_controller2
        delay_dsr_moveit_controller_after_robot_controller,                        # Wait for dsr_controller2 before dsr_moveit_controller
        delay_rviz_after_moveit_controller,                                        # Wait for MoveIt2 controllers before RViz and move_group
    ]

    return LaunchDescription(ARGUMENTS + nodes)
