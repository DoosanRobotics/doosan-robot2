# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
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
import yaml  # [MODIFIED] YAML 파일 읽기/쓰기 위해 추가
import subprocess # [MODIFIED] 
from urdf_parser_py.urdf import URDF # [MODIFIED] 

# [modified] 
def adjust_dsr_controller_yaml(yaml_path, active_joints, passive_joints):
    """
    Modify dsr_controller2.yaml file by:
    - Replacing all 'joints' entries with the provided active_joints (excluding fixed joints).
    - Removing 'passive_joints' if it exists.
    - Applying changes only to a specific list of controllers.
    """

    temp_yaml = "/tmp/adjusted_dsr_controller2.yaml"  # Temporary path for the modified YAML

    # Load the original YAML file
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    # Controllers whose 'joints' list will be updated
    controllers = [
        "dsr_controller2",
        "dsr_moveit_controller",
        "dsr_position_controller",
        "dsr_joint_trajectory",
    ]

    # Update each selected controller
    for ctrl in controllers:
        if ctrl in data:
            params = data[ctrl].get("ros__parameters", {})
            params["joints"] = list(active_joints)  # Replace joints with active_joints
            if "passive_joints" in params:
                params.pop("passive_joints", None)  # Remove passive_joints if present
            data[ctrl]["ros__parameters"] = params
            print(f"[DEBUG] Updated joints for controller: {ctrl} -> {active_joints}")

    # Save the modified YAML to a temporary file
    with open(temp_yaml, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

    print(f"[INFO] Adjusted yaml file saved to: {temp_yaml}")

    # Print the content of the modified YAML for debugging
    with open(temp_yaml, 'r') as debug_file:
        print("[DEBUG] Adjusted YAML content:\n" + debug_file.read())

    return temp_yaml

# [modified]
def parse_joints_from_urdf(model, color):
    """
    Parse URDF (generated from xacro) and extract joint lists.
    - Active joints: All non-fixed joints (revolute, prismatic, etc.).
    - Passive joints: All fixed joints.
    """

    # Build the path to the xacro file based on the robot model
    xacro_file = os.path.join(
        get_package_share_directory('dsr_description2'),
        'xacro',
        f"{model}.urdf.xacro"
    )

    # Generate URDF XML string by executing xacro
    urdf_xml = subprocess.check_output(['xacro', xacro_file, f'color:={color}']).decode('utf-8')

    # Parse URDF model from the XML string
    robot_model = URDF.from_xml_string(urdf_xml)

    # Extract active joints (non-fixed)
    active_joints = [joint.name for joint in robot_model.joints if joint.type != "fixed"]

    # Extract passive joints (fixed)
    passive_joints = [joint.name for joint in robot_model.joints if joint.type == "fixed"]

    # Debug print for verification
    print(f"[DEBUG] Active joints (for controllers): {active_joints}")
    print(f"[DEBUG] Passive joints (fixed): {passive_joints}")

    return urdf_xml, active_joints, passive_joints

# [modified]
def generate_robot_description_action(context, *args, **kwargs):
    """
    Generate robot_description and dynamic controller YAML based on the URDF model.
    - Extracts active/passive joints from URDF.
    - Updates controller YAML with the list of active joints.
    """

    # Retrieve 'model' and 'color' arguments from the launch context
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)

    # Parse URDF to extract active and passive joints
    urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color)

    # Adjust the dsr_controller2.yaml file using active joints (exclude fixed joints)
    original_yaml = os.path.join(
        get_package_share_directory("dsr_controller2"),
        "config",
        "dsr_controller2.yaml"
    )
    adjusted_yaml = adjust_dsr_controller_yaml(original_yaml, active_joints, passive_joints)

    # Provide robot_description (URDF) and controller YAML as launch configurations
    return [
        SetLaunchConfiguration('robot_description', urdf_xml),
        SetLaunchConfiguration('controller_yaml', adjusted_yaml)
    ]
#----------------------------------------------------------------------------------------------------------------------------------------
def rviz_node_function(context):
    """런치 시점에서 model 값을 평가하고, 패키지 경로를 찾은 후 launch 파일 실행"""
    model_value = LaunchConfiguration('model').perform(context)

    # 패키지 이름 생성
    model_value_str = f"{model_value}"
    package_name_str = f"dsr_moveit_config_{model_value}"

    # FindPackageShare 평가
    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("패키지 이름:", package_name_str)
    print("패키지 경로:", package_path_str)

    # Moveit2 config 
    moveit_config = (
        MoveItConfigsBuilder(model_value_str, "robot_description", package_name_str)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # [modified]
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
        # [modified], Pass all MoveIt2 parameters as a dictionary to the node
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
        
    ]
    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')

    # [modified] Generate dynamic controller YAML and robot_description
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

    # gui = LaunchConfiguration("gui")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_description2"),
                    "xacro",
                    LaunchConfiguration('model'),
                ]
            ),
            ".urdf.xacro",
        ]
    )

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
    
    # [modified] using temp yaml file
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"robot_description": LaunchConfiguration('robot_description')},
            LaunchConfiguration('controller_yaml')
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
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}], # [modified] using robot_description
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


    # [modified] MoveIt2 configuration and RViz launch
    rviz_node = OpaqueFunction(function=rviz_node_function)


    # [modified] Delay ros2_control_node until set_config_node finishes
    delay_control_node_after_set_config = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_config_node,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] set_config_node finished. Starting ros2_control_node..."),
                control_node
            ],
        )
    )

    # [modified] Delay robot_controller (dsr_controller2) until joint_state_broadcaster is active
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    )

    # [modified] Delay dsr_moveit_controller until robot_controller is active
    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
                # [modified]
                # ExecuteProcess(cmd=["ros2", "control", "list_controllers"], output="screen"),
            ],
        )
    )

    # [modified] Delay RViz and move_group until dsr_moveit_controller is active
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
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),  # [modified] Launch start log
        robot_description_action,        # [modified] Generate robot_description and adjusted controller YAML dynamically
        set_config_node,                 # [modified] Robot parameter configuration node
        run_emulator_node,               # [modified] Emulator node for virtual mode
        robot_state_pub_node,            # [modified] Robot state publisher (publishes /robot_description)
        delay_control_node_after_set_config,  # [modified] Wait for set_config_node before starting ros2_control_node
        joint_state_broadcaster_spawner, # [modified] Broadcast /joint_states
        delay_robot_controller_after_joint_state,  # [modified] Wait for joint_state_broadcaster before dsr_controller2
        delay_dsr_moveit_controller_after_robot_controller,  # [modified] Wait for dsr_controller2 before dsr_moveit_controller
        delay_rviz_after_moveit_controller,        # [modified] Wait for MoveIt2 controllers before RViz and move_group
    ]

    return LaunchDescription(ARGUMENTS + nodes)
