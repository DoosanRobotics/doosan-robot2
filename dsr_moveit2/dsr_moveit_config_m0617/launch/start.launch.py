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
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from dsr_bringup2.controller_config import adjust_dsr_controller_yaml, parse_joints_from_urdf

# Generate robot_description and select controller YAML based on the URDF model.
def generate_robot_description_action(context, *args, **kwargs):
    dynamic_yaml = LaunchConfiguration('dynamic_yaml').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)

    # Parse URDF to extract active and passive joints
    urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color, gripper)
    print(f"[DEBUG] model={model}, gripper={gripper}")
    print(f"[DEBUG] active_joints={active_joints}")
    print(f"[DEBUG] passive_joints={passive_joints}")

    # Decide controller YAML
    if dynamic_yaml:
        original_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            "dsr_controller2.yaml"
        )
        adjusted_yaml = adjust_dsr_controller_yaml(original_yaml, active_joints, passive_joints)
        print(f"[INFO] Using dynamically generated controller.yaml: {adjusted_yaml}")
    else:
        static_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            f"dsr_controller2_{model}.yaml"
        )
        if os.path.exists(static_yaml):
            adjusted_yaml = static_yaml
            print(f"[INFO] Using static controller.yaml: {adjusted_yaml}")
        else:
            adjusted_yaml = os.path.join(
                get_package_share_directory("dsr_controller2"),
                "config",
                "dsr_controller2.yaml"
            )
            print(f"[WARN] Model-specific YAML not found. Using default: {adjusted_yaml}")

    return [
        SetLaunchConfiguration('robot_description', urdf_xml),
        SetLaunchConfiguration('controller_yaml', adjusted_yaml),
    ]

def rviz_and_move_group_fn(context):
    model_value = LaunchConfiguration('model').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    package_name = f"dsr_moveit_config_{model_value}"
    package_path = FindPackageShare(package_name).perform(context)
    print("MoveIt Config Package:", package_name)
    print("Package Path:", package_path)

    # 
    moveit_config = (
        MoveItConfigsBuilder(model_value, "robot_description", package_name)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf.xacro", mappings={'gripper': LaunchConfiguration('gripper')})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"],      # List of planning pipelines to load (each loaded from config/<name>_planning.yaml)
                            default_planning_pipeline="ompl", # Name of the default planning pipeline (used if none is explicitly selected)
                            load_all= False                   # If pipelines is None: True loads all from config/default packages; False loads only from config package
                            )
        .to_moveit_configs()
    )

    common_params = [
        moveit_config.to_dict(),  # robot_description & robot_description_semantic from MoveitConfigbuilder
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=common_params,
    )

    rviz_base = os.path.join(get_package_share_directory(package_name), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=common_params,
    )
    return [run_move_group_node, rviz_node]

# sets up the parameters for the controller manager node, if 'gripper' argument is setted, it additionally loads the 'gripper_controller.yaml' file
def control_node_fn(context):
    params = [{"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}, LaunchConfiguration('controller_yaml')]

    if LaunchConfiguration('gripper').perform(context) == 'robotiq_2f85':
        pkg_share = get_package_share_directory("dsr_controller2")
        gripper_yaml = os.path.join(pkg_share, "config", "gripper_controller.yaml")
        params.append(gripper_yaml)
        print(f"[INFO] Including gripper YAML in controller_manager: {gripper_yaml}")

    node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=params,
        output="both",
    )
    return [node]

def gripper_spawner_fn(context):
    if LaunchConfiguration('gripper').perform(context) != 'robotiq_2f85':
        return []

    return [Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "-c", "controller_manager",
        ],
        output="screen",
    )]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='a0509', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('dynamic_yaml', default_value='false', description='Use dynamic controller.yaml'),
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotiq_2f85)'),
    ]

    # Build robot_description and select controller YAML
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

    # Run set_config
    set_config_node = Node(
        package="dsr_bringup2",
        executable="set_config",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": LaunchConfiguration('name'),
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('host'),
            "port": LaunchConfiguration('port'),
            "mode": LaunchConfiguration('mode'),
            "model": LaunchConfiguration('model'),
            "gripper": LaunchConfiguration('gripper'),
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        output="screen",
    )

    # Run emulator
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": LaunchConfiguration('name'),
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('host'),
            "port": LaunchConfiguration('port'),
            "mode": LaunchConfiguration('mode'),
            "model": LaunchConfiguration('model'),
            "gripper": LaunchConfiguration('gripper'),
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        output="screen",
    )

    # Run robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
        }],
    )

    # Run ros2_control_node(controller_manager)
    control_node = OpaqueFunction(function=control_node_fn)

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    # Spawn dsr_controller2
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # Spawn dsr_moveit_controller
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=["dsr_moveit_controller", "-c", "controller_manager"],
    )

    # MoveGroup + (optional) RViz
    rviz_and_move_group = OpaqueFunction(function=rviz_and_move_group_fn)

    # A) After set_config exits, start controller manager and then (after a short delay) spawn joint_state_broadcaster.
    delay_control_node_after_set_config = RegisterEventHandler(
        OnProcessExit(
            target_action=set_config_node,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] set_config finished. Starting ros2_control_node..."),
                control_node,
                TimerAction(period=1.0, actions=[
                    LogInfo(msg=">> [STEP 1B] Spawning joint_state_broadcaster..."),
                    joint_state_broadcaster_spawner
                ]),
            ],
        )
    )

    # B) Once joint_state_broadcaster is active, spawn dsr_controller2 (arm controller).
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    )

    # C) After dsr_controller2 becomes active, (conditionally) spawn the gripper position controller.
    delay_gripper_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3A] dsr_controller2 active. (cond) starting gripper_position_controller..."),
                OpaqueFunction(function=gripper_spawner_fn),
            ],
        )
    )

    # D) After dsr_controller2 becomes active, spawn dsr_moveit_controller (MoveIt-compatible trajectory controller).
    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    # E) After dsr_moveit_controller is active, start MoveGroup (and RViz if gui=true).
    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        )
    )

    nodes = [
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),
        robot_description_action,
        set_config_node,
        run_emulator_node,
        robot_state_pub_node,
        delay_control_node_after_set_config,
        delay_robot_controller_after_joint_state,
        delay_gripper_after_robot_controller,
        delay_dsr_moveit_controller_after_robot_controller,
        delay_rviz_after_moveit_controller,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
