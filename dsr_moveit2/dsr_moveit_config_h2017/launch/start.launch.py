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
import re
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction, EmitEvent
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from dsr_bringup2.controller_config import adjust_dsr_controller_yaml, parse_joints_from_urdf
from dsr_bringup2.utils import read_update_rate

# Generate robot_description and select controller YAML based on the URDF model.
def generate_robot_description_action(context, *args, **kwargs):
    dynamic_yaml = LaunchConfiguration('dynamic_yaml').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)
    name = LaunchConfiguration('name').perform(context)
    host = LaunchConfiguration('host').perform(context)
    rt_host = LaunchConfiguration('rt_host').perform(context)
    rt_thread_policy = LaunchConfiguration('rt_thread_policy').perform(context)
    rt_thread_priority = LaunchConfiguration('rt_thread_priority').perform(context)
    port = LaunchConfiguration('port').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    update_rate = read_update_rate() # get update_rate from yaml

    # Parse URDF to extract active and passive joints
    urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color, name, host, rt_host, port, mode, update_rate, LaunchConfiguration('gripper').perform(context), rt_thread_policy=rt_thread_policy, rt_thread_priority=rt_thread_priority)
    print(f"[DEBUG] model={model}, color={color}, name={name}, host={host}, rt_host={rt_host}, port={port}, mode={mode}, update_rate={update_rate}")
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
    ns_value = LaunchConfiguration('name').perform(context).strip('/')
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
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],      # List of planning pipelines to load (each loaded from config/<name>_planning.yaml)
                            default_planning_pipeline="ompl", # Name of the default planning pipeline (used if none is explicitly selected)
                            load_all= False                   # If pipelines is None: True loads all from config/default packages; False loads only from config package
                            )
        .to_moveit_configs()
    )
    
    move_group_params = [
        moveit_config.to_dict(),  # robot_description & robot_description_semantic from MoveitConfigbuilder
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
    ]
    
    rviz_params = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        moveit_config.robot_description_semantic,
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=move_group_params,
    )

    # RViz is optional; move_group is not. Returning here also skips writing the temporary RViz config.
    if not gui:
        return [run_move_group_node]

    rviz_base = os.path.join(get_package_share_directory(package_name), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    # dynamically modify RViz config to set correct namespace
    with open(rviz_full_config, 'r') as f:
        content = f.read()

    target_ns = f"/{ns_value}" if ns_value else ""

    new_content = re.sub(
        r'Move Group Namespace:.*', 
        f'Move Group Namespace: {target_ns}', 
        content
    )
    # Move Group Namespace 는 scene/trajectory 토픽에는 적용되지 않으므로 같은 네임스페이스로 직접 지정
    new_content = re.sub(
        r'Planning Scene Topic:\s*/?monitored_planning_scene',
        f'Planning Scene Topic: {target_ns}/monitored_planning_scene',
        new_content
    )
    new_content = re.sub(
        r'Trajectory Topic:\s*/?display_planned_path',
        f'Trajectory Topic: {target_ns}/display_planned_path',
        new_content
    )
    # The gripper panel binds to this exact action, so each robot's RViz drives its own gripper.
    new_content = re.sub(
        r'(\n( *)Name: Gripper Control\n)',
        lambda m: f"{m.group(1)}{m.group(2)}Action Name: {target_ns}/gripper_position_controller/gripper_cmd\n",
        new_content
    )

    # MoveIt RViz 플러그인은 기본 planning pipeline 을 "<Move Group Namespace>default_planning_pipeline"
    # (구분자 없이 이어 붙인 이름)의 파라미터로 읽는다. 네임스페이스가 있으면 그 이름으로도 넘겨 주어야
    # 기본 선택이 launch 에서 지정한 pipeline(ompl)이 된다. 안 넘기면 pilz 가 선택되고 planner 가 비어 계획이 실패한다.
    if target_ns:
        rviz_params.append({f"{target_ns}default_planning_pipeline": moveit_config.planning_pipelines["default_planning_pipeline"]})

    tmp_rviz = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.rviz')
    tmp_rviz.write(new_content)
    tmp_rviz.close()
    tmp_rviz_path = tmp_rviz.name

    print(f"[INFO] Generated dynamic RViz config: {tmp_rviz_path} (NS: {target_ns})")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", tmp_rviz_path],
        parameters=rviz_params,
    )

    cleanup_rviz_file = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                LogInfo(msg=f">> Cleaning up temp RViz config: {tmp_rviz_path}"),
                ExecuteProcess(cmd=['rm', tmp_rviz_path], output='screen')
            ]
        )
    )

    def _remove_tmp_rviz(context):
        if os.path.exists(tmp_rviz_path):
            os.remove(tmp_rviz_path)
        return []

    cleanup_rviz_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_remove_tmp_rviz)])
    )
    return [run_move_group_node, rviz_node, cleanup_rviz_file, cleanup_rviz_on_shutdown]

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

    gripper_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "-c", "controller_manager",
            "--param-file",
            os.path.join(get_package_share_directory("dsr_controller2"), "config", "gripper_controller.yaml"),
        ],
        output="screen",
    )

    def _on_gripper_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f">> [GRIPPER FAILED] gripper_position_controller spawner exited with code {event.returncode}"), EmitEvent(event=Shutdown(reason="gripper spawner failed"))]
        return []

    return [gripper_spawner, RegisterEventHandler(OnProcessExit(target_action=gripper_spawner, on_exit=_on_gripper_exit))]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='h2017', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('rt_thread_policy', default_value='', description='RT thread policy: 1 SCHED_FIFO, 2 SCHED_RR, 0 SCHED_OTHER, empty keeps the default'),
        DeclareLaunchArgument('rt_thread_priority', default_value='', description='RT thread priority for rt_thread_policy (1-99 for FIFO/RR, 0 for OTHER)'),
        DeclareLaunchArgument('dynamic_yaml', default_value='false', description='Use dynamic controller.yaml'),
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotiq_2f85)'),
        DeclareLaunchArgument('tf_prefix', default_value='', description='PREFIX FOR TF FRAME NAMES (e.g. "dsr01/")'),
    ]

    # Build robot_description and select controller YAML
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

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
        parameters=[{'frame_prefix': LaunchConfiguration('tf_prefix')}, {
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
        arguments=[
            "joint_state_broadcaster",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
    )

    # Spawn dsr_controller2
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120",
            "--param-file", LaunchConfiguration('controller_yaml'),
        ],
    )

    # Spawn dsr_moveit_controller
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "dsr_moveit_controller",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120",
            "--param-file", LaunchConfiguration('controller_yaml'),
        ],
    )

    # MoveGroup + (optional) RViz
    rviz_and_move_group = OpaqueFunction(function=rviz_and_move_group_fn)

    # A) Once joint_state_broadcaster exits with code 0, spawn dsr_controller2 (arm controller).
    def _step1_on_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f">> [STEP 1 FAILED] joint_state_broadcaster spawner exited with code {event.returncode}"), EmitEvent(event=Shutdown(reason="STEP 1 failed"))]
        return [
            LogInfo(msg=">> [STEP 1 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
            robot_controller_spawner,
        ]

    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=_step1_on_exit,
        )
    )

    # B) After dsr_controller2 exits with code 0, (conditionally) spawn the gripper position controller.
    def _step2_on_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f">> [STEP 2 FAILED] dsr_controller2 spawner exited with code {event.returncode}"), EmitEvent(event=Shutdown(reason="STEP 2 failed"))]
        return [
            LogInfo(msg=">> [STEP 2] dsr_controller2 active. (cond) starting gripper_position_controller..."),
            OpaqueFunction(function=gripper_spawner_fn),
        ]

    delay_gripper_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=_step2_on_exit,
        )
    )

    # C) After dsr_controller2 exits with code 0, spawn dsr_moveit_controller (MoveIt-compatible trajectory controller).
    def _step3_on_exit(event, context):
        if event.returncode != 0:
            return []  # _step2_on_exit watches the same spawner and reports the failure.
        return [
            LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
            dsr_moveit_controller_spawner,
        ]

    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=_step3_on_exit,
        )
    )

    # D) After dsr_moveit_controller exits with code 0, start MoveGroup (and RViz if gui=true).
    def _step4_on_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f">> [STEP 4 FAILED] dsr_moveit_controller spawner exited with code {event.returncode}"), EmitEvent(event=Shutdown(reason="STEP 4 failed"))]
        return [
            LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup (+ RViz if gui=true)..."),
            rviz_and_move_group,
        ]

    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=_step4_on_exit,
        )
    )

    nodes = [
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),
        robot_description_action,
        run_emulator_node,
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_joint_state,
        delay_gripper_after_robot_controller,
        delay_dsr_moveit_controller_after_robot_controller,
        delay_rviz_after_moveit_controller,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
