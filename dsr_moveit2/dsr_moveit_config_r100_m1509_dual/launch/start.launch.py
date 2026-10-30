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
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
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
    left_host = LaunchConfiguration('left_host').perform(context)
    right_host = LaunchConfiguration('right_host').perform(context)
    rt_host = LaunchConfiguration('rt_host').perform(context)
    port = LaunchConfiguration('port').perform(context)
    left_port = LaunchConfiguration('left_port').perform(context)
    right_port = LaunchConfiguration('right_port').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    update_rate = read_update_rate() # get update_rate from yaml

    if model == "r100_m1509_dual":
        # Build robot_description from the dual MoveIt wrapper that instantiates the composite macro.
        urdf_xml = Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("dsr_moveit_config_r100_m1509_dual"),
                "config",
                "r100_m1509_dual.urdf.xacro"
            ]),
            " ",
            "color:=", color,
            " ",
            "arm_prefix:=", "left_",
            " ",
            "arm2_prefix:=", "right_",
            " ",
            "enable_ros2_control:=", "true",
            " ",
            "host:=", left_host,
            " ",
            "port:=", left_port,
            " ",
            "rt_host:=", rt_host,
            " ",
            "mode:=", mode,
            " ",
            "model:=", "m1509",
            " ",
            "update_rate:=", str(update_rate),
            " ",
            "arm2_host:=", right_host,
            " ",
            "arm2_port:=", right_port,
            " ",
            "arm2_rt_host:=", rt_host,
            " ",
            "arm2_mode:=", mode,
            " ",
            "arm2_model:=", "m1509",
            " ",
            "arm2_update_rate:=", str(update_rate),
        ])
        active_joints, passive_joints = [], []
        print(f"[DEBUG] model={model}, color={color}, name={name}, host={host}, left_host={left_host}, right_host={right_host}, rt_host={rt_host}, port={port}, left_port={left_port}, right_port={right_port}, mode={mode}, update_rate={update_rate}")
        print(f"[DEBUG] active_joints={active_joints}")
        print(f"[DEBUG] passive_joints={passive_joints}")
    else:
        # Parse URDF to extract active and passive joints
        urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color, name, host, rt_host, port, mode, update_rate)
        print(f"[DEBUG] model={model}, color={color}, name={name}, host={host}, rt_host={rt_host}, port={port}, mode={mode}, update_rate={update_rate}")
        print(f"[DEBUG] active_joints={active_joints}")
        print(f"[DEBUG] passive_joints={passive_joints}")

    # Decide controller YAML
    # For r100_m1509_dual, use the local ros2_controllers.yaml (no dsr_controller2)
    if model == "r100_m1509_dual":
        adjusted_yaml = os.path.join(
            get_package_share_directory(f"dsr_moveit_config_{model}"),
            "config",
            "ros2_controllers.yaml"
        )
        print(f"[INFO] Using r100_m1509_dual controller.yaml: {adjusted_yaml}")
    elif dynamic_yaml:
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
    ns_value = LaunchConfiguration('name').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    if model_value == "r100_m1509_dual":
        joint_states_topic = f"/{ns_value}/joint_states_whole" if ns_value else "/joint_states_whole"
    else:
        joint_states_topic = f"/{ns_value}/joint_states" if ns_value else "/joint_states"

    package_name = f"dsr_moveit_config_{model_value}"
    package_path = FindPackageShare(package_name).perform(context)
    print("MoveIt Config Package:", package_name)
    print("Package Path:", package_path)

    # 
    # Keep pipeline deterministic for whole-body UI interactions.

    moveit_config = (
        MoveItConfigsBuilder(model_value, "robot_description", package_name)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf.xacro", mappings={'gripper': LaunchConfiguration('gripper')})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"],      # List of planning pipelines to load (each loaded from config/<name>_planning.yaml)
                            default_planning_pipeline="ompl", # Name of the default planning pipeline (used if none is explicitly selected)
                            load_all= False                   # If pipelines is None: True loads all from config/default packages; False loads only from config package
                            )
        .to_moveit_configs()
    )
    
    move_group_params = [
        moveit_config.to_dict(),  # robot_description & robot_description_semantic from MoveitConfigbuilder
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
        {"publish_robot_description": True},
        {"publish_robot_description_semantic": True},
        {"planning_scene_monitor_options": {"joint_state_topic": joint_states_topic}},
        {"joint_state_topic": joint_states_topic},
        {"publish_planning_scene": True},
        {"publish_geometry_updates": True},
        {"publish_state_updates": True},
        {"publish_transforms_updates": True},
        {"publish_planning_scene_frequency": 30.0},
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
        namespace=ns_value,
        output="screen",
        parameters=move_group_params,
        remappings=[
            ("/joint_states", joint_states_topic),
            ("joint_states", joint_states_topic),
        ],
    )

    # RViz is optional; move_group is not. Returning here also skips writing the temporary RViz config.
    if not gui:
        return [run_move_group_node]

    rviz_base = os.path.join(get_package_share_directory(package_name), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_full_config, "--ros-args", "--log-level", "fatal"],
        parameters=rviz_params,
        remappings=[
            ("goal_pose", "/goal_pose"),
            ("initialpose", "/initialpose"),
        ],
    )
    actions = [run_move_group_node]
    if gui:
        # Delay RViz startup so first rendered pose reflects settled joint states.
        actions.append(TimerAction(period=1.0, actions=[rviz_node]))
    return actions

# sets up the parameters for the controller manager node, if 'gripper' argument is setted, it additionally loads the 'gripper_controller.yaml' file
def control_node_fn(context):
    model_value = LaunchConfiguration('model').perform(context)
    ns_value = LaunchConfiguration('name').perform(context)
    cm_logger = f"{ns_value}.controller_manager" if ns_value else "controller_manager"
    if model_value == "r100_m1509_dual":
        controller_yaml = os.path.join(
            get_package_share_directory("dsr_moveit_config_r100_m1509_dual"),
            "config",
            "ros2_controllers.yaml"
        )
        print(f"[INFO] Using r100_m1509_dual controller.yaml in controller_manager: {controller_yaml}")
        params = [{"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}, controller_yaml]
    else:
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
        arguments=[
            "--ros-args",
            "--log-level", "dsr_controller2:=warn",
            "--log-level", f"{cm_logger}:=error",
        ],
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


def rviz_refresh_helper_fn(context):
    model_value = LaunchConfiguration('model').perform(context)
    enabled = LaunchConfiguration('enable_rviz_refresh').perform(context).lower() in ['true', '1']
    if not enabled:
        return []

    ns_value = LaunchConfiguration('name').perform(context)
    if ns_value:
        status_topic = f"/{ns_value}/move_action/_action/status"
        planning_scene_topic = f"/{ns_value}/planning_scene"
        monitored_scene_topic = f"/{ns_value}/monitored_planning_scene"
        clear_octomap_service = f"/{ns_value}/clear_octomap"
    else:
        status_topic = "/move_action/_action/status"
        planning_scene_topic = "/planning_scene"
        monitored_scene_topic = "/monitored_planning_scene"
        clear_octomap_service = "/clear_octomap"

    cmd = (
        "set +e; "
        f"STATUS_TOPIC='{status_topic}'; "
        f"SCENE_TOPIC='{planning_scene_topic}'; "
        f"MON_SCENE_TOPIC='{monitored_scene_topic}'; "
        f"CLEAR_OCTOMAP_SRV='{clear_octomap_service}'; "
        "LOG_FILE=/tmp/wb_refresh.log; "
        "echo \"[WB][REFRESH] helper started (status_topic=$STATUS_TOPIC)\" | tee -a \"$LOG_FILE\"; "
        "last_hit=0; "
        "while true; do "
        "hit=$(ros2 topic echo --once \"$STATUS_TOPIC\" 2>/dev/null | grep -c \"status: 4\"); "
        "if [ \"$hit\" -gt 0 ] && [ \"$last_hit\" -eq 0 ]; then "
        "echo \"[WB][REFRESH] execute success detected -> refresh scene\" | tee -a \"$LOG_FILE\"; "
        "ros2 topic pub --once \"$SCENE_TOPIC\" moveit_msgs/msg/PlanningScene \"{is_diff: true}\" >/dev/null 2>&1 || true; "
        "ros2 topic pub --once \"$MON_SCENE_TOPIC\" moveit_msgs/msg/PlanningScene \"{is_diff: true}\" >/dev/null 2>&1 || true; "
        "ros2 service call \"$CLEAR_OCTOMAP_SRV\" std_srvs/srv/Empty \"{}\" >/dev/null 2>&1 || true; "
        "last_hit=1; "
        "elif [ \"$hit\" -eq 0 ]; then "
        "last_hit=0; "
        "fi; "
        "sleep 0.5; "
        "done"
    )
    return [
        LogInfo(msg=f">> [WB][REFRESH] helper watching: {status_topic}"),
        ExecuteProcess(cmd=["bash", "-lc", cmd], output="screen"),
    ]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='r100_m1509_dual', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('left_host', default_value=LaunchConfiguration('host'), description='LEFT_ROBOT_IP'),
        DeclareLaunchArgument('right_host', default_value=LaunchConfiguration('host'), description='RIGHT_ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('left_port', default_value=LaunchConfiguration('port'), description='LEFT_ROBOT_PORT'),
        DeclareLaunchArgument('right_port', default_value='12348', description='RIGHT_ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='r100_m1509_dual', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('dynamic_yaml', default_value='true', description='Use dynamic controller.yaml'),
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotiq_2f85)'),
        DeclareLaunchArgument('left_init_on_start', default_value='true', description='Move left arm J1 to 180deg once at startup (dual only)'),
        DeclareLaunchArgument('enable_whole_body_controller', default_value='true', description='Spawn whole_body_controller scaffold plugin'),
        DeclareLaunchArgument('enable_rviz_refresh', default_value='true', description='Auto-refresh planning scene after execute (RViz whole-body UI refresh)'),
        DeclareLaunchArgument('tf_prefix', default_value='', description='PREFIX FOR TF FRAME NAMES (e.g. "dsr01/")'),
    ]

    # Build robot_description and select controller YAML
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

    # Run emulator for left arm
    left_run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": "left_m1509",
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('left_host'),
            "port": LaunchConfiguration('left_port'),
            "mode": LaunchConfiguration('mode'),
            "model": "m1509",
            "gripper": LaunchConfiguration('gripper'),
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])),
        output="screen",
    )

    # Run emulator for right arm
    right_run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": "right_m1509",
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('right_host'),
            "port": LaunchConfiguration('right_port'),
            "mode": LaunchConfiguration('mode'),
            "model": "m1509",
            "gripper": LaunchConfiguration('gripper'),
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])),
        output="screen",
    )

    joint_states_remap = PythonExpression([
        "('/' + '", LaunchConfiguration('name'), "' + '/joint_states_whole') if ('",
        LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('name'), "' != '') else ('/joint_states_whole' if '",
        LaunchConfiguration('model'), "' == 'r100_m1509_dual' else ('/' + '",
        LaunchConfiguration('name'), "' + '/joint_states' if '",
        LaunchConfiguration('name'), "' != '' else '/joint_states'))"
    ])

    # Run robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('joint_states', joint_states_remap),
        ],
        parameters=[{'frame_prefix': LaunchConfiguration('tf_prefix')}, {
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
        }],
    )

    # Run ros2_control_node(controller_manager)
    control_node = OpaqueFunction(function=control_node_fn)

    is_r100_m1509 = PythonExpression(["'", LaunchConfiguration('model'), "' == 'r100_m1509'"])
    is_r100_m1509_dual = PythonExpression(["'", LaunchConfiguration('model'), "' == 'r100_m1509_dual'"])
    left_init_enabled = PythonExpression([
        "'", LaunchConfiguration('left_init_on_start'), "'.lower() in ['true','1']"
    ])
    left_init_disabled = PythonExpression([
        "'", LaunchConfiguration('left_init_on_start'), "'.lower() not in ['true','1']"
    ])
    whole_body_enabled = PythonExpression([
        "'", LaunchConfiguration('enable_whole_body_controller'), "'.lower() in ['true','1']"
    ])
    whole_body_disabled = PythonExpression([
        "'", LaunchConfiguration('enable_whole_body_controller'), "'.lower() not in ['true','1']"
    ])

    is_dual_and_init_enabled = PythonExpression([
        "'", LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('left_init_on_start'), "'.lower() in ['true','1']"
    ])
    is_dual_and_whole_enabled = PythonExpression([
        "'", LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('enable_whole_body_controller'), "'.lower() in ['true','1']"
    ])
    is_dual_and_init_and_whole = PythonExpression([
        "'", LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('left_init_on_start'), "'.lower() in ['true','1'] and '",
        LaunchConfiguration('enable_whole_body_controller'), "'.lower() in ['true','1']"
    ])
    is_dual_and_init_and_no_whole = PythonExpression([
        "'", LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('left_init_on_start'), "'.lower() in ['true','1'] and '",
        LaunchConfiguration('enable_whole_body_controller'), "'.lower() not in ['true','1']"
    ])
    is_dual_and_no_init_and_whole = PythonExpression([
        "'", LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('left_init_on_start'), "'.lower() not in ['true','1'] and '",
        LaunchConfiguration('enable_whole_body_controller'), "'.lower() in ['true','1']"
    ])
    is_dual_and_no_init_and_no_whole = PythonExpression([
        "'", LaunchConfiguration('model'), "' == 'r100_m1509_dual' and '",
        LaunchConfiguration('left_init_on_start'), "'.lower() not in ['true','1'] and '",
        LaunchConfiguration('enable_whole_body_controller'), "'.lower() not in ['true','1']"
    ])
    controller_manager_path = PythonExpression([
        "'/' + '", LaunchConfiguration('name'), "' + '/controller_manager' if '",
        LaunchConfiguration('name'), "' != '' else '/controller_manager'"
    ])

    # Spawn joint_state_broadcaster (normal path)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
        condition=UnlessCondition(is_r100_m1509_dual),
    )

    # Spawn dsr_controller2 (skip for r100_m1509_dual to avoid interface conflicts)
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
        condition=UnlessCondition(is_r100_m1509_dual),
    )

    # Spawn dsr_moveit_controller
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "dsr_moveit_controller",
            "-c", controller_manager_path,
            "--activate",
            "--controller-manager-timeout", "120"
        ],
    )

    # Spawners for r100_m1509_dual (no conditional on the nodes themselves)
    r100_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
    )

    r100_dual_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "dual_dsr_moveit_controller",
            "-c", controller_manager_path,
            "--activate",
            "--controller-manager-timeout", "120"
        ],
    )

    # For r100_m1509_dual: also spawn dsr_controller2 as active (same policy as single-arm path).
    r100_robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
    )

    r100_diff_drive_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "-c", controller_manager_path,
            "--activate",
            "--controller-manager-timeout", "120"
        ],
    )

    r100_whole_body_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "-c", controller_manager_path,
            "--activate",
            "--controller-manager-timeout", "120"
        ],
    )

    # MoveGroup + (optional) RViz
    rviz_and_move_group = OpaqueFunction(function=rviz_and_move_group_fn)

    rviz_refresh_helper = OpaqueFunction(function=rviz_refresh_helper_fn)

    # A) Once joint_state_broadcaster is active, spawn dsr_controller2 (arm controller).
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    ,
        condition=UnlessCondition(is_r100_m1509_dual),
    )

    # B) After dsr_controller2 becomes active, (conditionally) spawn the gripper position controller.
    delay_gripper_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2] dsr_controller2 active. (cond) starting gripper_position_controller..."),
                OpaqueFunction(function=gripper_spawner_fn),
            ],
        )
    )

    # C) After dsr_controller2 becomes active, spawn dsr_moveit_controller (MoveIt-compatible trajectory controller).
    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    # For r100_m1509 (single-arm legacy path): start dsr_moveit_controller right after joint_state_broadcaster
    delay_dsr_moveit_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] joint_state_broadcaster active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        ),
        condition=IfCondition(is_r100_m1509),
    )

    # For r100_m1509_dual: delay spawners to ensure controller_manager is up
    delayed_r100_spawners = TimerAction(
        period=1.0,
        actions=[
            r100_joint_state_broadcaster_spawner,
            r100_robot_controller_spawner,
            r100_dual_moveit_controller_spawner,
        ],
        condition=IfCondition(is_r100_m1509_dual),
    )

    # For r100_m1509_dual: one-shot initial pose (left arm J1 = 180deg) before UI starts.
    left_init_motion_once = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "-t", "20",
            PythonExpression(["'/' + '", LaunchConfiguration('name'), "' + '/dual_dsr_moveit_controller/follow_joint_trajectory'"]),
            "control_msgs/action/FollowJointTrajectory",
            "{trajectory: {joint_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, right_joint_1, right_joint_2, right_joint_3, right_joint_4, right_joint_5, right_joint_6], points: [{positions: [3.14159, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 6, nanosec: 0}}]}}",
        ],
        output="log",
    )

    delay_left_init_motion_after_whole_body = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_whole_body_controller_spawner,
            on_exit=[
                TimerAction(
                    period=0.2,
                    actions=[left_init_motion_once],
                )
            ],
        ),
        condition=IfCondition(is_dual_and_init_and_whole),
    )

    delay_left_init_motion_after_dual_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_dual_moveit_controller_spawner,
            on_exit=[
                TimerAction(
                    period=0.2,
                    actions=[left_init_motion_once],
                )
            ],
        ),
        condition=IfCondition(is_dual_and_init_and_no_whole),
    )

    delay_diff_drive_controller_after_dual_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_dual_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [WB] dual_dsr_moveit_controller active. Starting diff_drive_controller..."),
                r100_diff_drive_controller_spawner,
            ],
        ),
        condition=IfCondition(is_dual_and_whole_enabled),
    )

    delay_whole_body_controller_after_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_diff_drive_controller_spawner,
            on_exit=[
                TimerAction(
                    period=0.3,
                    actions=[
                        LogInfo(msg=">> [WB] diff_drive_controller active. Starting whole_body_controller scaffold..."),
                        r100_whole_body_controller_spawner,
                    ],
                )
            ],
        ),
        condition=IfCondition(is_dual_and_whole_enabled),
    )

    # D) After dsr_moveit_controller is active, start MoveGroup (and RViz if gui=true).
    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        )
    , condition=UnlessCondition(is_r100_m1509_dual))

    # For r100_m1509_dual + no init hook: start MoveGroup + RViz after dual moveit controller
    delay_rviz_after_r100_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_dual_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dual moveit controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        ),
        condition=IfCondition(is_dual_and_no_init_and_no_whole),
    )

    delay_rviz_after_whole_body_no_init = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_whole_body_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [WB] whole_body_controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        ),
        condition=IfCondition(is_dual_and_no_init_and_whole),
    )

    # For r100_m1509_dual + init hook: start MoveGroup + RViz after initial left-arm move completes
    delay_rviz_after_left_init_motion = RegisterEventHandler(
        OnProcessExit(
            target_action=left_init_motion_once,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[
                        rviz_and_move_group
                    ],
                )
            ],
        ),
        condition=IfCondition(is_dual_and_init_enabled),
    )

    nodes = [
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),
        robot_description_action,
        left_run_emulator_node,
        right_run_emulator_node,
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_joint_state,
        delay_gripper_after_robot_controller,
        delay_dsr_moveit_controller_after_robot_controller,
        delay_dsr_moveit_controller_after_joint_state,
        delayed_r100_spawners,
        delay_left_init_motion_after_whole_body,
        delay_left_init_motion_after_dual_moveit,
        delay_diff_drive_controller_after_dual_moveit,
        delay_whole_body_controller_after_diff_drive,
        delay_rviz_after_moveit_controller,
        delay_rviz_after_r100_moveit_controller,
        delay_rviz_after_whole_body_no_init,
        delay_rviz_after_left_init_motion,
        rviz_refresh_helper,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
