#  dsr_moveit2 (reworked)
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  License: Apache-2.0

import os

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Moveit2
from moveit_configs_utils import MoveItConfigsBuilder

# dsr utils
from dsr_bringup2.controller_config import adjust_dsr_controller_yaml, parse_joints_from_urdf


# === [STEP 0] robot_description + controller_yaml 동적 결정 =====================
def generate_robot_description_action(context, *args, **kwargs):
    dynamic_yaml = LaunchConfiguration('dynamic_yaml').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)

    # URDF 파싱
    urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color, gripper)

    # controller.yaml 결정
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
        SetLaunchConfiguration('controller_yaml', adjusted_yaml)
    ]


# === [STEP 1] MoveGroup & RViz =================================================
def rviz_node_function(context):
    model_value = LaunchConfiguration('model').perform(context)
    package_name_str = f"dsr_moveit_config_{model_value}"
    package_path_str = FindPackageShare(package_name_str).perform(context)
    print("Package:", package_name_str)
    print("Package Path:", package_path_str)

    moveit_config = (
        MoveItConfigsBuilder(model_value, "robot_description", package_name_str)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/m1013.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp"],
            default_planning_pipeline="ompl",
            load_all=False
        )
        .to_moveit_configs()
    )

    # robot_description를 LaunchConfiguration 값으로 오버라이드
    common_params = [
        moveit_config.to_dict(),
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=common_params,
    )

    rviz_base = os.path.join(get_package_share_directory(package_name_str), "launch")
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


# === [STEP 2] ros2_control_node (controller_manager) 구성 ======================
def control_node_fn(context):
    params = [{
        "robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
    }, LaunchConfiguration('controller_yaml')]

    # gripper=robotique 이면 gripper_controller.yaml을 추가 로드
    if LaunchConfiguration('gripper').perform(context) == 'robotique':
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


# === [STEP 3] 그리퍼 스포너 (조건부) ============================================
def gripper_spawner_fn(context):
    if LaunchConfiguration('gripper').perform(context) != 'robotique':
        return []

    # spawner에는 --controller-type 옵션이 없음. 타입은 YAML에 선언됨.
    return [Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "-c", "controller_manager",
            # 필요 시 컨트롤러 개별 파라미터를 별도로 넘기려면 아래 주석 해제
            # "--param-file", os.path.join(get_package_share_directory("dsr_controller2"), "config", "gripper_controller.yaml"),
        ],
        output="screen",
    )]


# === 메인 Launch =================================================================
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
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotique)'),
    ]

    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

    # 설정/에뮬레이터
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

    # controller_manager (조건부 파라미터 포함)
    control_node = OpaqueFunction(function=control_node_fn)

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
        arguments=["dsr_moveit_controller", "-c", "controller_manager"],
    )

    rviz_node = OpaqueFunction(function=rviz_node_function)
    gripper_spawner = OpaqueFunction(function=gripper_spawner_fn)

    # === 이벤트 체인 ===
    delay_control_node_after_set_config = RegisterEventHandler(
        OnProcessExit(
            target_action=set_config_node,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] set_config_node finished. Starting ros2_control_node..."),
                control_node
            ],
        )
    )

    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    )

    # dsr_controller2 활성 후: (조건부) 그리퍼, 이어서 MoveIt 컨트롤러 병렬로 진행
    delay_gripper_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3a] dsr_controller2 active. (cond) starting gripper_position_controller..."),
                gripper_spawner
            ],
        )
    )

    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup + RViz2..."),
                rviz_node
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
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_joint_state,
        delay_gripper_after_robot_controller,            # (조건부) 그리퍼
        delay_dsr_moveit_controller_after_robot_controller,
        delay_rviz_after_moveit_controller,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
