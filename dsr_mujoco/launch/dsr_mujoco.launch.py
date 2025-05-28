# 
#  dsr_mujoco
#  Author: Theo Choi (theo.choi@doosan.com)
#  
#  Copyright (c) 2025 Doosan Robotics
#  Use of this source code is governed by the Apache License 2.0, see LICENSE
# 

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from dsr_mujoco.dsr_merge_gripper import merge_gripper
from dsr_mujoco.dsr_build_scene import build_scene
from launch.conditions import IfCondition
from pathlib import Path

ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('use_mujoco',   default_value = 'true',     description = 'Start Mujoco'    ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gripper', default_value='none', description='Use gripper'),
        DeclareLaunchArgument('scene_path', default_value='none', description='Relative path to MuJoCo scene XML file (demo/slope_demo_scene.xml)'),
        # DeclareLaunchArgument('controller_param_file', default_value='config/dsr_mujoco_controller.yaml', description='Controller YAML file'),
    ]

# Merge dsr and gripper xml, build scene
def prepare_mjcf_files_for_mujoco(context, *args, **kwargs):
    dsr_description_share = Path(get_package_share_directory("dsr_description2"))

    model_arg   = context.launch_configurations['model']
    gripper_arg = context.launch_configurations['gripper']
    input_scene_arg = context.launch_configurations['scene_path']

    # This is where merged and new scene XML files will be saved.
    output_dir_for_generated_files = dsr_description_share / "mujoco_models" / model_arg
    output_dir_for_generated_files.mkdir(parents=True, exist_ok=True) # Ensure the directory exists

    arm_xml_path  = dsr_description_share / "mujoco_models" / model_arg / f"{model_arg}.xml"
    if not arm_xml_path.exists():
        raise FileNotFoundError(f"Base arm XML file of '{model_arg}' not found: {arm_xml_path}, might be not supported yet")

    robot_model_to_include_in_scene_path: Path

    # Merge gripper
    if gripper_arg.lower() != 'none':
        # If a gripper is specified
        hand_xml_path = dsr_description_share / "mujoco_models" / gripper_arg / f"{gripper_arg}.xml"
        if not hand_xml_path.exists():
            raise FileNotFoundError(f"Gripper XML for '{gripper_arg}' not found: {hand_xml_path}")
        
        # Merge the arm and gripper XML files
        merged_robot_path = merge_gripper(arm_xml_path, hand_xml_path, output_dir_for_generated_files)
        robot_model_to_include_in_scene_path = merged_robot_path
        print(f"[Launch] Merged robot model generated: {robot_model_to_include_in_scene_path}")
    else:
        # If no gripper is specified, use base arm XML
        robot_model_to_include_in_scene_path = arm_xml_path
        print(f"[Launch] Using base arm model (no gripper): {robot_model_to_include_in_scene_path}")

    original_scene_template_path: Path
    # Prepare to build scene
    if input_scene_arg and input_scene_arg.lower() != 'none':
        p_candidate = Path(input_scene_arg)
        # Both works for ablsolute and relative
        if p_candidate.is_absolute() and p_candidate.exists():
            original_scene_template_path = p_candidate
        else:
            p_relative_candidate = dsr_description_share / "mujoco_models" / input_scene_arg.lstrip('/')
            if p_relative_candidate.exists():
                original_scene_template_path = p_relative_candidate
            else:
                error_msg = f"Input scene XML '{input_scene_arg}' not found. "
                if p_candidate.is_absolute():
                    error_msg += f"Tried absolute path: {p_candidate}. "
                error_msg += f"Also tried as relative path from '{dsr_description_share / 'mujoco_models'}' at: {p_relative_candidate}."
                raise FileNotFoundError(error_msg)
    else:
        # If no scene path is provided, use the default scene
        original_scene_template_path = dsr_description_share / "mujoco_models" / model_arg / "scene.xml"
        if not original_scene_template_path.exists():
            raise FileNotFoundError(f"Default scene.xml for model '{model_arg}' not found at {original_scene_template_path}")
    
    print(f"[Launch] Using original scene template: {original_scene_template_path}")

    # This is used by build_scene to include tag
    arm_model_filename_in_template = f"{model_arg}.xml" 
    
    # Build new scene
    generated_scene_path = build_scene(
        original_scene_xml_path=original_scene_template_path,
        robot_model_to_include=robot_model_to_include_in_scene_path, 
        output_dir=output_dir_for_generated_files, 
        arm_model_filename_in_original_scene=arm_model_filename_in_template
    )
    print(f"[Launch] Generated final scene file for MuJoCo: {generated_scene_path}")
   
    return [
        SetLaunchConfiguration("robot_model_xml_for_scene", str(robot_model_to_include_in_scene_path)),
        SetLaunchConfiguration("scene_path", str(generated_scene_path)), 
        LogInfo(msg=f"MuJoCo will use scene: {generated_scene_path}"),
        LogInfo(msg=f"Robot model included in scene: {robot_model_to_include_in_scene_path.name} (located at {robot_model_to_include_in_scene_path})"),
    ]

# Set controller YAML file (using gripper needs other)
def prepare_controller_config(context, *args, **kwargs):
    dsr_mujoco_share = Path(get_package_share_directory('dsr_mujoco'))
    gripper_arg = context.launch_configurations['gripper']
    
    controller_config_file: Path
    if gripper_arg.lower() != 'none':
        controller_config_file = dsr_mujoco_share / 'config' / f"dsr_mujoco_controller_with_{gripper_arg}.yaml"
        if not controller_config_file.exists():
            raise FileNotFoundError(f"Controller YAML for gripper '{gripper_arg}' not found.")
    else:
        # Gripper is 'none', use the default controller YAML
        controller_config_file = dsr_mujoco_share / 'config' / "dsr_mujoco_controller.yaml"
    
    return [
        SetLaunchConfiguration("controller_param_file",  str(controller_config_file)),
        LogInfo(msg=f"Controller config: {controller_config_file}"),
    ]

def generate_launch_description():

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
            " ",
            "use_mujoco:=",
            LaunchConfiguration('use_mujoco'),
            " ",
            "color:=",
            LaunchConfiguration('color'),
            " ",
            "namespace:=",
            PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
            " ",
            "gripper:=",  
            LaunchConfiguration('gripper'),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_param_file = LaunchConfiguration('controller_param_file')
    
    # Mujoco node
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        output='screen',
        parameters=[
            robot_description,
            controller_param_file,
            {'mujoco_model_path': LaunchConfiguration('scene_path')},
            {"use_sim_time": LaunchConfiguration('use_sim_time')},
        ],
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        arguments=[
            "joint_state_broadcaster", 
            "-c", "controller_manager",  
        ],
        output="screen",
    )
    
    # Position controller
    dsr_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        arguments=[
            "dsr_position_controller", 
            "-c", "controller_manager", 
        ],
        output="screen",
    )
    
    # Gripper controller for 2f85
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        arguments=[
            'left_knuckle_position_controller',
            '-c','controller_manager'
            ],
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('gripper'), "' == '2f85'"]))
    )
    
    # Delay mujoco's robot joint broadcaster after mujoco node start
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=0.2,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    # Delay mujoco's robot controller after mujoco node start
    delay_dsr_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=0.2,
                    actions=[
                        dsr_position_controller_spawner, 
                        gripper_controller_spawner
                        ]
                )
            ]
        )
    )
    
    return LaunchDescription(ARGUMENTS + [
        # Merge arm and gripper xmls
        OpaqueFunction(function=prepare_mjcf_files_for_mujoco),
        # Set controller YAML file
        OpaqueFunction(function=prepare_controller_config),

        node_mujoco,
        delay_joint_state_broadcaster,
        delay_dsr_position_controller,
    ])

