# 
#  dsr_bringup2
#  Author: Theo Choi (theo.choi@doosan.com)
#  
#  Copyright (c) 2025 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from dsr_mujoco.dsr_merge_gripper import merge_gripper
from pathlib import Path

ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('use_mujoco',   default_value = 'true',     description = 'Start Mujoco'    ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gripper', default_value='2f85', description='Use gripper'),
        DeclareLaunchArgument('scene_path', default_value='none', description='Relative path to MuJoCo scene XML file (demo/slope_demo_scene.xml)'),
        # DeclareLaunchArgument('controller_param_file', default_value='config/dsr_mujoco_controller.yaml', description='Controller YAML file'),
    ]

# Merge dsr and gripper xmls and set the YAML file
def prepare_files_for_mujoco(context, *args, **kwargs):
    share = Path(get_package_share_directory("dsr_description2"))
 
    model_arg   = context.launch_configurations['model']
    gripper_arg = context.launch_configurations['gripper']
    scene_arg = context.launch_configurations['scene_path']

    arm = share / "mujoco_models" / model_arg / f"{model_arg}.xml"
    hand = share / "mujoco_models" / gripper_arg / f"{gripper_arg}.xml"
    # Use demo or custom path for scene
    if scene_arg and scene_arg.lower() != 'none':
        p = Path(scene_arg)
        if p.is_absolute() and p.exists():
            scene = p
        else:
            rel = share / "mujoco_models" / scene_arg.lstrip('/')
            if rel.exists():
                scene = rel
            else:
                raise FileNotFoundError(f"Scene XML not found for {scene_arg}")
    # Default blank scene
    else:
        scene = share / "mujoco_models" / model_arg / "scene.xml"
        if not scene.exists():
            raise FileNotFoundError(f"Default scene.xml not found for {model_arg}")

    merged_path, scene_path = merge_gripper(arm, hand, scene)

    # Bring the yaml file
    ctrl = Path(get_package_share_directory('dsr_mujoco')) \
           / 'config' \
           / f"dsr_mujoco_controller_with_{gripper_arg}.yaml"
    if not ctrl.exists():
        raise FileNotFoundError(f"Controller YAML not found: {ctrl}")
        
    return [
        SetLaunchConfiguration("merged_mjcf", str(merged_path)),
        SetLaunchConfiguration("scene_path",   str(scene_path)),
        SetLaunchConfiguration("controller_param_file",  str(ctrl)),
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
    
    # controller_param_file = PathJoinSubstitution(
    #     [FindPackageShare("dsr_mujoco"), "config",
    #     "dsr_mujoco_controller_with_2f85.yaml"]
    # )
    controller_param_file = LaunchConfiguration('controller_param_file')

    from launch.actions import LogInfo

    print_path = LogInfo(
        msg=[ "!!!! Controller YAML !!!! → ",
            controller_param_file ]
    )

    # Mujoco node
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        output='screen',
        parameters=[
            robot_description,
            controller_param_file,
            # {'mujoco_model_path': mujoco_model_path,},
            {'mujoco_model_path': LaunchConfiguration('scene_path')},
            {"use_sim_time": True},
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
    
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=PathJoinSubstitution([LaunchConfiguration('name'), "mj"]),
        arguments=[
            'left_knuckle_position_controller',
            '-c','controller_manager'
            ],
        output="screen",
    )
    
    # Delay mujoco's robot joint broadcaster after mujoco node start
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=1.0,
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
                    period=1.0,
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
        OpaqueFunction(function=prepare_files_for_mujoco),
        
        node_mujoco,
        delay_joint_state_broadcaster,
        delay_dsr_position_controller,

        print_path,  # to do : remove this
    ])

