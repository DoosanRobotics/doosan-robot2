# dsr_bringup2/launch/rviz.launch.py
import os

from launch import LaunchDescription
from launch.actions import (RegisterEventHandler, DeclareLaunchArgument)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, PathJoinSubstitution,
                                  LaunchConfiguration, PythonExpression)
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='dsr01',          description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1',      description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345',          description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual',        description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='m1013',          description='ROBOT_MODEL'),
        DeclareLaunchArgument('gripper', default_value='robotiq_85',   description='Gripper model'),
        DeclareLaunchArgument('color', default_value='white',          description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='false',          description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false',          description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        # NEW: gripper serial port for real mode
        DeclareLaunchArgument('gripper_com_port', default_value='/dev/ttyUSB0', description='Gripper serial port'),
    ]

    xacro_path = os.path.join(get_package_share_directory('dsr_description2'), 'xacro')

    mode = LaunchConfiguration('mode')

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration('model')
            ]),
            ".urdf.xacro",
            " color:=",   LaunchConfiguration('color'),
            " gripper:=", LaunchConfiguration('gripper'),
            " use_gazebo:=", LaunchConfiguration('gz'),
            # 필요하면 네임스페이스도: " namespace:=", LaunchConfiguration('name')
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Controller YAML
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml']
    )

    # Nodes
    set_config_node = Node(
        package='dsr_bringup2',
        executable='set_config',
        namespace=LaunchConfiguration('name'),
        parameters=[{
            'name':    LaunchConfiguration('name'),
            'rate':    100,
            'standby': 5000,
            'command': True,
            'host':    LaunchConfiguration('host'),
            'port':    LaunchConfiguration('port'),
            'mode':    LaunchConfiguration('mode'),
            'model':   LaunchConfiguration('model'),
            'gripper': LaunchConfiguration('gripper'),
            'mobile':  'none',
            'rt_host': LaunchConfiguration('rt_host'),
        }],
        output='screen',
    )

    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace=LaunchConfiguration('name'),
        parameters=[{
            'name':    LaunchConfiguration('name'),
            'rate':    100,
            'standby': 5000,
            'command': True,
            'host':    LaunchConfiguration('host'),
            'port':    LaunchConfiguration('port'),
            'mode':    LaunchConfiguration('mode'),
            'model':   LaunchConfiguration('model'),
            'gripper': LaunchConfiguration('gripper'),
            'mobile':  'none',
            'rt_host': LaunchConfiguration('rt_host'),
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output='screen',
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description,
                    PathJoinSubstitution([FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"])],
        output="both",
    )


    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{"robot_description": robot_description_content}],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dsr_description2'), 'rviz', 'default.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('name'),
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        namespace=LaunchConfiguration('name'),
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        namespace=LaunchConfiguration('name'),
        executable='spawner',
        arguments=['dsr_controller2', '-c', 'controller_manager'],
    )

    # Gripper controllers
    robotiq_activation_controller_spawner = Node(
        package='controller_manager',
        namespace=LaunchConfiguration('name'),
        executable='spawner',
        arguments=['robotiq_activation_controller', '-c', 'controller_manager'],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
    )

    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        namespace=LaunchConfiguration('name'),
        executable='spawner',
        arguments=['robotiq_gripper_controller', '-c', 'controller_manager'],
    )

    # Event chaining
    delay_control_node_after_config = RegisterEventHandler(
        OnProcessExit(
            target_action=set_config_node,
            on_exit=[control_node],
        )
    )

    delay_robot_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # real: robot -> activation -> gripper
    delay_activation_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[robotiq_activation_controller_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
    )
    delay_gripper_after_activation = RegisterEventHandler(
        OnProcessExit(
            target_action=robotiq_activation_controller_spawner,
            on_exit=[robotiq_gripper_controller_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
    )

    # virtual: robot -> gripper
    delay_gripper_after_robot_virtual = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[robotiq_gripper_controller_spawner],
        ),
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'real'"])),
    )

    delay_rviz_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        set_config_node,
        run_emulator_node,

        delay_control_node_after_config,

        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_jsb,

        delay_activation_after_robot,
        delay_gripper_after_activation,
        delay_gripper_after_robot_virtual,

        delay_rviz_after_robot,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
