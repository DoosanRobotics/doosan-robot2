import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    
    ARGUMENTS =[ 
        DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',  default_value = '192.168.137.100', description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',  default_value = 'real',   description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model', default_value = 'm0617',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('gz',    default_value = 'false',     description = 'USE GAZEBO SIM'    ),
    ]

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    moveit_config = (
        MoveItConfigsBuilder("m0617")
        .robot_description(file_path="config/m0617.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace="m0617",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0617"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace="m0617",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        remappings=[
            (
                "/joint_states",
                "/dsr01/joint_states",
            ),
        ],
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0617"),
        "config",
        "ros2_controllers.yaml",
    )
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

    robot_description = {"robot_description": robot_description_content}

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("dsr_controller2"),
    #     "config",
    #     "dsr_controller2.yaml",
    # )   
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],

        # parameters=[robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dsr_moveit_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dsr_controller2", "-c", "/controller_manager"],
    )

    # Warehouse mongodb server
    # db_config = LaunchConfiguration("db")
    # mongodb_server_node = Node(
    #     package="warehouse_ros_mongo",
    #     executable="mongo_wrapper_ros.py",
    #     parameters=[
    #         {"warehouse_port": 33829},
    #         {"warehouse_host": "localhost"},
    #         {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
    #     ],
    #     output="screen",
    #     condition=IfCondition(db_config),
    # )

    return LaunchDescription(ARGUMENTS +
        [
            db_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            # robot_controller_spawner,
            # mongodb_server_node,
            joint_state_broadcaster_spawner,
            dsr_moveit_controller_spawner,
        ]
    )
