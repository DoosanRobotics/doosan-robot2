
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext
from typing import List

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

args =[ 
    DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'     ),
    DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
    DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
    DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
    DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
    DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
]

def generate_launch_description():

    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')
    # planning_context
    robot_description = {'robot_description' : Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])}

    robot_description_semantic_config = load_file('dsr_moveit_config_m1013', 'config/m1013.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('dsr_moveit_config_m1013', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    # Planning Functionality
    ompl_planning_pipeline_config = { 'move_group' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 1000 } }
    ompl_planning_yaml = load_yaml('dsr_moveit_config_m1013', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('dsr_moveit_config_m1013', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True}

    # Start the actual move_group node/action server
    run_move_group_node = Node(package='moveit_ros_move_group',
                               executable='move_group',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           trajectory_execution,
                                           moveit_controllers,
                                           planning_scene_monitor_parameters])

    # RViz
    rviz_config_file = get_package_share_directory('dsr_description2') + "/rviz/moveit.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base', 'base_link'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    # Fake joint driver
    fake_joint_driver_node = Node(package='fake_joint_driver',
                                  executable='fake_joint_driver_node',
                                  parameters=[{'controller_name': 'dsr_joint_trajectory_controller'},
                                              os.path.join(get_package_share_directory("dsr_moveit_config_m1013"), "config", "fake_controllers.yaml"),
                                              os.path.join(get_package_share_directory("dsr_moveit_config_m1013"), "config", "start_positions.yaml"),
                                              robot_description]
                                  )

    # Warehouse mongodb server
    mongodb_server_node = Node(package='warehouse_ros_mongo',
                               executable='mongo_wrapper_ros.py',
                               parameters=[{'warehouse_port': 33829},
                                           {'warehouse_host': 'localhost'},
                                           {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'}],
                               output='screen')

        # dsr_control2 
    dsr_control2 = Node(package='dsr_control2', 
                        executable='dsr_control_node2', 
                        name='dsr_control_node2', 
                        output='screen',
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
                            #parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.    
                        ]
                    )

    return LaunchDescription(args + [ rviz_node, static_tf, robot_state_publisher, run_move_group_node, fake_joint_driver_node, mongodb_server_node, dsr_control2 ])
