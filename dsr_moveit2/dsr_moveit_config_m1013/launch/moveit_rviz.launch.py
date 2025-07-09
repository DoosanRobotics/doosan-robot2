from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("m1013", package_name="dsr_moveit_config_m1013").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
