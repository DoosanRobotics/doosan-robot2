import os
import yaml
import subprocess
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF

def adjust_dsr_controller_yaml(yaml_path, active_joints, passive_joints):
    # Adjust dsr_controller2.yaml to replace the 'joints' list with active_joint and remove any 'passive_joints' entries.
    temp_yaml = "/tmp/adjusted_dsr_controller2.yaml"

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    controllers = [
        "dsr_controller2",
        "dsr_moveit_controller",
        "dsr_position_controller",
        "dsr_joint_trajectory",
    ]

    # Update 'joints' for each target controller
    for ctrl in controllers:
        if ctrl in data:
            params = data[ctrl].get("ros__parameters", {})
            params["joints"] = list(active_joints)
            if "passive_joints" in params:
                params.pop("passive_joints", None)  # Remove passive_joints if exists
            data[ctrl]["ros__parameters"] = params

    # Save modified YAML to a temporary file
    with open(temp_yaml, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

    return temp_yaml

def parse_joints_from_urdf(model, color=None):
    # Parse URDF generated from a xacro file to extract active (non-fixed) and passive (fixed) joints.
    if color is None:
        color = "white"  # default color

    # Build xacro file path
    xacro_file = os.path.join(
        get_package_share_directory('dsr_description2'),
        'xacro',
        f"{model}.urdf.xacro"
    )

    # Execute xacro to generate URDF XML
    urdf_xml = subprocess.check_output(
        ['xacro', xacro_file, f'color:={color}']
    ).decode('utf-8')

    # Parse URDF
    robot_model = URDF.from_xml_string(urdf_xml)

    # Extract active and passive joints
    active_joints = [j.name for j in robot_model.joints if j.type != "fixed"]
    passive_joints = [j.name for j in robot_model.joints if j.type == "fixed"]

    return urdf_xml, active_joints, passive_joints
