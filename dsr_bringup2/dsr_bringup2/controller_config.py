# 
#  dsr_bringup2
#  Author: Gijung Nam (gijung.nam@doosan.com)
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


def parse_joints_from_urdf(model, color=None, gripper='none'):
    if color is None:
        color = "white"

    xacro_file = os.path.join(
        get_package_share_directory('dsr_description2'),
        'xacro',
        f"{model}.urdf.xacro"
    )

    urdf_xml = subprocess.check_output(
        ['xacro', xacro_file, f'color:={color}', f'gripper:={gripper}']
    ).decode('utf-8')

    robot_model = URDF.from_xml_string(urdf_xml)

    def is_active_joint(j):
        if j.type == "fixed":
            return False
        if getattr(j, "mimic", None) is not None:
            return False
        return j.name.startswith("joint_")

    active_joints = [j.name for j in robot_model.joints if is_active_joint(j)]

    passive_joints = sorted({
        j.name for j in robot_model.joints
        if (j.type == "fixed" or getattr(j, "mimic", None) is not None) and j.name.startswith("joint_")
    })

    return urdf_xml, active_joints, passive_joints
