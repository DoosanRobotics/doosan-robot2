import yaml
import os
from ament_index_python.packages import get_package_share_directory

def read_update_rate():
    pkg_share = get_package_share_directory("dsr_controller2")
    yaml_path = os.path.join(pkg_share, "config", "dsr_controller2.yaml")
    with open(yaml_path, "r") as f:
        yaml_data = yaml.safe_load(f)
    try:
        update_rate = yaml_data["controller_manager"]["ros__parameters"]["update_rate"]
    except Exception:
        update_rate = 100  # fallback default
    print(f"[dsr_controller2] Loaded update_rate from YAML: {update_rate}")
    return update_rate