#!/usr/bin/env python3
import os
import subprocess
import pathlib
import yaml
from ament_index_python.packages import get_package_share_directory

# Find the Git root for the current package
def find_git_root_for_package(here: pathlib.Path):
    pkg_name = None

    # Determine package name from install/lib/<pkg>
    for parent in here.parents:
        if parent.name == "lib":
            pkg_name = parent.parent.name
            break

    if pkg_name is None:
        return None

    # Determine workspace root from "install/"
    ws_root = None
    for parent in here.parents:
        if parent.name == "install":
            ws_root = parent.parent
            break

    if ws_root is None:
        return None

    src_dir = ws_root / "src"
    if not src_dir.exists():
        return None

    # 1) Search any folder named <pkg_name>
    candidates = list(src_dir.rglob(pkg_name))
    if not candidates:
        return None

    pkg_src_path = candidates[0]  # choose the first match

    # 2) Find closest parent folder that contains a .git directory
    for parent in [pkg_src_path, *pkg_src_path.parents]:
        if (parent / ".git").exists():
            return parent

    return None


# Fallback: find any git repo inside src/
def find_any_git_in_src(ws_root: pathlib.Path):
    src_dir = ws_root / "src"
    if not src_dir.exists():
        return None

    for child in src_dir.rglob("*"):
        if (child / ".git").exists():
            return child
    return None


# Show Git information
def show_git_info():
    """
    Print Git commit / branch / user info for debugging.
    Supports both source and install execution.
    """
    here = pathlib.Path(__file__).resolve()
    git_root = find_git_root_for_package(here)

    # Fallback search if direct package search fails
    if git_root is None:
        for parent in here.parents:
            if parent.name == "install":
                ws_root = parent.parent
                git_root = find_any_git_in_src(ws_root)
                break

    if git_root is None:
        print("[Git Info] No .git directory found in workspace")
        return {
            "commit": "unknown",
            "branch": "unknown",
            "user": "unknown",
            "email": "unknown",
        }

    def run(cmd):
        try:
            return subprocess.check_output(cmd, cwd=git_root).decode().strip()
        except Exception:
            return "unknown"

    info = {
        "commit": run(["git", "rev-parse", "--short", "HEAD"]),
        "branch": run(["git", "rev-parse", "--abbrev-ref", "HEAD"]),
        "user": run(["git", "config", "user.name"]),
        "email": run(["git", "config", "user.email"]),
    }

    print(f"\n[Git Info] {info['user']} <{info['email']}> | "
          f"{info['branch']}@{info['commit']} (root={git_root})\n")

    return info

def read_update_rate():
    pkg_share = get_package_share_directory("dsr_controller2")
    yaml_path = os.path.join(pkg_share, "config", "dsr_controller2.yaml")

    if not os.path.exists(yaml_path):
        print(f"[dsr_controller2] YAML file not found: {yaml_path}")
        return 100

    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
            
        root = data
        if "/**" in root:
            root = root["/**"]

        update_rate = (
            root.get("controller_manager", {})
                .get("ros__parameters", {})
                .get("update_rate", 100)
        )

        print(f"[dsr_controller2] Loaded update_rate from YAML: {update_rate}")
        return update_rate

    except Exception as e:
        print(f"[dsr_controller2] Failed to read YAML: {e}")
        return 100
