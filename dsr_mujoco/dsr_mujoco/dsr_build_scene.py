import xml.etree.ElementTree as ET
from pathlib import Path
import shutil

# Build new MJCF(XML) scene file, check scene file and add new tag.
def build_scene(original_scene_xml_path: Path,
                robot_model_to_include: Path,
                output_dir: Path,
                arm_model_filename_in_original_scene: str) -> Path:
    
    if not original_scene_xml_path.exists():
        raise FileNotFoundError(f"Original scene XML not found: {original_scene_xml_path}")
    if not robot_model_to_include.exists():
        raise FileNotFoundError(f"Robot model to include not found: {robot_model_to_include}")

    output_dir.mkdir(parents=True, exist_ok=True)

    scene_root = ET.Element("mujoco", model=f"scene_with_{robot_model_to_include.stem}")
    
    ET.SubElement(scene_root, "include", file=robot_model_to_include.name)

    orig_scene_tree = ET.parse(original_scene_xml_path)
    orig_scene_root = orig_scene_tree.getroot()

    for child in list(orig_scene_root):
        # Skip the original model tag
        if child.tag == "include" and child.get("file") == arm_model_filename_in_original_scene:
            continue
        scene_root.append(child)

    new_scene_filename = f"generated_scene_for_{robot_model_to_include.stem}.xml"
    new_scene_path = output_dir / new_scene_filename
    
    scene_tree = ET.ElementTree(scene_root)
    scene_tree.write(new_scene_path, encoding="utf-8", xml_declaration=True)

    print(f"[build_scene] new scene generated at → {new_scene_path}")
    return new_scene_path