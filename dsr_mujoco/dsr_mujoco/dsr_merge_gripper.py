import xml.etree.ElementTree as ET
from pathlib import Path
import tempfile
import copy

# Merge Mujoco MJCF(XML) files
def merge_gripper(arm_xml: Path,
               hand_xml: Path,
               scene_xml: Path,
               flange_body: str = "link_6") -> tuple[Path, Path]:

    # Parse both XML files
    arm_tree  = ET.parse(arm_xml)
    tool_tree = ET.parse(hand_xml)
    arm_root  = arm_tree.getroot()
    tool_root = tool_tree.getroot()

    # Tag list to copy
    SECTIONS = ("asset", "default", "actuator", "tendon",
                "sensor", "equality", "contact", "option")
    for tag in SECTIONS:
        src = tool_root.find(tag)
        if src is None:
            continue

        dst = arm_root.find(tag)
        if dst is None:
            dst = ET.SubElement(arm_root, tag)

        # Copy all attributes
        for attr_name, attr_val in src.attrib.items():
            dst.set(attr_name, attr_val)

        # Copy all child elements
        for child in list(src):
            dst.append(copy.deepcopy(child))


    flange = arm_root.find(f".//body[@name='{flange_body}']")
    if flange is None:
        raise RuntimeError(f"Body {flange_body} not found in {arm_xml}")

    # Works whether or not tool XML has <worldbody>
    for body in (tool_root.find("worldbody") or tool_root).findall("body"):
        flange.append(body)

    merged_filename = f"{arm_xml.stem}_{hand_xml.stem}.xml"
    merged_path = arm_xml.parent / merged_filename
 
    arm_tree.write(merged_path, encoding="utf-8", xml_declaration=True)

    # Build new scene file
    scene_root = ET.Element("mujoco", model=f"{arm_xml.stem} merged scene")
    ET.SubElement(scene_root, "include", file=merged_path.name)

    orig_scene = ET.parse(scene_xml).getroot()
    for child in list(orig_scene):
        # Skip only <include file="{arm}.xml"/> tag
        if child.tag == "include" and child.get("file") == arm_xml.name:
            continue
        scene_root.append(child)

    scene_tree = ET.ElementTree(scene_root)
    scene_path = arm_xml.parent / "scene_merged.xml"
    scene_tree.write(scene_path, encoding="utf-8", xml_declaration=True)

    print(f"[merge_mjcf] merged robot → {merged_path}")
    print(f"[merge_mjcf] new scene    → {scene_path}")

    # Return both paths
    return merged_path, scene_path

