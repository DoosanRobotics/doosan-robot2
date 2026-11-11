import xml.etree.ElementTree as ET
from pathlib import Path
import tempfile
import copy

# Merge MuJoCo MJCF(XML) files
def merge_gripper(arm_xml: Path,
               hand_xml: Path,
               output_dir: Path,
               flange_body: str = "link_6") -> Path:

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

    # Ensure output_dir exists
    output_dir.mkdir(parents=True, exist_ok=True)

    merged_filename = f"{arm_xml.stem}_{hand_xml.stem}_merged.xml"
    merged_path = output_dir / merged_filename
 
    arm_tree.write(merged_path, encoding="utf-8", xml_declaration=True)

    print(f"[merge_gripper] merged robot XML generated at → {merged_path}")

    # Return path to the merged robot XML
    return merged_path
