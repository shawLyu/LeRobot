"""Read the canonical SO101 joint limits without loading a physics engine."""

import math
import xml.etree.ElementTree as ET

from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.robot.so101.joints import JOINTS

MODEL = resource_path('assets/so101/so101.xml')


def joint_ranges(path=MODEL):
    root = ET.parse(path).getroot()
    if root.find("compiler").get("angle") != "radian":
        raise ValueError("SO101 joint limits must be explicit radians")
    joints = root.findall("./worldbody//joint")
    if tuple(joint.get("name") for joint in joints) != JOINTS:
        raise ValueError("unexpected SO101 joint map")
    ranges = [list(map(float, joint.get("range", "").split())) for joint in joints]
    if any(len(bounds) != 2 or not all(math.isfinite(v) for v in bounds)
           or bounds[0] >= bounds[1] for bounds in ranges):
        raise ValueError("invalid SO101 joint limits")
    return ranges
