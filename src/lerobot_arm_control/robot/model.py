"""Scalar-joint MJCF metadata, independent of a motor SDK or physics runtime."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RobotModel:
    path: Path
    joints: tuple[str, ...]
    ranges: tuple[tuple[float, float], ...]

    @classmethod
    def from_mjcf(cls, path, *, expected_joints=None):
        path = Path(path)
        root = ET.parse(path).getroot()
        compiler = root.find('compiler')
        if compiler is None or compiler.get('angle') != 'radian':
            raise ValueError('joint limits must be explicit radians')
        nodes = root.findall('./worldbody//joint')
        joints = tuple(node.get('name') for node in nodes)
        if (not joints or any(not name for name in joints) or len(set(joints)) != len(joints)
                or expected_joints is not None and joints != tuple(expected_joints)):
            raise ValueError('unexpected or invalid joint map')
        if any(node.get('type', 'hinge') != 'hinge' for node in nodes):
            raise ValueError('angular control metadata requires scalar hinge joints')
        ranges = tuple(tuple(map(float, node.get('range', '').split())) for node in nodes)
        if any(len(bounds) != 2 or not all(math.isfinite(v) for v in bounds)
               or bounds[0] >= bounds[1] for bounds in ranges):
            raise ValueError('invalid joint limits')
        return cls(path, joints, ranges)

    @property
    def limits(self):
        return dict(zip(self.joints, self.ranges, strict=True))
