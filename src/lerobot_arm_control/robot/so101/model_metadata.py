"""SO101 model coordinates, without loading a physics engine."""

import math

from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.robot.model import RobotModel
from lerobot_arm_control.robot.so101.joints import JOINTS

MODEL = resource_path('assets/so101/so101.xml')


def display_model(end_effector='gripper'):
    """Choose geometry without changing the canonical encoder/base reference model."""
    if end_effector not in ('gripper', 'leader_handle'):
        raise ValueError('unknown SO101 end effector')
    return (resource_path('assets/so101/so101_leader.xml')
            if end_effector == 'leader_handle' else MODEL)


def joint_references(config, *, calibration_sha256=None):
    """Physical angle minus CAD angle; never change encoder feedback or calibration."""
    simulation = config.get('simulation', {})
    references = simulation.get('joint_reference_degrees', {})
    if not isinstance(references, dict) or set(references)-set(JOINTS):
        raise ValueError('unknown model joint reference')
    if references:
        digest = simulation.get('joint_reference_calibration_sha256')
        if (not isinstance(digest, str) or len(digest) != 64
                or (calibration_sha256 is not None and digest != calibration_sha256)):
            raise ValueError('model joint reference requires matching joint calibration')
    values = [references.get(joint, 0.) for joint in JOINTS]
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError('model joint references must be finite degrees')
    return [math.radians(value) for value in values]


def entry_tolerance(config):
    """Small measured-start allowance; commanded waypoints remain strictly bounded."""
    degrees = config.get('push_t', {}).get('entry_tolerance_degrees', 0.)
    if type(degrees) not in (int, float) or not math.isfinite(degrees) or not 0 <= degrees <= 5:
        raise ValueError('Push-T entry tolerance must be between 0 and 5 degrees')
    return math.radians(degrees)


def joint_ranges(path=MODEL, *, references=None):
    offsets = references if references is not None else [0.]*len(JOINTS)
    return [[low+offset, high+offset] for (low, high), offset in zip(
        RobotModel.from_mjcf(path, expected_joints=JOINTS).ranges, offsets, strict=True)]
