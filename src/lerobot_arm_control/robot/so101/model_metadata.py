"""Read the canonical SO101 joint limits without loading a physics engine."""

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


def joint_ranges(path=MODEL):
    return [list(bounds) for bounds in RobotModel.from_mjcf(
        path, expected_joints=JOINTS).ranges]
