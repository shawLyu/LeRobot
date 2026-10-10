"""Reject unsafe or uninspected multi-joint pose acquisition plans before writes."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from lerobot_arm_control.calibration.marker_sampling import ACTIVE, close_workers, validate_plan
from lerobot_arm_control.transport.realtime import JOINTS


def plan():
    raw = dict.fromkeys(JOINTS, 1000)
    poses = [{'name': 'start', 'raw': {j: raw[j] for j in ACTIVE}}]
    poses.append({'name': 'lift', 'raw': {**poses[0]['raw'], 'shoulder_lift': 1032}})
    return {'expected_initial_raw': raw, 'poses': poses}


def test_finite_single_joint_scan_passes():
    p = plan()
    assert validate_plan(p, p['expected_initial_raw'], dict.fromkeys(JOINTS, [0, 4095]))


def test_failed_motor_cleanup_still_closes_camera_and_reports_failure():
    motor = Mock(process=SimpleNamespace(returncode=0))
    camera = Mock(process=SimpleNamespace(returncode=0))
    motor.close.side_effect = RuntimeError('restore failed')
    assert close_workers(motor, camera) == 'motors: restore failed'
    camera.close.assert_called_once()
    motor.close.side_effect = None
    motor.process.returncode = 1
    assert close_workers(motor, camera) == 'motors: worker exit 1'


@pytest.mark.parametrize('change', ['start', 'large', 'coupled', 'jaw', 'limit', 'float'])
def test_bad_plan_is_rejected(change):
    p = plan()
    initial = deepcopy(p['expected_initial_raw'])
    limits = dict.fromkeys(JOINTS, [0, 4095])
    target = p['poses'][1]['raw']
    if change == 'start':
        initial['wrist_flex'] += 5
    elif change == 'large':
        target['shoulder_lift'] += 1
    elif change == 'coupled':
        target['elbow_flex'] += 1
    elif change == 'jaw':
        target['gripper'] = 1001
    elif change == 'limit':
        limits['shoulder_lift'] = [1001, 4095]
    else:
        target['shoulder_lift'] = 1032.
    with pytest.raises(ValueError):
        validate_plan(p, initial, limits)
