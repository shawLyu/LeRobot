"""Finite remote-clock paths require local proof before enabling hardware."""

import hashlib
from types import SimpleNamespace

import pytest
from fakes import Bus, Calibration

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.joint_path import ACTIVE, JointPath, plan_digest
from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.transport.realtime import JOINTS


@pytest.fixture
def playback(tmp_path):
    bus, calibration, now = Bus(), Calibration(), [0.]
    calibration.path = tmp_path/'angles.json'
    calibration.path.write_text('{}')
    calibration.fingerprint = {'mjcf_sha256': 'model'}
    control = ManualControl(bus, calibration, {j: [1000, 3000] for j in JOINTS},
                            clock=lambda: now[0])
    plan = {'version': 1, 'initial_raw': dict.fromkeys(JOINTS, 2200),
            'waypoints': [{'raw': dict.fromkeys(ACTIVE, 2400), 'dwell_seconds': 2}]}
    validation = {'checked': True, 'plan_sha256': plan_digest(plan),
                  'control_config_sha256': CONTROL_CONFIG_SHA256, 'model_sha256': 'model',
                  'joint_calibration_sha256': hashlib.sha256(b'{}').hexdigest()}
    return bus, control, plan, validation, now


@pytest.mark.parametrize('issue', ['proof', 'limit', 'envelope', 'moved', 'jaw', 'nan'])
def test_rejected_path_never_enables_or_writes(playback, issue):
    bus, control, plan, validation, _ = playback
    if issue == 'proof':
        validation['control_config_sha256'] = 'stale'
    elif issue == 'limit':
        plan['waypoints'][0]['raw']['elbow_flex'] = 2999
    elif issue == 'envelope':
        plan['waypoints'][0]['raw']['shoulder_pan'] = 2700
    elif issue == 'moved':
        bus.registers['shoulder_lift']['Present_Position'] += 5
    elif issue == 'jaw':
        plan['waypoints'][0]['raw']['gripper'] = 2400
    else:
        plan['waypoints'][0]['dwell_seconds'] = float('nan')
    if issue not in ('proof', 'nan'):
        validation['plan_sha256'] = plan_digest(plan)
    with pytest.raises(ValueError):
        JointPath().start(control, plan, validation)
    assert bus.writes == []


def test_worker_clock_drives_all_five_together_and_holds_without_enabling_jaw(playback):
    bus, control, plan, validation, now = playback
    path = JointPath()
    path.start(control, plan, validation)
    assert control.enabled == set(ACTIVE)
    assert all(j != 'gripper' for _, j, _ in bus.writes)
    now[0] = path.segments[0]['seconds']/2
    positions = dict.fromkeys(JOINTS, 2300)
    positions['gripper'] = 2200
    path.advance_hardware(positions, {})
    assert all(control.raw(j, control.targets[j]) == 2300 for j in ACTIVE)
    now[0] = path.segments[0]['seconds']+.1
    path.advance_hardware({**dict.fromkeys(ACTIVE, 2400), 'gripper': 2200}, {})
    assert path.phase == 'dwelling'
    now[0] += 2
    path.advance_hardware({**dict.fromkeys(ACTIVE, 2400), 'gripper': 2200}, {})
    assert path.phase == 'completed'
    assert control.enabled == set(ACTIVE)
    assert set(control.stopped) == set(ACTIVE)


def test_tracking_failure_holds_all_joints(playback):
    _, control, plan, validation, now = playback
    path = JointPath()
    path.start(control, plan, validation)
    positions = dict.fromkeys(JOINTS, 2200)
    now[0] = path.segments[0]['seconds']
    path.advance_hardware(positions, {})
    now[0] += .41
    path.advance_hardware(positions, {})
    assert path.phase.startswith('stopped: joint path tracking error')
    assert set(control.stopped) == set(ACTIVE)


def test_existing_automatic_motion_is_not_replaced(playback):
    bus, control, plan, validation, _ = playback
    control.trajectory = SimpleNamespace(active=True)
    with pytest.raises(ValueError, match='another automatic'):
        JointPath().start(control, plan, validation)
    assert bus.writes == []
