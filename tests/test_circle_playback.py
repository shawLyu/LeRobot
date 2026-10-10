"""The remote circle player requires a local validation result before any motor writes."""

import copy

import pytest
from test_manual_control import Bus, Calibration

from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
from lerobot_arm_control.transport.realtime import JOINTS


@pytest.fixture
def playback():
    bus = Bus()
    control = ManualControl(bus, Calibration(), [[-1, 1]] * 6,
                            {joint: [1000, 3000] for joint in JOINTS})
    circle = Circle()
    for joint, angle in zip(JOINTS, circle.points[0]):
        bus.registers[joint]['Present_Position'] = control.raw(joint, angle)
    initial = [control.angle(joint, bus.registers[joint]['Present_Position']) for joint in JOINTS]
    validation = {'checked': True, 'initial_qpos': initial, 'model_sha256': circle.model_hash,
                  'plan_sha256': circle.plan_hash, 'control_config_sha256': circle.profile_hash}
    return bus, control, circle, validation


@pytest.mark.parametrize('field', ['checked', 'model_sha256', 'plan_sha256',
                                  'control_config_sha256', 'initial_qpos'])
def test_invalid_local_validation_never_enables_motors(playback, field):
    bus, control, circle, validation = playback
    validation[field] = [float('nan')]*6 if field == 'initial_qpos' else False
    with pytest.raises(ValueError):
        circle.start_hardware(control, validation)
    assert bus.writes == []


def test_moved_entry_pose_requires_new_local_validation(playback):
    bus, control, circle, validation = playback
    bus.registers['elbow_flex']['Present_Position'] += 4
    with pytest.raises(ValueError, match='arm moved'):
        circle.start_hardware(control, validation)
    assert bus.writes == []


def test_matching_local_validation_starts_pure_playback_without_mutating_proof(playback):
    bus, control, circle, validation = playback
    original = copy.deepcopy(validation)
    circle.start_hardware(control, validation, laps=1)
    assert circle.phase == 'entering'
    assert circle.laps == 1
    assert circle.period == 4
    assert control.enabled == set(JOINTS)
    assert validation == original
    control.close()
