"""The remote circle player requires a local validation result before any motor writes."""

import copy
import json

import pytest
from fakes import Bus, Calibration

from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
from lerobot_arm_control.transport.realtime import JOINTS


@pytest.fixture
def playback():
    bus = Bus()
    control = ManualControl(bus, Calibration(), {joint: [1000, 3000] for joint in JOINTS})
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


def test_uncleared_candidate_cannot_write_to_motors(playback, tmp_path):
    from lerobot_arm_control.tasks.circle.circle_trajectory import PLAN

    saved = json.loads(PLAN.read_text())
    saved['execution_ready'] = False
    path = tmp_path/'candidate.json'
    path.write_text(json.dumps(saved))
    bus, control, _, validation = playback
    circle = Circle(path)
    with pytest.raises(ValueError, match='not been cleared'):
        circle.start_hardware(control, validation, laps=1)
    assert bus.writes == []


def fast_plan(tmp_path, speed=60):
    from lerobot_arm_control.core.paths import resource_path

    saved = json.loads(resource_path('data/trajectories/table-circle-8cm.json').read_text())
    saved['period_seconds'] = 360/speed
    saved['execution_profile'].update(angular_speed_deg_s=speed, ramp_seconds=2)
    path = tmp_path/'fast-circle.json'
    path.write_text(json.dumps(saved))
    return path


def test_explicit_sixty_degree_circle_keeps_six_second_period_and_checks_ramps(tmp_path):
    circle = Circle(fast_plan(tmp_path))
    assert circle.period == 6 and circle.status()['angular_speed_deg_s'] == 60
    result = circle.check_motion_budget(circle.points[0])
    assert result['peak_joint_speed_rad_s'] < circle.motion_budget['max_joint_speed_rad_s']
    assert result['peak_control_frame_acceleration_rad_s2'] < 3.49
    circle.start(circle.points[0], 0, laps=3)
    end = circle.entry_seconds+3*circle.period+circle.ramp_seconds
    assert circle.sample(end) == pytest.approx(circle.points[0], abs=1e-10)
    assert circle.phase == 'completed'


def test_faster_than_reviewed_circle_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='angular speed'):
        Circle(fast_plan(tmp_path, 91))


def test_ninety_degree_circle_retains_six_second_ramps_and_rejects_overlap(tmp_path):
    path = fast_plan(tmp_path, 90)
    saved = json.loads(path.read_text())
    saved['execution_profile']['ramp_seconds'] = 6
    path.write_text(json.dumps(saved))
    circle = Circle(path)
    assert circle.period == 4 and circle.ramp_seconds == 6
    with pytest.raises(ValueError, match='overlapping'):
        circle.start(circle.points[0], 0, laps=1)
    circle.start(circle.points[0], 0, laps=3)
    # At 6 s acceleration ends at 0.75 lap; braking begins at 12 s, at 2.25 laps.
    assert circle.cycles(6) == pytest.approx(.75)
    assert circle.cycles(12) == pytest.approx(2.25)
    end = circle.entry_seconds+18
    assert circle.sample(end) == pytest.approx(circle.points[0], abs=1e-9)
    assert circle.phase == 'completed'
    assert circle.check_motion_budget(circle.points[0])['peak_joint_speed_rad_s'] < 1.571


def test_fast_plan_explicitly_selects_shared_controller_with_circle_tuning(tmp_path):
    from lerobot_arm_control.control.control_defaults import (
        CIRCLE_POSITION_FEEDBACK,
        POSITION_FEEDBACK,
    )

    circle = Circle(fast_plan(tmp_path))
    assert circle.feedback_profile == CIRCLE_POSITION_FEEDBACK
    assert circle.feedback_profile['native_stream']['acceleration'] == 16
    assert Circle().feedback_profile == POSITION_FEEDBACK
    circle.feedback_profile['joints']['elbow_flex']['kp'] = 0
    assert CIRCLE_POSITION_FEEDBACK['joints']['elbow_flex']['kp'] == .08
    path = fast_plan(tmp_path)
    saved = json.loads(path.read_text())
    saved['execution_profile']['controller_profile'] = 'unreviewed'
    path.write_text(json.dumps(saved))
    with pytest.raises(ValueError, match='unknown circle controller'):
        Circle(path)


def test_ninety_degree_profile_has_adequate_acceleration_without_changing_sixty():
    from lerobot_arm_control.control.control_defaults import CIRCLE_POSITION_FEEDBACK
    from lerobot_arm_control.core.paths import resource_path

    for height in (34, 36, 38):
        circle = Circle(resource_path(f'data/trajectories/table-circle90-h{height}cm.json'))
        assert circle.feedback_profile['name'] == 'smooth-circle90-v2'
        assert circle.feedback_profile['native_stream']['acceleration'] == 32
        assert circle.feedback_profile['native_stream']['transition_acceleration'] == 16
        assert circle.period == 4 and circle.ramp_seconds == 6
    assert CIRCLE_POSITION_FEEDBACK['native_stream']['acceleration'] == 16


def test_acceleration_schedule_follows_the_same_continuous_speed_ramps():
    from lerobot_arm_control.core.paths import resource_path

    circle = Circle(resource_path('data/trajectories/table-circle90-h38cm.json'))
    circle.start(circle.points[0], 0, laps=3)
    start = circle.entry_seconds
    for elapsed, expected in [(0, 0), (3, .5), (6, 1), (12, 1), (15, .5), (18, 0)]:
        assert circle.speed_fraction(start+elapsed) == pytest.approx(expected)
    assert circle.speed_fraction(start/2) == 0


def test_fast_circle_remote_budget_is_rechecked_before_any_motor_write(tmp_path):
    from lerobot_arm_control.core.paths import resource_path

    circle = Circle(fast_plan(tmp_path))
    circle.motion_budget['max_joint_acceleration_rad_s2'] = .05
    bus = Bus()
    calibration = Calibration()
    calibration.path = resource_path('data/calibration/follower_mujoco.json')
    control = ManualControl(bus, calibration, {j: [0, 4095] for j in JOINTS})
    validation = {'checked': True, 'initial_qpos': circle.points[0],
                  'model_sha256': circle.model_hash, 'plan_sha256': circle.plan_hash,
                  'control_config_sha256': circle.profile_hash}
    with pytest.raises(ValueError, match='speed/acceleration'):
        circle.start_hardware(control, validation, laps=1)
    assert not bus.writes


def test_fast_circle_rejects_the_default_controller_before_any_motor_write(tmp_path):
    from lerobot_arm_control.core.paths import resource_path

    circle = Circle(fast_plan(tmp_path))
    bus, calibration = Bus(), Calibration()
    calibration.path = resource_path('data/calibration/follower_mujoco.json')
    control = ManualControl(bus, calibration, {j: [0, 4095] for j in JOINTS})
    validation = {'checked': True, 'initial_qpos': circle.points[0],
                  'model_sha256': circle.model_hash, 'plan_sha256': circle.plan_hash,
                  'control_config_sha256': circle.profile_hash}
    with pytest.raises(ValueError, match='controller differs'):
        circle.start_hardware(control, validation, laps=1)
    assert not bus.writes


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
    assert circle.period == 16
    assert control.enabled == set(JOINTS)
    assert validation == original
    control.close()
