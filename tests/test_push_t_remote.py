"""Remote planning/streaming and physical execution gates, without any hardware."""

import copy
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from fakes import Bus, Calibration

from lerobot_arm_control.tasks.push_t.push_t_motor import (
    PushMotor,
    joint_segments,
    observation_issue,
    recovery_issue,
    refresh_camera_observations,
    same_scene,
)
from lerobot_arm_control.transport.tabletop_stream import DisplayStream

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Push:
    start: object
    direction: object
    distance: float
    joints: list
    approach_count: int
    transition: str = "low transfer"

@pytest.fixture
def observed():
    return {'units': 'm', 'marker_visible': True, 'capture_age_at_send_ms': 20,
            'objects': {name: {'xy': [x, .2], 'yaw': 0, 'observed': True,
                               'pose_valid': True, 'tracking_status': 'observed'}
                        for name, x in (('wood_t', -.4), ('black_t', -.3))},
            'robot': {'status': 'live', 'qpos': [.3]*6, 'sample_age_at_send_ms': 20,
                      'registration': {'position': [-.4, 0, .0024],
                                       'quaternion': [1, 0, 0, 0], 'pose_age_s': 0,
                                       'calibrated': False, 'status': 'visual_estimate',
                                       'pose_source': 'cad_silhouette_and_motor_angles'}}}


@pytest.fixture
def motor(observed):
    from lerobot_arm_control.transport.realtime import JOINTS

    clock = [0.]
    bus = Bus()
    instance = PushMotor(bus, Calibration(), [[-1, 1]]*6,
                         {j: [1000, 3000] for j in JOINTS}, clock=lambda: clock[0])
    observed['robot']['qpos'] = [.3]*6
    observed['robot']['registration'].update(calibrated=True, status='tracked',
                                            pose_source='known_marker_mounts')
    instance.observe(observed, 0)
    action = Push(np.zeros(2), np.ones(2), .002, joints=[[.32]*6], approach_count=0)
    segments = joint_segments([.3]*6, action, np.array([[-1, 1]]*6))
    return instance, bus, clock, observed, segments


def tick(instance, bus):
    telemetry = {k: bus.sync_read(k) for k in ('Status', 'Torque_Enable')}
    instance.tick(bus.sync_read('Present_Position'), telemetry)


def test_unknown_marker_mounts_cannot_enable_any_motor(motor):
    instance, bus, _, state, segments = motor
    state['robot']['registration'].update(calibrated=False, pose_source='visual_estimate')
    instance.submit(state, segments)
    tick(instance, bus)
    assert not bus.writes
    assert instance.status()['phase'] == 'stopped'
    assert 'marker-to-base' in instance.status()['reason']


def test_preview_constructs_no_torque_or_goal_writes(motor):
    instance, bus, _, _, _ = motor
    tick(instance, bus)
    instance.close()
    assert bus.writes == []


def test_secondary_table_reference_never_refreshes_an_old_primary_object(observed):
    observed['capture_age_at_send_ms'] = 600
    observed['control_table_observation'] = {
        'source': 'side', 'captured_unix_ns': 1_000_000_000,
        'capture_age_at_send_ms': 50, 'marker_visible': True}
    assert 'wood_t primary camera observation older' in observation_issue(observed, 0, 0)
    for obj in observed['objects'].values():
        obj.update(observation_source='side', observation_age_s=.05)
    assert observation_issue(observed, 0, 0) is None
    assert 'table localization lost' in observation_issue(observed, 0, .451)


def test_secondary_evidence_can_take_over_when_primary_expires_between_publications(observed):
    observed.update(captured_unix_ns=1_000_000_000, capture_age_at_send_ms=420)
    observed['control_table_observation'] = {'marker_visible': True,
                                            'capture_age_at_send_ms': 200}
    for obj in observed['objects'].values():
        obj['camera_observations'] = {'side': {'xy': [obj['xy'][0]+.002, .2], 'yaw': 0,
            'captured_unix_ns': 1_220_000_000, 'tracking_status': 'observed', 'observed': True}}
    before = copy.deepcopy(observed)
    refreshed = refresh_camera_observations(observed, 0, .1)
    assert refreshed['objects']['wood_t']['observation_source'] == 'side'
    assert refreshed['objects']['wood_t']['observation_age_s'] == pytest.approx(.2)
    assert refreshed['objects']['wood_t']['xy'] == [-.398, .2]
    assert observation_issue(observed, 0, .1) is None
    assert observed == before
    assert observation_issue(observed, 0, .31) is not None


def test_controller_fault_reason_is_preserved_in_remote_push_status(motor, monkeypatch):
    instance, bus, _, state, segments = motor
    reason = 'shoulder_lift: loaded motion stall: hold'
    monkeypatch.setattr(instance.control, 'step', lambda *args: instance.control.stop(reason))
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'stopped'
    assert instance.status()['reason'] == reason
    assert set(instance.control.stopped.values()) == {reason}


def test_exhausted_enable_readbacks_hold_without_killing_feedback_or_retrying_motion(motor):
    instance, bus, _, state, segments = motor
    original = bus.read

    def read(register, joint, normalize=False, num_retry=0):
        if register == 'Acceleration':
            raise RuntimeError('Acceleration: There is no status packet!')
        return original(register, joint, normalize, num_retry)

    bus.read = read
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'stopped'
    assert 'motor enable failed: Acceleration:' in instance.status()['reason']
    assert instance.job is None and instance.pending is None
    assert bus.sync_read('Goal_Position') == bus.sync_read('Present_Position')
    before = list(bus.writes)
    tick(instance, bus)
    assert bus.writes == before


@pytest.mark.parametrize('valid', [True, False])
def test_motor_preflight_accepts_only_valid_saved_fixed_base(motor, valid):
    instance, bus, _, state, segments = motor
    state['robot']['registration'].update(status='fixed', calibrated=True,
                                        fixed_to_table=valid,
                                        pose_source='fixed_marker_calibration')
    instance.observe(state, 0)
    instance.submit(state, segments)
    tick(instance, bus)
    assert bool(bus.writes) is valid
    assert instance.status()['phase'] == ('running' if valid else 'stopped')


def test_physical_stream_reaches_waypoint_and_holds_without_releasing_torque(motor):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'running'
    assert all(r['Torque_Enable'] for r in bus.registers.values())
    for index in range(1, 31):
        clock[0] = index*.02
        for r in bus.registers.values():
            r['Present_Position'] = r['Goal_Position']
        state['robot']['qpos'] = [instance.control.angle(j, r['Present_Position'])
                                 for j, r in bus.registers.items()]
        instance.observe(state, clock[0])
        tick(instance, bus)
    assert instance.status()['phase'] == 'completed'
    assert all(r['Torque_Enable'] for r in bus.registers.values())
    assert not any(register in ('EEPROM_Lock', 'Operating_Mode', 'Homing_Offset')
                   for register, _, _ in bus.writes)
    instance.close()
    assert all(r['Acceleration'] == 0 for r in bus.registers.values())


def test_pre_enable_telemetry_cannot_falsely_trip_the_newly_enabled_stream(motor):
    instance, bus, clock, state, segments = motor
    before_enable = {key: bus.sync_read(key) for key in ('Status', 'Torque_Enable')}
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'running'
    clock[0] = .02
    instance.observe(state, clock[0])
    instance.tick(bus.sync_read('Present_Position'), before_enable)
    assert instance.status()['phase'] == 'running'
    assert all(bus.sync_read('Torque_Enable').values())


def test_completed_pose_keeps_preload_between_segments_and_on_clean_close(motor, monkeypatch):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments)
    tick(instance, bus)
    for values in bus.registers.values():
        values['Present_Position'] = 2210  # Small persistent gravity-loaded error.
    for index in range(1, 61):
        clock[0] = index / 60
        instance.observe(state, clock[0])
        tick(instance, bus)
    assert instance.status()['phase'] == 'completed'
    nominal = dict(instance.control.targets)
    correction = dict(instance.control.position_feedback.correction)
    assert abs(correction['elbow_flex']) > .0001
    clock[0] += .02
    tick(instance, bus)
    assert instance.status()['holding_closed_loop']
    assert instance.control.targets == nominal
    assert abs(instance.control.position_feedback.correction['elbow_flex']) > .0001

    def unexpected_enable():
        pytest.fail('next segment must not reset the supported equilibrium')

    monkeypatch.setattr(instance.control, 'enable_all', unexpected_enable)
    # Current measured pose is within the next segment's initial-pose tolerance.
    instance.observe(state, clock[0])
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'running'
    clock[0] += 1
    instance.observe(state, clock[0])
    tick(instance, bus)
    assert instance.status()['phase'] == 'completed'
    goals = bus.sync_read('Goal_Position')
    instance.close()
    assert bus.sync_read('Goal_Position') == goals
    assert all(bus.sync_read('Torque_Enable').values())


def test_fault_while_holding_completed_pose_still_latches_stop(motor):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments)
    tick(instance, bus)
    for values in bus.registers.values():
        values['Present_Position'] = 2210
    clock[0] = 1
    instance.observe(state, clock[0])
    tick(instance, bus)
    assert instance.status()['phase'] == 'completed'
    bus.registers['shoulder_lift']['Torque_Enable'] = 0
    clock[0] += .02
    tick(instance, bus)
    assert instance.status()['phase'] == 'stopped'
    assert 'lost torque' in instance.status()['reason']


@pytest.mark.parametrize('movement,baseline', [(0, True), (.004, True), (0, False)])
def test_camera_switch_uses_same_camera_target_baseline_without_widening_limit(
        motor, movement, baseline):
    instance, bus, clock, state, segments = motor
    target = state['objects']['black_t']
    if baseline:
        target['camera_observations'] = {'side': {'xy': [target['xy'][0]+.004,
                                                         target['xy'][1]], 'yaw': 0}}
    reference = copy.deepcopy(state)
    instance.submit(reference, segments)
    tick(instance, bus)
    target.update(observation_source='side', observation_age_s=0)
    target['xy'][0] += .004+movement
    instance.observe(state, .02)
    clock[0] = .02
    tick(instance, bus)
    assert (instance.status()['phase'] == 'running') is (baseline and movement == 0)
    assert same_scene(reference, state) is (baseline and movement == 0)
    if movement:
        assert instance.status()['reason'] == 'target moved during push'


def test_push_uses_shared_compensation_and_restores_native_ram_after_hold(observed):
    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.transport.realtime import JOINTS

    bus = Bus()
    clock = [0.]
    limits = {joint: [1000, 3000] for joint in JOINTS}
    instance = PushMotor(bus, Calibration(), [[-1, 1]]*6, limits,
                         clock=lambda: clock[0])
    circle_control = ManualControl(Bus(), Calibration(), limits)
    assert instance.control.position_feedback.profile == circle_control.position_feedback.profile
    observed['robot']['registration'].update(status='fixed', calibrated=True,
                                            fixed_to_table=True,
                                            pose_source='fixed_marker_calibration')
    instance.observe(observed, 0)
    action = Push(np.zeros(2), np.ones(2), .004, joints=[[.35]*6], approach_count=0)
    instance.submit(observed, joint_segments([.3]*6, action, np.array([[-1., 1.]]*6)))
    tick(instance, bus)
    stream = instance.control.position_feedback.profile['native_stream']
    expected_acceleration = stream['acceleration']
    assert set(bus.sync_read('Acceleration').values()) == {expected_acceleration}
    assert set(bus.sync_read('Goal_Velocity').values()) == {1000}
    for index in range(1, 61):
        clock[0] = index/60
        instance.observe(observed, clock[0])
        tick(instance, bus)
    assert instance.control.command_angles['elbow_flex'] > instance.control.targets['elbow_flex']
    assert set(bus.sync_read('P_Coefficient').values()) == {16}
    assert set(bus.sync_read('Lock').values()) == {1}
    assert not any(register in ('Lock', 'D_Coefficient',
                                'Operating_Mode', 'Homing_Offset', 'EEPROM_Lock')
                   for register, _, _ in bus.writes)
    assert sorted((j, v) for r, j, v in bus.writes if r == 'I_Coefficient') == [
        ('elbow_flex', 0), ('shoulder_lift', 0)]
    instance.close()
    assert set(bus.sync_read('Acceleration').values()) == {0}
    assert set(bus.sync_read('Goal_Velocity').values()) == {0}
    assert set(bus.sync_read('Torque_Enable').values()) == {1}
    assert set(bus.sync_read('P_Coefficient').values()) == {16}
    assert not any(instance.control.position_feedback.correction.values())


def test_push_pid_also_respects_the_cad_joint_envelope():
    from lerobot_arm_control.transport.realtime import JOINTS

    instance = PushMotor(Bus(), Calibration(), [[-.31, .31]]*6,
                         {j: [1000, 3000] for j in JOINTS})
    feedback = instance.control.position_feedback
    for _ in range(120):
        command = feedback.target('elbow_flex', .309, .25, 1/60)
        assert -.31 <= command <= .31


@pytest.mark.parametrize('failure', ['stale_image', 'lost_torque', 'tracking_error', 'operator'])
def test_fault_cancels_motion_and_holds(motor, failure):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments)
    tick(instance, bus)
    clock[0] = .1
    if failure == 'stale_image':
        clock[0] = .6
    elif failure == 'lost_torque':
        bus.registers['gripper']['Torque_Enable'] = 0
    elif failure == 'tracking_error':
        bus.registers['elbow_flex']['Present_Position'] = 2600
    else:
        instance.cancel('operator hold')
    tick(instance, bus)
    assert instance.status()['phase'] == 'stopped'
    assert instance.job is None


def test_visibility_recovery_allows_missing_visual_poses_but_not_stale_camera(motor):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments, recovery=True)
    hidden = copy.deepcopy(state)
    hidden['marker_visible'] = False
    hidden['objects'] = {}
    hidden['robot']['registration'] = {'status': 'mount_calibration_missing'}
    instance.observe(hidden, 0)
    assert observation_issue(hidden, 0, 0, require_base=True)
    assert recovery_issue(hidden, 0, 0) is None
    tick(instance, bus)
    assert instance.status()['phase'] == 'running'
    assert instance.status()['recovery']
    clock[0] = .6
    tick(instance, bus)
    assert instance.status()['phase'] == 'stopped'
    assert 'camera stream stale' in instance.status()['reason']


def test_cached_target_is_usable_after_initialization_but_held_wood_is_not(observed):
    observed['objects']['black_t'].update(observed=False, tracking_status='static_cached')
    assert observation_issue(observed, 0, 0)
    assert observation_issue(observed, 0, 0, initialized=True) is None
    observed['objects']['wood_t'].update(observed=False, tracking_status='held')
    assert 'wood_t' in observation_issue(observed, 0, 0, initialized=True)


def test_plans_become_invalid_when_the_scene_moves(observed):
    assert same_scene(observed, copy.deepcopy(observed))
    moved = copy.deepcopy(observed)
    moved['objects']['wood_t']['xy'][0] += .01
    assert not same_scene(observed, moved)
    moved = copy.deepcopy(observed)
    moved['robot']['registration']['position'][0] += .005
    assert not same_scene(observed, moved)


def test_circle_quintic_entry_and_bounded_push_speed_reject_out_of_limit_paths():
    limits = np.array([[-1., 1.]]*6)
    action = Push(np.zeros(2), np.ones(2), .004, joints=[[.1]*6, [.2]*6], approach_count=1)
    segments = joint_segments([0.]*6, action, limits)
    assert all(1.875*max(abs(a-b) for a, b in zip(s['start'], s['end']))/s['seconds']
               <= s['speed_rad_s']+1e-9 for s in segments)
    assert segments[0]['speed_rad_s'] == .8 and segments[0]['seconds'] >= .35
    assert segments[1]['speed_rad_s'] == .15
    slow = joint_segments([0.]*6, action, limits, entry_speed=.2)
    assert slow[0]['speed_rad_s'] == .2
    assert slow[0]['seconds'] >= 1.875*.1/.2
    assert slow[1]['seconds'] == segments[1]['seconds']
    with pytest.raises(ValueError, match='shared trajectory limit'):
        joint_segments([0.]*6, action, limits, entry_speed=.9)
    action.joints[-1][0] = 2
    with pytest.raises(ValueError, match='joint limits'):
        joint_segments([0.]*6, action, limits)


def test_slow_display_cannot_block_observation_and_retains_textures():
    stopped, entered, release = threading.Event(), threading.Event(), threading.Event()
    sent = []

    def write(value):
        entered.set()
        release.wait(1)
        sent.append(json.loads(value))

    stream = DisplayStream(30, stopped, write=write)
    packet = {'sequence': 1, 'appearance_pngs': {'tabletop.png': 'texture'},
              'motor_samples': [{'seq': 1}], 'capture_age_at_send_ms': 20,
              'stream_queued_unix_ns': time.time_ns()}
    stream.offer(packet)
    assert entered.wait(1)
    started = time.monotonic()
    for index in range(2, 102):
        stream.offer({**packet, 'sequence': index, 'motor_samples': [{'seq': index}]})
    assert time.monotonic()-started < .1
    release.set()
    deadline = time.monotonic()+1
    while len(sent) < 2 and time.monotonic() < deadline:
        time.sleep(.01)
    stopped.set()
    stream.changed.set()
    stream.worker.join(1)
    assert sent[-1]['sequence'] == 101
    assert sent[-1]['appearance_pngs']['tabletop.png'] == 'texture'
    assert len(sent[-1]['motor_samples']) == 100



def test_saved_fixed_base_does_not_expire_but_feedback_and_camera_still_do(observed):
    base = observed['robot']['registration']
    base.update(status='fixed', calibrated=True, fixed_to_table=True,
                pose_source='fixed_marker_calibration', calibration_age_s=86400, pose_age_s=86400)
    assert observation_issue(observed, 0, .1, require_base=True) is None
    base['fixed_to_table'] = False
    assert observation_issue(observed, 0, .1, require_base=True)
    base['fixed_to_table'] = True
    observed['robot']['sample_age_at_send_ms'] = 600
    assert observation_issue(observed, 0, .1, require_base=True)


def test_out_of_range_waypoint_is_rejected_without_killing_feedback_owner(motor):
    instance, bus, clock, state, segments = motor
    invalid = copy.deepcopy(segments)
    invalid[0]['end'][3] = 10.
    instance.submit(state, invalid)
    tick(instance, bus)
    assert instance.status()['phase'] == 'stopped'
    assert 'wrist_flex: motor waypoint outside encoder limits' == instance.status()['reason']
    assert bus.writes == []
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'running'


def test_new_target_camera_pauses_path_without_losing_support_then_replans(motor, monkeypatch):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments)
    tick(instance, bus)
    instance.control.position_feedback.integral['shoulder_lift'] = .04
    instance.control.position_feedback.correction['shoulder_lift'] = .04
    targets = dict(instance.control.targets)
    state['objects']['black_t'].update(observation_source='side', observation_age_s=0)
    clock[0] = .02
    instance.observe(state, clock[0])
    tick(instance, bus)
    assert instance.status()['phase'] == 'paused'
    assert instance.status()['holding_closed_loop']
    assert instance.job is None
    assert instance.control.targets == targets
    assert instance.control.position_feedback.integral['shoulder_lift'] == pytest.approx(.04)
    assert not instance.control.stopped
    clock[0] += .02
    tick(instance, bus)
    assert instance.control.targets == targets
    assert instance.control.position_feedback.integral['shoulder_lift'] == pytest.approx(.04)

    def unexpected_enable():
        pytest.fail('replanning must reuse the supported controller')

    monkeypatch.setattr(instance.control, 'enable_all', unexpected_enable)
    instance.observe(state, clock[0])
    instance.submit(state, segments)
    tick(instance, bus)
    assert instance.status()['phase'] == 'running'


@pytest.mark.parametrize('fault', [False, True])
def test_paused_supported_hold_preserves_goals_on_close_but_still_stops_for_fault(motor, fault):
    instance, bus, clock, state, segments = motor
    instance.submit(state, segments)
    tick(instance, bus)
    instance.control.position_feedback.integral['shoulder_lift'] = .04
    instance.control.position_feedback.correction['shoulder_lift'] = .04
    state['objects']['black_t'].update(observation_source='side', observation_age_s=0)
    clock[0] = .02
    instance.observe(state, clock[0])
    tick(instance, bus)
    assert instance.status()['phase'] == 'paused'
    if fault:
        bus.registers['shoulder_lift']['Torque_Enable'] = 0
        clock[0] += .02
        tick(instance, bus)
        assert instance.status()['phase'] == 'stopped'
        assert instance.control.stopped
    else:
        goals = bus.sync_read('Goal_Position')
        instance.close()
        assert bus.sync_read('Goal_Position') == goals
        assert all(bus.sync_read('Torque_Enable').values())
