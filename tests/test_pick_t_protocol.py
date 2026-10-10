"""Physical pick/place protocol guards with a substitute motor bus; no hardware."""

import copy
import hashlib
import math
from types import SimpleNamespace

import pytest
from test_push_t_remote import motor as motor  # noqa: F401
from test_push_t_remote import observed as observed  # noqa: F401

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.tasks.push_t.push_t_receiver import PushReceiver
from lerobot_arm_control.transport.pick_t_protocol import (
    pick_observation_issue,
    placement_error,
    segment_digest,
    stage_segments_issue,
)


@pytest.fixture
def pick_motor(motor):
    instance, bus, clock, state, _ = motor
    instance.task = 'pick_t'
    state['captured_unix_ns'] = 1_000_000_000
    state['robot']['registration'].update(status='fixed', fixed_to_table=True,
                                        pose_source='fixed_marker_calibration')
    return instance, bus, clock, state


def test_carrying_allows_hidden_wood_but_not_stale_camera_or_missing_fixed_base(pick_motor):
    _, _, _, state = pick_motor
    reference = copy.deepcopy(state)
    state['objects'].pop('wood_t')
    assert pick_observation_issue(state, 0, .1, 'transfer', reference) is None
    assert pick_observation_issue(state, 0, .6, 'transfer', reference)
    assert pick_observation_issue(state, 0, .1, 'descend', reference)
    state['robot']['registration']['calibrated'] = False
    assert pick_observation_issue(state, 0, .1, 'transfer', reference)


@pytest.mark.parametrize('bad', ['sequence', 'proof', 'arm_during_close', 'jaw_during_transfer'])
def test_invalid_pick_stages_never_write_to_motors(pick_motor, tmp_path, bad):
    instance, bus, _, state = pick_motor
    model_hash = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    endpoint = PushReceiver(SimpleNamespace(motor=instance, metadata={
        'fingerprint': {'mjcf_sha256': model_hash}}), tmp_path/'received',
        clock=lambda: 0., task='pick_t')
    endpoint.observe(state)
    stage = 'prepare'
    end = [.3]*5+[.32]
    if bad == 'sequence':
        stage = 'lift'
    elif bad == 'arm_during_close':
        endpoint.next_stage = 3
        stage, end = 'close', [.31]*5+[.28]
    elif bad == 'jaw_during_transfer':
        endpoint.next_stage = 5
        stage = 'transfer'
    segments = [{'start': [.3]*6, 'end': end, 'seconds': 1., 'speed_rad_s': .25}]
    validation = {'checked': True, 'segments_sha256': segment_digest(segments),
                  'model_sha256': model_hash, 'control_config_sha256': CONTROL_CONFIG_SHA256}
    if bad == 'proof':
        validation['segments_sha256'] = 'changed'
    command = {'version': 1, 'id': 1, 'op': 'pick_execute', 'stage': stage,
               'segments': segments, 'reference': state, 'validation': validation,
               'model_sha256': model_hash, 'control_config_sha256': CONTROL_CONFIG_SHA256}
    with pytest.raises(ValueError):
        endpoint.handle(command)
    assert bus.writes == []


@pytest.mark.parametrize('contact', [False, True])
def test_close_requires_contact_before_allowing_lift(pick_motor, contact):
    instance, bus, clock, state = pick_motor
    segments = [{'start': [.3]*6, 'end': [.3]*5+[.27],
                 'seconds': .5, 'speed_rad_s': .25}]
    instance.submit(state, segments, stage='close')
    for _ in range(160):
        clock[0] += 1/60
        for joint, values in bus.registers.items():
            if values['Torque_Enable']:
                values['Present_Position'] = (2193 if contact and joint == 'gripper'
                                              else values['Goal_Position'])
        instance.observe(state, clock[0])
        instance.tick(bus.sync_read('Present_Position'),
                      {key: bus.sync_read(key) for key in ('Status', 'Torque_Enable')})
        if instance.status()['phase'] in ('completed', 'stopped'):
            break
    assert instance.status()['phase'] == ('completed' if contact else 'stopped')
    if not contact:
        assert 'empty grasp' in instance.status()['reason']
    assert all(bus.sync_read('Torque_Enable').values())


def test_placement_verification_cannot_use_a_cached_or_invalid_pose(observed):
    target = {'xy': observed['objects']['wood_t']['xy'], 'yaw': 2*math.pi}
    assert placement_error(observed, target) == pytest.approx((0., 0.))
    observed['objects']['wood_t']['observed'] = False
    assert placement_error(observed, target) is None


def test_transfer_speed_is_separately_bounded():
    segments = [{'start': [.3]*6, 'end': [.4]*5+[.3],
                 'seconds': 1., 'speed_rad_s': .8}]
    assert stage_segments_issue('transfer', segments)


@pytest.mark.parametrize('stage', ['descend', 'close'])
def test_grasp_occlusion_has_a_two_second_limit(pick_motor, stage):
    _, _, _, state = pick_motor
    reference = copy.deepcopy(state)
    state['objects']['wood_t'].update(observed=False, tracking_status='held',
                                     observation_age_s=1.9)
    assert pick_observation_issue(state, 0, .1, stage, reference) is None
    state['objects']['wood_t']['observation_age_s'] = 2.1
    assert 'two seconds' in pick_observation_issue(state, 0, .1, stage, reference)


def test_rejected_mount_fit_is_explained_in_policy_preflight(pick_motor):
    _, _, _, state = pick_motor
    state['robot']['registration'].update(
        calibrated=False, status='pose_rejected', reprojection_error_px=2.4,
        max_reprojection_error_px=1.5, rejection_reason='known mount does not fit')
    assert '2.40 > 1.50' in pick_observation_issue(state, 0, .1, 'prepare')
