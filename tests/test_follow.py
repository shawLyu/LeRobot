"""Verify bounded joint mirroring and the remote read-only/physical-control boundary."""

import math
from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from lerobot_arm_control.control.follow import (
    JointFollower,
    initialization_target,
    load_follow_config,
)
from lerobot_arm_control.runtime import follow_server
from lerobot_arm_control.transport.realtime import JOINTS


def test_mirror_slew_limit_joint_bounds_and_trigger_normalization():
    config = load_follow_config()
    config['max_speed_rad_s'] = 2
    mapping = JointFollower(config, [0]*6)
    for _ in range(80):
        target = mapping.step([10, 0, 0, 0, 0, math.radians(59)], .05)
    assert target[0] == pytest.approx(mapping.limits[0][1])
    assert target[5] == pytest.approx(math.radians(45))
    previous = list(target)
    target = mapping.step([-10]*6, .01)
    assert max(abs(a-b) for a, b in zip(target, previous, strict=True)) <= .02000001


@pytest.mark.parametrize('leader_degrees, follower_degrees',
                         [(-5, 0), (0, 0), (30, 90 * 30 / 118), (59, 45),
                          (90, 90 * 90 / 118), (118, 90), (125, 90)])
def test_trigger_uses_full_118_degree_travel_without_saturating_at_calibration_points(
        leader_degrees, follower_degrees):
    config = load_follow_config()
    assert config['leader_trigger_degrees'] == [0, 118]
    mapping = JointFollower(config, [0]*6)
    for _ in range(120):
        target = mapping.step([0]*5+[math.radians(leader_degrees)], .05)
    assert math.degrees(target[5]) == pytest.approx(follower_degrees)


def test_stale_samples_latch_hold_until_explicit_resume():
    mapping = JointFollower(load_follow_config(), [.1]*6)
    assert mapping.step([1]*6, .1, sample_age=1) == [.1]*6
    assert mapping.step([1]*6, .1) == [.1]*6
    mapping.resume([.2]*6)
    assert mapping.step([1]*6, .1)[0] > .2
    for pose in ([0]*5, [0]*5+[float('nan')], [True]*6):
        with pytest.raises(ValueError):
            mapping.step(pose, .1)


def test_initialization_recovers_small_model_violation_without_remapping_the_gripper():
    config = load_follow_config()
    measured = [0, math.radians(-100.85), 0, 0, 0, .3]
    limits = [(-2, 2)]*6
    limits[1] = tuple(map(math.radians, (-100, 100)))
    encoders = {joint: (-3, 3) for joint in JOINTS}
    goal = initialization_target(config, measured, limits, encoders)
    assert goal[1] == pytest.approx(math.radians(-99.5))
    assert goal[5] == measured[5]
    mapping = JointFollower(config, measured, limits)
    target = mapping.step(goal, .01, map_trigger=False)
    assert target[1]-measured[1] == pytest.approx(.006)
    assert target[5] == measured[5]
    measured[1] = math.radians(-103)
    with pytest.raises(ValueError, match='too far'):
        initialization_target(config, measured, limits, encoders)
    measured[1] = math.radians(-101.6)
    with pytest.raises(ValueError, match='too far'):
        initialization_target(config, measured, limits, encoders)
    measured[1] = math.radians(-100.85)
    encoders['shoulder_lift'] = (-1.75, 2)
    with pytest.raises(ValueError, match='encoder travel'):
        initialization_target(config, measured, limits, encoders)


@pytest.mark.parametrize('raw', [{j: -1 for j in JOINTS}, {j: 65535 for j in JOINTS}, {}])
def test_invalid_encoder_packets_cannot_produce_follow_targets(raw):
    with pytest.raises(ValueError, match='encoder'):
        follow_server.angles(raw, {})


@pytest.fixture
def remote(monkeypatch, tmp_path):
    config = load_follow_config()
    config['simulation_shared_calibration'] = True
    config['trigger_range_measured'] = False
    arms = {name: SimpleNamespace(name=name, role=role)
            for name, role in [('arm1', 'leader'), ('follower', 'follower')]}
    calls, packets = [], []
    buses = {}

    class Bus:
        def __init__(self, arm):
            self.arm = arm

        def sync_read(self, register, normalize=False):
            assert normalize is False
            calls.append((self.arm.name, register))
            return {joint: 2200 if register == 'Present_Position' else 0 for joint in JOINTS}

        def write(self, *args, **kwargs):
            raise AssertionError('preview must not write either bus')

        sync_write = write

    @contextmanager
    def connect(arm):
        buses[arm.name] = Bus(arm)
        yield buses[arm.name]

    calibration = SimpleNamespace(fit=lambda joint: {'zero_raw': 2000, 'radians_per_tick': .0015})
    monkeypatch.setattr(follow_server, 'connected', connect)
    monkeypatch.setattr(follow_server, 'load_arms', lambda: arms)
    monkeypatch.setattr(follow_server, 'load_follow_config', lambda path: config)
    monkeypatch.setattr(follow_server, 'workspace_root', lambda: tmp_path)
    monkeypatch.setattr(follow_server, 'calibrated_mapping', lambda arm, bus:
        (calibration, {'arm': arm.name, 'mode': 'independent', 'sha256': 'verified',
                       'fingerprint': {'raw_limits': {j: [1000, 3000] for j in JOINTS}}}))
    monkeypatch.setattr(follow_server, 'calibration_identity', lambda *args: {'port': 'white'})
    events = []

    class Thread:
        def __init__(self, target, args, daemon):
            events.append(args[1])

        def start(self):
            pass

    monkeypatch.setattr(follow_server.threading, 'Thread', Thread)

    def send(packet):
        packets.append(packet)
        if packet['type'] == 'state':
            events[0].set()

    monkeypatch.setattr(follow_server, 'send', send)
    return config, calls, packets


def test_live_shared_preview_reads_both_buses_without_writes(remote):
    _, calls, packets = remote
    follow_server.serve(SimpleNamespace(config=None, hardware_control=False))
    assert ('arm1', 'Present_Position') in calls
    assert ('follower', 'Present_Position') in calls
    assert packets[0]['leader']['mode'] == 'temporary_shared_simulation_only'
    assert packets[-1]['leader_qpos'] == pytest.approx([.3]*6)
    assert packets[-1]['control'] is None


def test_real_execution_rejects_unmeasured_trigger_before_any_controller_writes(remote):
    with pytest.raises(ValueError, match='Measure leader trigger'):
        follow_server.serve(SimpleNamespace(config=None, hardware_control=True))


@pytest.mark.parametrize('initialization',
                         ['inside', 'recover', 'recover_lag', 'timeout', 'too_far'])
def test_execute_wires_only_follower_controller_and_closes_it_on_disconnect(
        remote, monkeypatch, initialization):
    config, _, packets = remote
    config['trigger_range_measured'] = True
    events = []
    limits = [(-1, 1)]*6
    if initialization != 'inside':
        limits[1] = (.35 if initialization == 'too_far' else .315, 1)
    if initialization == 'timeout':
        config['initialization']['timeout_seconds'] = 1e-9
    monkeypatch.setattr(follow_server, 'joint_ranges', lambda: limits)

    class Controller:
        def __init__(self, bus, calibration, limits, raw_limits):
            assert bus.arm.name == 'follower'
            self.limits = {joint: [-1, 1] for joint in JOINTS}
            self.stopped = {}

        def enable_all(self):
            events.append('enable_follower')

        def target(self, joint, angle):
            assert events[0] == 'enable_follower'
            events.append(joint)

        def step(self, raw, telemetry):
            events.append('feedback_step')

        def status(self):
            return {'writes_enabled': True}

        def stop(self, reason):
            self.stopped = {'shoulder_lift': reason}
            events.append('stop_hold')

        def close(self):
            events.append('close_hold')

    class Thread:
        def __init__(self, target, args, daemon):
            self.commands, self.disconnected = args

        def start(self):
            import time

            self.commands.put((time.monotonic(), {'version': 1, 'op': 'resume', 'id': 1}))

    # A single complete cycle ends with client disconnect, including cleanup.
    def send(packet):
        packets.append(packet)
        if packet['type'] == 'state':
            if initialization in ('recover', 'recover_lag') and packet['stage'] == 'initializing':
                assert packet['seq'] < 100, 'initialization must finish despite encoder lag'
                return
            raise StopIteration('client disconnected')

    monkeypatch.setattr(follow_server, 'ManualControl', Controller)
    monkeypatch.setattr(follow_server.threading, 'Thread', Thread)
    monkeypatch.setattr(follow_server, 'send', send)
    def feedback(bus):
        raw = {joint: 2200 for joint in JOINTS}
        if initialization == 'recover' and any(p['type'] == 'state' for p in packets):
            raw['shoulder_lift'] = 2216
        return {'Present_Position': raw}

    monkeypatch.setattr(follow_server, 'read_control_feedback', feedback)
    with pytest.raises(StopIteration, match='disconnected'):
        follow_server.serve(SimpleNamespace(config=None, hardware_control=True))
    assert events[-1] == 'close_hold'
    if initialization in ('inside', 'recover', 'recover_lag'):
        assert all(joint in events for joint in JOINTS)
        assert packets[-1]['following'] is True
        assert packets[-1]['control']['control_hz'] == config['hz']
    else:
        assert packets[-1]['stage'] == 'held'
        assert packets[-1]['following'] is False
    if initialization in ('recover', 'recover_lag'):
        first = next(p for p in packets if p['type'] == 'state')
        assert first['stage'] == 'initializing'
        assert first['target_qpos'][5] == pytest.approx(.3)
    if initialization == 'recover_lag':
        assert packets[-1]['qpos'][1] == pytest.approx(.3)
        assert packets[-1]['target_qpos'][1] >= limits[1][0]
    if initialization == 'too_far':
        assert 'enable_follower' not in events
        assert any(p.get('ok') is False and 'too far' in p['error'] for p in packets)
        assert 'too far' in packets[-1]['notice']
        assert packets[-1]['command_error'] in packets[-1]['notice']


def test_follow_derives_pid_and_frequency_from_shared_config(tmp_path):
    import json

    from lerobot_arm_control.control.control_defaults import CONTROL, CONTROL_HZ

    config = load_follow_config()
    assert config['controller_profile'] == CONTROL['name']
    assert config['hz'] == CONTROL_HZ == 60
    config.pop('hz')
    path = tmp_path/'follow.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='control.json'):
        load_follow_config(path)
