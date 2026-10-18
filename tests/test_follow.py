"""Verify bounded joint mirroring and the remote read-only/physical-control boundary."""

import math
from contextlib import contextmanager, nullcontext
from types import SimpleNamespace

import pytest

from lerobot_arm_control.control.follow import (
    JointFollower,
    hardware_follow_limits,
    initialization_target,
    load_follow_config,
)
from lerobot_arm_control.runtime import follow_server
from lerobot_arm_control.transport.realtime import JOINTS


def test_mirror_slew_limit_joint_bounds_and_trigger_normalization():
    config = load_follow_config()
    config['entry_speed_rad_s'] = 2
    from lerobot_arm_control.robot.so101.model_metadata import joint_ranges

    limits = joint_ranges()
    limits[5] = tuple(map(math.radians, config['follower_gripper_degrees']))
    mapping = JointFollower(config, [0]*6, limits)
    midpoint = math.radians(sum(config['leader_trigger_degrees'])/2)
    for _ in range(80):
        target = mapping.step([10, 0, 0, 0, 0, midpoint], .05)
    assert target[0] == pytest.approx(mapping.limits[0][1])
    assert target[5] == pytest.approx(math.radians(sum(config['follower_gripper_degrees'])/2))
    mapping.hold(target)
    mapping.resume(target)
    previous = list(target)
    target = mapping.step([-10]*6, .01)
    assert max(abs(a-b) for a, b in zip(target, previous, strict=True)) <= .02000001


@pytest.mark.parametrize('fraction', [-.1, 0, .25, .5, .75, 1, 1.1])
def test_trigger_maps_measured_full_travel_and_clamps_only_at_endpoints(fraction):
    config = load_follow_config()
    low, high = config['leader_trigger_degrees']
    mapping = JointFollower(config, [0]*6, [(-3, 3)]*6)
    leader_degrees = low+fraction*(high-low)
    for _ in range(120):
        target = mapping.step([0]*5+[math.radians(leader_degrees)], .05)
    closed, opened = config['follower_gripper_degrees']
    expected = closed+(opened-closed)*max(0, min(1, fraction))
    assert math.degrees(target[5]) == pytest.approx(expected)


def test_current_follow_mapping_fits_current_command_envelope_and_initializes_in_place():
    import json

    from fakes import Bus

    from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.core.paths import resource_path
    from lerobot_arm_control.robot.so101.model_metadata import joint_ranges

    path = resource_path('data/calibration/follower_mujoco.json')
    saved = json.loads(path.read_text())
    calibration = AngleCalibration(path, saved['fingerprint'])
    bus = Bus()
    control = ManualControl(bus, calibration, saved['fingerprint']['raw_limits'])
    config = load_follow_config()
    control.set_command_limits('gripper', tuple(map(math.radians,
                                                   config['follower_gripper_degrees'])))
    cad = joint_ranges()
    limits = hardware_follow_limits(cad, control.limits)
    assert limits[5] == control.limits['gripper']
    assert limits[5][0] < cad[5][0]
    for joint, bounds, effective in zip(JOINTS[:5], cad[:5], limits[:5], strict=True):
        assert effective == (max(bounds[0], control.limits[joint][0]),
                             min(bounds[1], control.limits[joint][1]))
    plan = json.loads(resource_path('data/trajectories/table-circle120-tilt30.json').read_text())
    initial = plan['qpos'][0]
    assert initialization_target(config, initial, limits, control.rest_limits) == initial
    assert control.raw_limits['gripper'] == (464, 1881)
    for fraction in (0, .5, 1):
        mapping = JointFollower(config, initial, limits)
        low, high = config['leader_trigger_degrees']
        leader = initial[:5]+[math.radians(low+fraction*(high-low))]
        for _ in range(300):
            target = mapping.step(leader, 1/60)
        fit = calibration.fit('gripper')
        mapped_raw = target[5]/fit['radians_per_tick']+fit['zero_raw']
        assert mapped_raw == pytest.approx(464+fraction*(1881-464), abs=1e-9)
        assert abs(control.raw('gripper', target[5])-mapped_raw) <= .5+1e-9
    assert not bus.writes


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
    monkeypatch.setattr(follow_server, 'LatestOutput', lambda send, event: nullcontext(send))
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
    def connect(arm, *, allow_alarm=False):
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
                         ['inside', 'slow_enable', 'recover', 'recover_lag', 'timeout', 'too_far'])
def test_execute_wires_only_follower_controller_and_closes_it_on_disconnect(
        remote, monkeypatch, initialization):
    config, _, packets = remote
    config['trigger_range_measured'] = True
    events = []
    limits = [(-1, 1)]*6
    clock, command_queues = [100.], []
    if initialization == 'slow_enable':
        import time

        monkeypatch.setattr(follow_server, 'time', SimpleNamespace(
            monotonic=lambda: clock[0], time_ns=time.time_ns, sleep=lambda _: None))
    if initialization not in ('inside', 'slow_enable'):
        limits[1] = (.35 if initialization == 'too_far' else .315, 1)
    if initialization == 'timeout':
        config['initialization']['timeout_seconds'] = 1e-9
    monkeypatch.setattr(follow_server, 'joint_ranges', lambda: limits)

    class Controller:
        def __init__(self, bus, calibration, raw_limits):
            assert bus.arm.name == 'follower'
            self.limits = {joint: [-1, 1] for joint in JOINTS}
            self.rest_limits = dict(self.limits)
            self.stopped = {}
            self.last_fault = None

        def set_command_limits(self, joint, bounds):
            assert joint == 'gripper'
            assert bounds == tuple(map(math.radians, config['follower_gripper_degrees']))
            self.limits[joint] = bounds

        def enable_all(self):
            events.append('enable_follower')
            if initialization == 'slow_enable':
                clock[0] += .8
                command_queues[0].put((clock[0]-.4, {'version': 1, 'op': 'ping', 'id': 2}))

        def heartbeat(self):
            pass

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
            command_queues.append(self.commands)

        def start(self):
            self.commands.put((follow_server.time.monotonic(),
                               {'version': 1, 'op': 'resume', 'id': 1}))

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
    if initialization in ('inside', 'slow_enable', 'recover', 'recover_lag'):
        assert all(joint in events for joint in JOINTS)
        assert packets[-1]['stage'] in ('following', 'aligning')
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


@pytest.mark.parametrize('closed_degrees', [-12.43, -14.])
def test_closed_physical_gripper_initializes_by_bounded_opening_only(closed_degrees):
    from lerobot_arm_control.robot.so101.model_metadata import joint_ranges

    config = load_follow_config()
    measured = [0.]*5+[math.radians(closed_degrees)]
    limits = joint_ranges()
    encoders = {j: (-3., 3.) for j in JOINTS}
    encoders['gripper'] = tuple(map(math.radians, (-14.5, 100)))
    goal = initialization_target(config, measured, limits, encoders)
    assert goal[:5] == measured[:5]
    assert goal[5] == pytest.approx(limits[5][0]+math.radians(.5))
    assert 0 < goal[5]-measured[5] <= math.radians(5)
    mapping = JointFollower(config, measured, limits)
    previous = measured[5]
    for _ in range(10):
        target = mapping.step(goal, 1/60, map_trigger=False)
        assert 0 <= target[5]-previous <= config['entry_speed_rad_s']/60+1e-12
        previous = target[5]
    assert target == pytest.approx(goal)
    for invalid in (-15., 103.):
        with pytest.raises(ValueError):
            initialization_target(config, [0.]*5+[math.radians(invalid)], limits, encoders)
    with pytest.raises(ValueError, match='too far'):
        initialization_target(config, [math.radians(-114), 0, 0, 0, 0, .3], limits, encoders)


def test_closed_gripper_exception_does_not_relax_upper_model_stop():
    config = load_follow_config()
    with pytest.raises(ValueError, match='too far'):
        initialization_target(config, [0.]*5+[math.radians(103)],
                              [(-3., 3.)]*5+[tuple(map(math.radians, (-10, 100)))],
                              {j: (-3., 3.) for j in JOINTS})


def test_auto_initialization_waits_for_ready_and_never_retries():
    from lerobot_arm_control.control.follow import StartupFollow

    startup = StartupFollow(True)
    assert not startup.update(ready=False)
    assert startup.update(ready=True)
    assert not startup.update(ready=True)
    assert not startup.update(ready=True, inhibited=True)
    assert not startup.update(ready=True)


@pytest.mark.parametrize('cancellation', ['inhibited', 'manual_command'])
def test_pause_fault_or_manual_start_cancels_pending_auto_initialization(cancellation):
    from lerobot_arm_control.control.follow import StartupFollow

    startup = StartupFollow(True)
    assert not startup.update(ready=True, **{cancellation: True})
    assert not startup.update(ready=True)
    held = StartupFollow(False)
    assert not held.update(ready=True)


def test_auto_start_configuration_rejects_non_boolean(tmp_path):
    import json

    config = load_follow_config()
    for key in ('hz', 'controller_profile'):
        config.pop(key)
    config['initialization']['auto_start'] = 'false'
    path = tmp_path/'follow.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='auto_start must be boolean'):
        load_follow_config(path)


def test_tracking_uses_latest_pose_after_bounded_alignment_and_rearms_on_resume():
    config = load_follow_config()
    mapping = JointFollower(config, [0]*6)
    target = mapping.step([.4]*5+[0], .01)
    assert max(target) <= config['entry_speed_rad_s']*.01
    assert not mapping.tracking
    for _ in range(100):
        mapping.step([.4]*5+[0], .01)
    assert mapping.tracking
    aligned_trigger = mapping.target[5]
    assert mapping.step([.8]*5+[0], .01) == [.8]*5+[aligned_trigger]
    mapping.hold([.2]*6)
    assert not mapping.tracking
    assert mapping.step([1]*6, .01) == [.2]*6
    mapping.resume([.2]*6)
    target = mapping.step([1]*6, .01)
    assert not mapping.tracking
    assert max(target) <= .2+config['entry_speed_rad_s']*.01


@pytest.mark.parametrize('confirmed', [0, 1])
def test_leader_fault_is_independently_confirmed_without_writes(confirmed):
    reads = []

    class Bus:
        def sync_read(self, register, normalize):
            return {j: int(j == 'gripper') for j in JOINTS}

        def read(self, register, joint, normalize):
            reads.append((register, joint))
            return confirmed

    status = follow_server.read_leader_status(Bus())
    assert reads == [('Status', 'gripper')]
    assert status['gripper'] == confirmed


def test_remote_standalone_does_not_need_observer_or_client_heartbeats(remote, monkeypatch):
    config, _, packets = remote
    config['trigger_range_measured'] = True
    heartbeats, closed = [], []
    monkeypatch.setattr(follow_server, 'FollowBroadcast',
                        lambda path: nullcontext(follow_server.send))
    monkeypatch.setattr(follow_server.signal, 'signal', lambda *args: None)

    class Controller:
        def __init__(self, *args):
            self.limits = {j: [-1, 1] for j in JOINTS}
            self.rest_limits = dict(self.limits)
            self.stopped = {}
            self.last_fault = None
            self.command_angles = {}
        def set_command_limits(self, joint, bounds):
            self.limits[joint] = bounds
        def enable_all(self):
            pass
        def heartbeat(self):
            heartbeats.append(True)
        def target(self, *args):
            pass
        def step(self, *args):
            pass
        def status(self):
            return {}
        def close(self):
            closed.append(True)

    monkeypatch.setattr(follow_server, 'ManualControl', Controller)
    monkeypatch.setattr(follow_server, 'read_control_feedback',
                        lambda bus: {'Present_Position': {j: 2200 for j in JOINTS}})
    follow_server.serve(SimpleNamespace(config=None, hardware_control=True, standalone=True))
    assert heartbeats
    assert closed == [True]
    assert packets[0]['remote_autonomous'] is True
    assert packets[-1]['stage'] in ('aligning', 'following')


def test_follower_overload_is_published_and_stays_latched_after_alarm_clears(remote, monkeypatch):
    config, _, packets = remote
    config['trigger_range_measured'] = True
    writes = []
    fault = {'reason': 'gripper: overload (Status=32): hold',
             'packet_error': '[RxPacketError] Overload error!'}
    monkeypatch.setattr(follow_server, 'FollowBroadcast',
                        lambda path: nullcontext(follow_server.send))
    monkeypatch.setattr(follow_server.signal, 'signal', lambda *args: None)

    class Controller:
        def __init__(self, *args):
            self.limits = {j: (-1, 1) for j in JOINTS}
            self.rest_limits = dict(self.limits)
            self.stopped, self.command_angles = {}, {}
            self.last_fault = None
        def set_command_limits(self, joint, bounds):
            self.limits[joint] = bounds
        def enable_all(self):
            pass
        def heartbeat(self):
            pass
        def target(self, joint, angle):
            writes.append((joint, angle))
        def step(self, raw, telemetry):
            if telemetry['Status']['gripper']:
                self.last_fault = fault
                self.stopped = {'gripper': fault['reason']}
        def status(self):
            return {'last_fault': self.last_fault}
        def close(self):
            pass

    statuses = iter([32, 0, 0])
    monkeypatch.setattr(follow_server, 'ManualControl', Controller)
    monkeypatch.setattr(follow_server, 'read_control_feedback', lambda bus: {
        'Present_Position': {j: 2200 for j in JOINTS}, 'Status': {'gripper': next(statuses)}})
    def send(packet):
        packets.append(packet)
        if packet['type'] == 'state' and packet['seq'] == 2:
            raise StopIteration('monitor stayed online')
    monkeypatch.setattr(follow_server, 'send', send)
    with pytest.raises(StopIteration, match='stayed online'):
        follow_server.serve(SimpleNamespace(config=None, hardware_control=True, standalone=True))
    states = [p for p in packets if p['type'] == 'state']
    assert len(states) == 3
    assert all(p['stage'] == 'held' and not p['following'] for p in states)
    assert [p['warning']['active'] for p in states] == [True, False, False]
    assert all(p['warning']['source'] == 'follower' and p['warning']['requires_resume']
               for p in states)
    assert len(writes) == 6  # No commands on the following two healthy-but-latched samples.


def test_device_error_during_leader_status_confirmation_remains_a_fault():
    class Bus:
        def sync_read(self, *args, **kwargs):
            return {joint: int(joint == 'gripper') for joint in JOINTS}
        def read(self, *args, **kwargs):
            raise RuntimeError('[RxPacketError] Input voltage error!')
    assert follow_server.read_leader_status(Bus())['gripper'] == 1


def test_voltage_warning_keeps_process_online_and_holds_until_explicit_resume(remote, monkeypatch):
    config, _, packets = remote
    config['trigger_range_measured'] = True
    writes, holds, closed = [], [], []
    monkeypatch.setattr(follow_server, 'FollowBroadcast',
                        lambda path: nullcontext(follow_server.send))
    monkeypatch.setattr(follow_server.signal, 'signal', lambda *args: None)

    class Controller:
        def __init__(self, *args):
            self.limits = {j: [-1, 1] for j in JOINTS}
            self.rest_limits = dict(self.limits)
            self.stopped = {}
            self.last_fault = None
            self.command_angles = {}
        def set_command_limits(self, joint, bounds):
            self.limits[joint] = bounds
        def enable_all(self):
            pass
        def heartbeat(self):
            pass
        def target(self, *args):
            writes.append(args)
        def stop(self, reason):
            self.stopped = {'gripper': reason}
            holds.append(reason)
        def step(self, *args):
            pass
        def status(self):
            return {}
        def close(self):
            closed.append(True)

    statuses = iter([0, 1, 0, 0])  # Resume preflight, active fault, then healthy samples.
    monkeypatch.setattr(follow_server, 'read_leader_status',
                        lambda *args: {'gripper': next(statuses)})
    monkeypatch.setattr(follow_server, 'ManualControl', Controller)
    monkeypatch.setattr(follow_server, 'read_control_feedback',
                        lambda bus: {'Present_Position': {j: 2200 for j in JOINTS}})
    def send(packet):
        packets.append(packet)
        if packet['type'] == 'state' and packet['seq'] == 2:
            raise StopIteration('test finished after three live states')
    monkeypatch.setattr(follow_server, 'send', send)
    with pytest.raises(StopIteration, match='three live states'):
        follow_server.serve(SimpleNamespace(config=None, hardware_control=True, standalone=True))
    states = [p for p in packets if p['type'] == 'state']
    assert len(states) == 3 and len(holds) == 1 and closed == [True]
    assert not writes
    assert all(p['stage'] == 'held' and 'process online' in p['notice'] for p in states)
    assert states[0]['warning']['active'] is True
    assert states[-1]['warning']['active'] is False
    assert 'e + Enter resumes' in states[-1]['notice']
