"""One remote process owns both buses; the leader bus is strictly read-only."""

import argparse
import hashlib
import math
import queue
import signal
import sys
import threading
import time
from collections import deque
from contextlib import ExitStack
from pathlib import Path

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.follow import (
    JointFollower,
    StartupFollow,
    hardware_follow_limits,
    initialization_target,
    load_follow_config,
)
from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.control.servo_feedback import read_control_feedback
from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.robot.so101.driver import connected
from lerobot_arm_control.robot.so101.model_metadata import MODEL, joint_ranges
from lerobot_arm_control.runtime.common import (
    defer_cyclic_gc,
    next_deadline,
    receive,
    save_control_trace,
    send,
)
from lerobot_arm_control.runtime.tabletop_robot import calibration_identity
from lerobot_arm_control.transport.follow_observer import FollowBroadcast
from lerobot_arm_control.transport.latest_output import LatestOutput
from lerobot_arm_control.transport.realtime import JOINTS, VERSION, WATCHDOG_SECONDS


def calibrated_mapping(arm, bus):
    identity = calibration_identity(bus, arm, MODEL)
    path = workspace_root()/'data/calibration'/f'{arm.name}_mujoco.json'
    calibration = AngleCalibration(path, identity)
    if calibration.count() != 6:
        raise ValueError(f'{arm.name}: independent six-joint calibration required: {path}')
    metadata = {'arm': arm.name, 'fingerprint': identity, 'path': str(path),
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'mode': 'independent'}
    return calibration, metadata


def angles(raw, fits):
    if set(raw) != set(JOINTS) or any(type(v) is not int or not 0 <= v <= 4095
                                     for v in raw.values()):
        raise ValueError('invalid six-joint single-turn encoder feedback')
    return [(raw[joint]-fits[joint]['zero_raw'])*fits[joint]['radians_per_tick']
            for joint in JOINTS]



def read_leader_status(bus, diagnostics=None):
    """Confirm bulk flags independently, as on the follower, without motor writes."""
    status = bus.sync_read('Status', normalize=False)
    for joint, value in status.items():
        if value:
            try:
                status[joint] = int(bus.read('Status', joint, normalize=False))
            except RuntimeError as exc:
                if '[RxPacketError]' not in str(exc):
                    raise
                # The SDK raises on a device-error bit in the reply header.
                # Retain the fault flag so motion holds; do not turn a hardware
                # alarm into a clean sample or kill the monitoring process.
                status[joint] = int(value)
                if diagnostics is not None:
                    diagnostics[joint] = str(exc)
    return status


def leader_fault_snapshot(bus, status, packet_errors):
    """Read diagnostic evidence after holding, without clearing or masking a fault."""
    evidence = {'time_unix_ns': time.time_ns(), 'status': dict(status),
                'packet_errors': dict(packet_errors), 'registers': {}}
    for register in ('Status', 'Present_Voltage', 'Present_Temperature',
                     'Present_Current', 'Min_Voltage_Limit', 'Max_Voltage_Limit',
                     'Torque_Enable'):
        try:
            evidence['registers'][register] = bus.sync_read(register, normalize=False)
        except (ConnectionError, RuntimeError) as exc:
            evidence['registers'][register] = {'error': str(exc)}
    return evidence


def receive_console(commands, disconnected):
    """Only the owning remote terminal can change motion state."""
    try:
        for sequence, line in enumerate(sys.stdin):
            op = {'e': 'resume', 'h': 'hold', ' ': 'hold', 'q': 'quit'}.get(line.rstrip('\r\n'))
            if op == 'quit':
                break
            if op:
                commands.put_nowait((time.monotonic(), {
                    'version': VERSION, 'id': sequence, 'op': op}))
    finally:
        disconnected.set()


def serve(args):
    standalone = getattr(args, 'standalone', False)
    config = load_follow_config(args.config)
    arms = load_arms()
    leader, follower = arms[config['leader']], arms[config['follower']]
    if leader.role != 'leader' or follower.role != 'follower':
        raise ValueError('follow roles must match configs/arms.toml')
    commands, disconnected = queue.Queue(maxsize=16), threading.Event()
    with ExitStack() as stack:
        trace = deque(maxlen=36000)
        metadata = {'follow_config': config}
        metadata['leader_fault_events'] = []
        metadata['follower_fault_events'] = []
        previous_follower_fault = None
        previous_leader_faults = {}
        latched_leader_faults = {}
        if getattr(args, 'latency_test', False):
            metadata.update(measurement_completed=False,
                            measurement_joints=list(args.latency_joints))
        trace_path = getattr(args, 'control_log', None)
        if standalone and trace_path is None:
            trace_path = workspace_root()/'logs'/f'follow-{time.time_ns()}.json'
            print(f'Control trace: {trace_path}', file=sys.stderr, flush=True)
        if trace_path:
            stack.callback(save_control_trace, trace_path, trace, (), metadata)
        defer_cyclic_gc(stack)
        if standalone:
            publish = stack.enter_context(FollowBroadcast(workspace_root()/'logs/follow.sock'))
            for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous_handler = signal.signal(signum, lambda *_: disconnected.set())
                stack.callback(signal.signal, signum, previous_handler)
        else:
            publish = stack.enter_context(LatestOutput(send, disconnected))
        leader_bus = stack.enter_context(connected(leader, allow_alarm=True))
        follower_bus = stack.enter_context(connected(follower, allow_alarm=True))
        calibration, follower_metadata = calibrated_mapping(follower, follower_bus)
        follower_fits = {joint: calibration.fit(joint) for joint in JOINTS}
        leader_path = workspace_root()/'data/calibration'/f'{leader.name}_mujoco.json'
        if (leader_path.exists() or args.hardware_control
                or not config['simulation_shared_calibration']):
            leader_calibration, leader_metadata = calibrated_mapping(leader, leader_bus)
            leader_fits = {joint: leader_calibration.fit(joint) for joint in JOINTS}
        else:
            leader_fits = follower_fits
            leader_metadata = {'arm': leader.name, 'mode': 'temporary_shared_simulation_only',
                               'source_arm': follower.name,
                               'source_sha256': follower_metadata['sha256'],
                               'fingerprint': calibration_identity(leader_bus, leader, MODEL)}
        metadata.update(leader=leader_metadata, follower=follower_metadata)
        if args.hardware_control and config['trigger_range_measured'] is not True:
            raise ValueError('Measure leader trigger travel and set '
                             'trigger_range_measured=true before execution')
        raw = follower_bus.sync_read('Present_Position', normalize=False)
        measured = angles(raw, follower_fits)
        controller = None
        limits = joint_ranges()
        if args.hardware_control:
            controller = ManualControl(follower_bus, calibration, limits,
                                       follower_metadata['fingerprint']['raw_limits'])
            stack.callback(controller.close)
            controller.set_command_limits(
                'gripper', tuple(map(math.radians, config['follower_gripper_degrees'])))
            limits = hardware_follow_limits(limits, controller.limits)
        mapping = JointFollower(config, measured, limits)
        measurement_joint = None
        measurement_elapsed = 0.
        active = not args.hardware_control
        initialize_to, initialization_started, command_error = None, 0, None
        if not active:
            mapping.hold(measured)
        publish({'type': 'hello', 'backend': 'hardware', 'joints': JOINTS,
              'remote_autonomous': standalone,
              'hardware_control': args.hardware_control,
              'control_config_sha256': CONTROL_CONFIG_SHA256,
              'config': config, 'leader': leader_metadata, 'follower': follower_metadata})
        threading.Thread(target=receive_console if standalone else receive,
                         args=(commands, disconnected), daemon=True).start()
        startup = StartupFollow(standalone and config['initialization']['auto_start']
                                and not getattr(args, 'start_held', False))
        if startup.update(ready=True):
            commands.put_nowait((time.monotonic(), {'version': VERSION, 'id': -1, 'op': 'resume'}))
        sequence, previous, deadline = 0, time.monotonic(), time.monotonic()
        started, last_notice = previous, None
        heartbeat = previous
        notice = ('Connected; waiting for client initialization; E resumes; Space holds'
                  if controller
                  else 'Read-only hardware -> simulated follower')
        while not disconnected.is_set():
            if (getattr(args, 'duration', None) is not None
                    and time.monotonic()-started >= args.duration):
                break
            while not commands.empty():
                received, command = commands.get_nowait()
                try:
                    if (command.get('version') != VERSION
                            or time.monotonic()-received > WATCHDOG_SECONDS):
                        raise ValueError('invalid or stale follow command')
                    op = command.get('op')
                    if op == 'ping':
                        heartbeat = max(heartbeat, received)
                        if controller:
                            controller.heartbeat()
                    elif op == 'mark':
                        if command.get('joint') not in JOINTS:
                            raise ValueError('invalid measurement joint')
                        measurement_joint = command['joint']
                    elif op == 'hold':
                        active = False
                        command_error = None
                        initialize_to = None
                        mapping.hold(measured if controller else mapping.target)
                        if controller:
                            controller.stop('follow paused: hold')
                        notice = 'Held; E resumes from current follower position'
                    elif op == 'resume':
                        if controller:
                            measured = angles(follower_bus.sync_read(
                                'Present_Position', normalize=False), follower_fits)
                            goal = initialization_target(config, measured, limits,
                                                         controller.rest_limits)
                            if any(read_leader_status(leader_bus).values()):
                                raise ValueError('leader motor fault; cannot initialize')
                            controller.enable_all()
                            initialize_to = goal if goal != measured else None
                            initialization_started = time.monotonic()
                            if standalone:
                                previous = initialization_started
                        mapping.resume(measured if controller else mapping.target)
                        active = True
                        latched_leader_faults = {}
                        heartbeat = time.monotonic()
                        command_error = None
                        notice = ('Initializing follower into its model range'
                                  if initialize_to else 'Following')
                    else:
                        raise ValueError('unknown follow command')
                    publish({'type': 'ack', 'id': command['id'], 'op': op, 'ok': True})
                except (ValueError, RuntimeError) as exc:
                    active = False
                    initialize_to = None
                    mapping.hold(measured if controller else mapping.target)
                    notice = command_error = str(exc)
                    if controller:
                        controller.stop('follow command failed: hold')
                    publish({'type': 'ack', 'id': command.get('id'),
                             'ok': False, 'error': str(exc)})
            sampled = time.monotonic()
            leader_raw = leader_bus.sync_read('Present_Position', normalize=False)
            leader_sampled = time.monotonic()
            leader_qpos = angles(leader_raw, leader_fits)
            leader_packet_errors = {}
            leader_status = read_leader_status(leader_bus, leader_packet_errors)
            if controller:
                telemetry = read_control_feedback(follower_bus)
                raw = telemetry.pop('Present_Position')
            else:
                raw = follower_bus.sync_read('Present_Position', normalize=False)
                telemetry = {}
            follower_sampled = time.monotonic()
            measured = angles(raw, follower_fits)
            now = time.monotonic()
            stale = (now-sampled > config['max_sample_age_seconds']
                     or (now-previous > config['max_sample_age_seconds'] if standalone
                         else now-heartbeat > WATCHDOG_SECONDS))
            if standalone and controller and not stale:
                # Liveness comes from this remote acquisition loop, never a viewer.
                controller.heartbeat()
                heartbeat = now
            if active and (stale or any(leader_status.values())):
                active = False
                initialize_to = None
                mapping.hold(measured if controller else mapping.target)
                notice = ('Stale/faulted leader or client: held; E required to resume; '
                          f'sample_age={now-sampled:.3f}s, heartbeat_age={now-heartbeat:.3f}s, '
                          f'leader_faults={dict((j, v) for j, v in leader_status.items() if v)}')
                if controller:
                    controller.stop(notice)
            if initialize_to is not None:
                tolerance = math.radians(config['initialization']['target_tolerance_degrees'])
                # This initializes the nominal command ramp. Native servo tracking
                # error remains observable; it must not gate teleoperation forever.
                if all(low <= value <= high and abs(value-goal) <= tolerance
                       for value, goal, (low, high) in zip(
                           mapping.target, initialize_to, limits, strict=True)):
                    initialize_to = None
                    mapping.resume(mapping.target)
                    notice = 'Initialized; following'
                elif now-initialization_started > config['initialization']['timeout_seconds']:
                    active = False
                    initialize_to = None
                    mapping.hold(measured)
                    controller.stop('initialization timed out: hold')
                    notice = 'Initialization timed out; held; E retries'
            target = mapping.step(initialize_to if initialize_to is not None else leader_qpos,
                                  now-previous, sample_age=now-sampled,
                                  map_trigger=initialize_to is None)
            if controller:
                if active:
                    for joint, angle in zip(JOINTS, target, strict=True):
                        controller.target(joint, angle)
                controller.step(raw, telemetry)
                if controller.stopped:
                    active = False
                    initialize_to = None
                    mapping.hold(measured)
                    notice = command_error or 'Follower held: '+str(controller.stopped)
            command_sent = time.monotonic()
            leader_faults = {j: v for j, v in leader_status.items() if v}
            if leader_faults:
                latched_leader_faults = leader_faults
            if leader_faults and leader_faults != previous_leader_faults:
                evidence = leader_fault_snapshot(leader_bus, leader_status, leader_packet_errors)
                metadata['leader_fault_events'].append(evidence)
                metadata['leader_fault_events'] = metadata['leader_fault_events'][-256:]
                if standalone:
                    print(f'Leader fault evidence (after hold): {evidence}',
                          file=sys.stderr, flush=True)
            previous_leader_faults = leader_faults
            follower_fault = controller.last_fault if controller else None
            if follower_fault is not None and follower_fault is not previous_follower_fault:
                metadata['follower_fault_events'].append(
                    {'time_unix_ns': time.time_ns(), **follower_fault})
                metadata['follower_fault_events'] = metadata['follower_fault_events'][-256:]
                previous_follower_fault = follower_fault
            stage = ('initializing' if initialize_to is not None else
                     'following' if active and mapping.tracking else
                     'aligning' if active else 'held')
            if stage == 'aligning':
                notice = 'Aligning slowly to leader; then live tracking; Space holds'
            elif stage == 'following':
                notice = 'Following live leader'
            warning = None
            control_status = controller.status() if controller else None
            contact = control_status.get('gripper_contact', {}) if control_status else {}
            if contact.get('active') and stage == 'following':
                notice = ('Following; gripper contact hold; '
                          'open trigger past held position to release')
            if latched_leader_faults:
                warning = {'severity': 'warning', 'source': 'leader',
                           'status': dict(latched_leader_faults),
                           'active': bool(leader_faults), 'requires_resume': True}
                kind = ('voltage' if all(v == 1 for v in latched_leader_faults.values())
                        else 'motor')
                condition = 'active' if leader_faults else 'cleared'
                notice = (f'WARNING: leader {kind} alarm {condition}; process online; '
                          f'follower held; joints={list(latched_leader_faults)}; '
                          + ('waiting for healthy readings' if leader_faults else 'E resumes'))
            if controller and controller.stopped and follower_fault is not None:
                follower_status = telemetry.get('Status', {})
                warning = {'severity': 'warning', 'source': 'follower',
                           'status': dict(follower_status),
                           'active': any(follower_status.values()), 'requires_resume': True,
                           'fault': follower_fault}
                notice = ('WARNING: follower '+follower_fault['reason']
                          + '; motion latched; monitoring online; E resumes after healthy readings')
                if follower_fault.get('hold_error'):
                    notice += '; '+follower_fault['hold_error']
            if getattr(args, 'latency_test', False) and stage == 'following':
                measurement_elapsed += min(now-previous, .1)
                if measurement_elapsed >= 12*len(args.latency_joints):
                    metadata['measurement_completed'] = True
                    break
                measurement_joint = args.latency_joints[int(measurement_elapsed//12)]
                notice += f' | MEASURE {measurement_joint}: {12-measurement_elapsed%12:.0f}s'
            if standalone:
                notice = notice.replace('Space holds', 'remote h + Enter holds')
                notice = notice.replace('E required', 'remote e + Enter required')
                notice = notice.replace('E resumes', 'remote e + Enter resumes')
            if standalone and notice != last_notice:
                print(notice+' | remote terminal: h + Enter holds, e + Enter resumes, q quits',
                      file=sys.stderr, flush=True)
                last_notice = notice
            if trace_path:
                trace.append({'seq': sequence, 'leader_time': leader_sampled,
                              'measurement_joint': measurement_joint,
                              'follower_time': follower_sampled, 'command_time': command_sent,
                              'leader': list(leader_qpos), 'actual': list(measured),
                              'reference': list(target),
                              'command': [controller.command_angles.get(j, measured[i])
                                          for i, j in enumerate(JOINTS)] if controller else target,
                              'stage': stage,
                              'leader_status': dict(leader_status),
                              'follower_status': dict(telemetry.get('Status', {})),
                              'gripper_contact': contact,
                              'warning': warning,
                              'notice': notice,
                              'sample_ms': (now-sampled)*1000,
                              'sensor_to_command_ms': (command_sent-leader_sampled)*1000})
            publish({'type': 'state', 'seq': sequence, 'time_unix_ns': time.time_ns(),
                  'raw': raw, 'qpos': measured, 'leader_qpos': leader_qpos,
                  'target_qpos': target,
                  'following': active and initialize_to is None and mapping.tracking,
                  'stage': stage, 'notice': notice,
                  'warning': warning,
                  'command_error': command_error,
                  'leader_status': leader_status, 'sample_age_ms': (now-sampled)*1000,
                  'follower_status': dict(telemetry.get('Status', {})),
                  'heartbeat_age_ms': (now-heartbeat)*1000,
                  'control': (control_status | {'control_hz': config['hz']}
                              if controller else None)})
            sequence += 1
            previous = now
            deadline = next_deadline(deadline, time.monotonic(), 1/config['hz'])
            disconnected.wait(max(0, deadline-time.monotonic()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--hardware-control', action='store_true')
    parser.add_argument('--control-log', type=Path)
    parser.add_argument('--standalone', action='store_true',
                        help='Own control on this host; viewers are read-only subscribers')
    parser.add_argument('--start-held', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--latency-test', action='store_true')
    parser.add_argument('--latency-joints', nargs='+', choices=JOINTS, default=JOINTS)
    args = parser.parse_args()
    if args.standalone and not args.hardware_control:
        parser.error('--standalone requires --hardware-control')
    if args.latency_test and (not args.standalone or not args.control_log):
        parser.error('--latency-test requires --standalone and --control-log')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    try:
        serve(args)
    except Exception as exc:
        send({'type': 'error', 'error': str(exc)})
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
