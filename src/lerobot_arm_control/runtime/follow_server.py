"""One remote process owns both buses; the leader bus is strictly read-only."""

import argparse
import hashlib
import math
import queue
import threading
import time
from contextlib import ExitStack
from pathlib import Path

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.follow import (
    JointFollower,
    initialization_target,
    load_follow_config,
)
from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.control.servo_feedback import read_control_feedback
from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.robot.so101.driver import connected
from lerobot_arm_control.robot.so101.model_metadata import MODEL, joint_ranges
from lerobot_arm_control.runtime.realtime_server import next_deadline, receive, send
from lerobot_arm_control.runtime.tabletop_robot import calibration_identity
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


def serve(args):
    config = load_follow_config(args.config)
    arms = load_arms()
    leader, follower = arms[config['leader']], arms[config['follower']]
    if leader.role != 'leader' or follower.role != 'follower':
        raise ValueError('follow roles must match configs/arms.toml')
    commands, disconnected = queue.Queue(maxsize=16), threading.Event()
    with ExitStack() as stack:
        leader_bus = stack.enter_context(connected(leader))
        follower_bus = stack.enter_context(connected(follower))
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
            limits = [(max(bounds[0], controller.limits[joint][0]),
                       min(bounds[1], controller.limits[joint][1]))
                      for joint, bounds in zip(JOINTS, limits, strict=True)]
            if any(low >= high for low, high in limits):
                raise ValueError('follower has no usable calibrated motion range')
        mapping = JointFollower(config, measured, limits)
        active = not args.hardware_control
        initialize_to, initialization_started, command_error = None, 0, None
        if not active:
            mapping.hold(measured)
        send({'type': 'hello', 'backend': 'hardware', 'joints': JOINTS,
              'hardware_control': args.hardware_control,
              'control_config_sha256': CONTROL_CONFIG_SHA256,
              'config': config, 'leader': leader_metadata, 'follower': follower_metadata})
        threading.Thread(target=receive, args=(commands, disconnected), daemon=True).start()
        sequence, previous, deadline = 0, time.monotonic(), time.monotonic()
        heartbeat = previous
        notice = ('Connected; waiting for client initialization; E resumes; Space holds'
                  if controller
                  else 'Read-only hardware -> simulated follower')
        while not disconnected.is_set():
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
                                                         controller.limits)
                            if any(leader_bus.sync_read('Status', normalize=False).values()):
                                raise ValueError('leader motor fault; cannot initialize')
                            controller.enable_all()
                            initialize_to = goal if goal != measured else None
                            initialization_started = time.monotonic()
                        mapping.resume(measured if controller else mapping.target)
                        active = True
                        heartbeat = time.monotonic()
                        command_error = None
                        notice = ('Initializing follower into its model range'
                                  if initialize_to else 'Following')
                    else:
                        raise ValueError('unknown follow command')
                    send({'type': 'ack', 'id': command['id'], 'op': op, 'ok': True})
                except (ValueError, RuntimeError) as exc:
                    active = False
                    initialize_to = None
                    mapping.hold(measured if controller else mapping.target)
                    notice = command_error = str(exc)
                    if controller:
                        controller.stop('follow command failed: hold')
                    send({'type': 'ack', 'id': command.get('id'), 'ok': False, 'error': str(exc)})
            sampled = time.monotonic()
            leader_raw = leader_bus.sync_read('Present_Position', normalize=False)
            leader_qpos = angles(leader_raw, leader_fits)
            leader_status = leader_bus.sync_read('Status', normalize=False)
            if controller:
                telemetry = read_control_feedback(follower_bus)
                raw = telemetry.pop('Present_Position')
            else:
                raw = follower_bus.sync_read('Present_Position', normalize=False)
                telemetry = {}
            measured = angles(raw, follower_fits)
            now = time.monotonic()
            stale = (now-sampled > config['max_sample_age_seconds']
                     or now-heartbeat > WATCHDOG_SECONDS)
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
            send({'type': 'state', 'seq': sequence, 'time_unix_ns': time.time_ns(),
                  'raw': raw, 'qpos': measured, 'leader_qpos': leader_qpos,
                  'target_qpos': target, 'following': active and initialize_to is None,
                  'stage': ('initializing' if initialize_to is not None else
                            'following' if active else 'held'), 'notice': notice,
                  'command_error': command_error,
                  'leader_status': leader_status, 'sample_age_ms': (now-sampled)*1000,
                  'heartbeat_age_ms': (now-heartbeat)*1000,
                  'control': (controller.status() | {'control_hz': config['hz']}
                              if controller else None)})
            sequence += 1
            previous = now
            deadline = next_deadline(deadline, time.monotonic(), 1/config['hz'])
            disconnected.wait(max(0, deadline-time.monotonic()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--hardware-control', action='store_true')
    args = parser.parse_args()
    try:
        serve(args)
    except Exception as exc:
        send({'type': 'error', 'error': str(exc)})
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
