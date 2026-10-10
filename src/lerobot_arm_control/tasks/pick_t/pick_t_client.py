"""Observe, locally simulate, then optionally execute a finite T pick/place policy."""

import argparse
import copy
import hashlib
import json
import math
import shutil
import time
from pathlib import Path

from lerobot_arm_control.planning.pick_t import PickPlanner
from lerobot_arm_control.robot.so101.model_metadata import MODEL, joint_ranges
from lerobot_arm_control.runtime.tabletop_client import ROOT, TableLink, atomic_json
from lerobot_arm_control.tasks.push_t.push_t_capture import calibration_bytes
from lerobot_arm_control.tasks.push_t.push_t_motor import joint_limit_issue, same_scene
from lerobot_arm_control.transport.pick_t_protocol import (
    CARRYING,
    STAGES,
    pick_observation_issue,
    placement_error,
)


def read_state(link):
    hello, packet, received, _, error = link.snapshot()
    if error or link.process.poll() is not None:
        raise RuntimeError(error or 'remote policy worker exited')
    return hello, packet[0] if packet else None, received


def acknowledge(link, operation, **fields):
    command_id = link.send(operation, **fields)
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        read_state(link)
        with link.lock:
            ack = link.acks.pop(command_id, None)
        if ack:
            if not ack['ok']:
                raise RuntimeError(ack.get('error', 'remote stage rejected'))
            return command_id
        time.sleep(.01)
    raise TimeoutError('remote stage acknowledgement timed out')


def save_snapshot(link, state, hello, output):
    output.mkdir(parents=True, exist_ok=True)
    atomic_json(output/'state.json', state)
    atomic_json(output/'calibration.json', hello)
    calibration = ROOT/'data/calibration/follower_mujoco.json'
    (output/'joint_calibration.json').write_bytes(calibration_bytes(hello, calibration))
    shutil.copyfile(link.output/'marker.png', output/'marker.png')
    if (link.output/'appearance').is_dir():
        shutil.copytree(link.output/'appearance', output/'appearance', dirs_exist_ok=True)
    else:
        (output/'appearance').mkdir(exist_ok=True)


def live(args, result):
    options = ['--pick-t', '--omit-native-images', '--stream-fps', '10']
    if args.execute:
        options.append('--hardware-control')
    link = TableLink(args.host, args.remote_root, args.output/'capture', server_args=options)
    try:
        deadline, last_report = time.monotonic()+args.timeout, 0.
        while time.monotonic() < deadline:
            hello, state, received = read_state(link)
            issue = pick_observation_issue(state, received, time.monotonic(), STAGES[0])
            if issue is None:
                issue = joint_limit_issue(state['robot']['qpos'], joint_ranges())
            if issue is None:
                break
            result['waiting_reason'] = issue
            if time.monotonic()-last_report >= 2:
                print(json.dumps({'event': 'pick_waiting', 'reason': issue}), flush=True)
                last_report = time.monotonic()
            time.sleep(.02)
        else:
            raise TimeoutError('pick/place preflight: '+result['waiting_reason'])
        if args.execute and hello['robot'].get('motion_available') is not True:
            raise RuntimeError('hardware executor is unavailable')
        save_snapshot(link, state, hello, args.output/'snapshot')
        planner = PickPlanner(args.output/'snapshot', args.output/'simulation')
        plan = planner.plan()
        simulation = planner.simulate(plan)
        atomic_json(args.output/'plan.json', plan)
        atomic_json(args.output/'simulation.json', simulation)
        result.update(plan=str(args.output/'plan.json'), simulation=simulation)
        if not simulation['passed']:
            raise ValueError('contact simulation rejected grasp or placement')
        if not args.execute:
            result.update(passed=True, status='preview_passed', physical_motion=False)
            return
        _, fresh, received = read_state(link)
        if pick_observation_issue(fresh, received, time.monotonic(), STAGES[0]) or not same_scene(
                state, fresh, wood_tolerance=.002, joint_tolerance=.02):
            raise ValueError('scene changed during planning; rerun from current observations')
        original_wood, target = copy.deepcopy(state['objects']['wood_t']), plan['target']
        original_wood['reference_capture_unix_ns'] = state['captured_unix_ns']
        result['physical_motion'] = True
        for stage_plan in plan['stages']:
            stage = stage_plan['stage']
            _, reference, received = read_state(link)
            reference = copy.deepcopy(reference)
            issue = pick_observation_issue(reference, received, time.monotonic(), stage,
                                           {'objects': {'wood_t': original_wood}})
            if issue:
                raise RuntimeError(issue)
            if stage in CARRYING or (stage in ('descend', 'close') and
                                    not reference['objects'].get('wood_t', {}).get('observed')):
                reference['objects']['wood_t'] = original_wood
            comparison = copy.deepcopy(reference)
            comparison['robot']['qpos'] = state['robot']['qpos']
            if stage in CARRYING or not comparison['objects']['wood_t'].get('observed'):
                comparison['objects']['wood_t'] = original_wood
            if not same_scene(state, comparison, wood_tolerance=.002):
                raise RuntimeError('target or fixed base changed during pick/place')
            segments = copy.deepcopy(stage_plan['segments'])
            actual = reference['robot']['qpos']
            segments[0]['start'][:5] = actual[:5]
            if stage not in ('lift', 'transfer', 'lower', 'open'):
                segments[0]['start'][5] = actual[5]
                if stage not in ('prepare', 'close', 'open'):
                    for segment in segments:
                        segment['start'][5] = segment['end'][5] = actual[5]
            if stage in ('prepare', 'close', 'open'):
                for segment in segments:
                    segment['start'][:5] = segment['end'][:5] = actual[:5]
            for segment in segments:
                segment['seconds'] = max(segment['seconds'], 1.875*max(
                    abs(a-b) for a, b in zip(segment['start'], segment['end'], strict=True))
                    /segment['speed_rad_s'])
            checked = planner.check_segments(segments, stage, original_wood)
            validation = planner.validation(segments)
            job = acknowledge(link, 'pick_execute', stage=stage, segments=segments,
                              reference={key: reference[key] for key in
                                         ('captured_unix_ns', 'robot', 'objects')},
                              validation=validation, model_sha256=validation['model_sha256'])
            print(json.dumps({'event': 'pick_stage', 'stage': stage,
                              'collision_samples': checked, 'job_id': job}), flush=True)
            deadline = time.monotonic()+sum(s['seconds'] for s in segments)+5
            while time.monotonic() < deadline:
                _, state_now, received = read_state(link)
                issue = pick_observation_issue(
                    state_now, received, time.monotonic(), stage, reference)
                if issue:
                    raise RuntimeError(issue)
                status = state_now.get('push_t', {})
                motor = status.get('motor') or {}
                if status.get('job_id') == job:
                    if motor['phase'] == 'completed':
                        result.setdefault('completed_stages', []).append(stage)
                        break
                    if motor['phase'] == 'stopped':
                        raise RuntimeError(motor.get('reason', 'motor stage stopped'))
                time.sleep(.01)
            else:
                raise TimeoutError(stage+' did not complete')
        # Wait for complete, newly captured wood observations after the arm retreats.
        final_capture = state_now['captured_unix_ns']
        seen, successful = set(), 0
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            _, frame, received = read_state(link)
            if frame['captured_unix_ns'] > final_capture and frame['captured_unix_ns'] not in seen:
                seen.add(frame['captured_unix_ns'])
                error = placement_error(frame, target)
                fresh_camera = time.monotonic()-received+frame['capture_age_at_send_ms']/1000 <= .5
                good = error and fresh_camera and error[0] <= .005 and error[1] <= math.radians(8)
                successful = successful+1 if good else 0
                if error:
                    result['final_center_error_m'], result['final_yaw_error_rad'] = error
                if successful == 3:
                    result.update(passed=True, status='placed', verification_frames=3)
                    acknowledge(link, 'pick_result', result=result)
                    return
            time.sleep(.02)
        raise RuntimeError('placement did not pass three fresh visible-T observations')
    finally:
        if args.execute and link.process.poll() is None:
            try:
                link.send('pick_stop')
            except OSError:
                pass
        link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Permit finite physical pick/place')
    parser.add_argument('--snapshot', type=Path, help='Offline scene; never opens hardware')
    parser.add_argument('--output', type=Path, default=ROOT/f'logs/pick-t-{time.time_ns()}')
    parser.add_argument('--host', default='lerobot')
    parser.add_argument('--remote-root', default='/home/xy/Research/lerobot')
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    if args.snapshot and args.execute:
        parser.error('offline snapshots cannot execute hardware')
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 120:
        parser.error('timeout must be in (0, 120] seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    result = {'passed': False, 'physical_motion': False, 'status': 'initializing',
              'model_sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest()}
    try:
        if args.snapshot:
            planner = PickPlanner(args.snapshot, args.output/'simulation')
            plan = planner.plan()
            simulation = planner.simulate(plan)
            atomic_json(args.output/'plan.json', plan)
            atomic_json(args.output/'simulation.json', simulation)
            result.update(passed=simulation['passed'], simulation=simulation,
                          status='offline_passed' if simulation['passed'] else 'offline_rejected')
        else:
            live(args, result)
    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - hold on any policy failure.
        result.update(status='stopped', error=str(exc) or type(exc).__name__)
    atomic_json(args.output/'result.json', result)
    print(json.dumps(result), flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
