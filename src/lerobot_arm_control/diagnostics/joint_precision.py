"""Local orchestration of a finite, supervised isolated-joint repeatability trial.

Encoder statistics do not measure Cartesian accuracy. All servo control runs in
the existing remote worker using the shared control profile and watchdog.
"""

import argparse
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path

import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, TRAJECTORY
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
from lerobot_arm_control.runtime.realtime_client import Link

ROOT = workspace_root()


def precision_metrics(samples):
    """Use settled windows, keeping approach direction and reference quantization."""
    groups = {}
    for sample in samples:
        if sample['settled_window']:
            groups.setdefault(sample['visit'], []).append(sample)
    visits = []
    for visit, rows in groups.items():
        errors = np.array([row['reference_raw']-row['raw'] for row in rows])
        visits.append({'visit': visit, 'reference_raw': rows[0]['reference_raw'],
                       'samples': len(rows), 'median_raw': float(np.median(
                           [row['raw'] for row in rows])),
                       'rms_error_ticks': float(np.sqrt(np.mean(errors**2))),
                       'max_error_ticks': int(np.max(np.abs(errors)))})
    settled = [row for row in samples if row['settled_window']]
    if not settled:
        raise ValueError('No settled measurements')
    errors = np.array([row['reference_raw']-row['raw'] for row in settled])
    references = sorted({row['reference_raw'] for row in settled})
    repeats = {str(reference): float(np.ptp([visit['median_raw'] for visit in visits
                                            if visit['reference_raw'] == reference]))
               for reference in references}
    return {'settled_samples': len(settled), 'visits': visits,
            'rms_error_ticks': float(np.sqrt(np.mean(errors**2))),
            'p95_abs_error_ticks': float(np.percentile(np.abs(errors), 95)),
            'max_abs_error_ticks': int(np.max(np.abs(errors))),
            'repeat_visit_median_range_ticks': repeats,
            'benchmark_target_ticks': 2,
            'target_met_in_this_trial': bool(np.max(np.abs(errors)) <= 2),
            'measurement': 'servo encoder; Cartesian accuracy and gearbox backlash unmeasured'}


def marker_metrics(samples, expected_visits):
    """Independent image repeatability; missing visits cannot imply perfect return."""
    groups, seen = {}, set()
    for sample in samples:
        captured = sample['captured_unix_ns']
        if (sample['visit'] is None or not sample['settled_window'] or captured in seen):
            continue
        seen.add(captured)
        groups.setdefault(sample['visit'], []).append(sample)
    visits = [{'visit': visit, 'reference_raw': rows[0]['reference_raw'],
               'samples': len(rows), 'median_image_center_px': np.median(
                   [row['image_center_px'] for row in rows], axis=0).tolist()}
              for visit, rows in sorted(groups.items())]
    ranges = {}
    for reference in sorted({visit['reference_raw'] for visit in visits}):
        centers = np.array([visit['median_image_center_px'] for visit in visits
                            if visit['reference_raw'] == reference and visit['samples'] >= 2])
        ranges[str(reference)] = (float(np.linalg.norm(
            centers[:, None]-centers[None, :], axis=2).max()) if len(centers) >= 2 else None)
    return {'visits': visits, 'max_pairwise_visit_median_distance_px': ranges,
            'all_visits_observed': set(expected_visits) <= {
                visit['visit'] for visit in visits if visit['samples'] >= 2},
            'minimum_samples_per_visit': 2,
            'precision_validated': False,
            'measurement': 'native image center; no metric or absolute Cartesian accuracy claim'}


def trial(args):
    output = args.output or ROOT/f'logs/gripper-precision-{time.time_ns()}'
    output.mkdir(parents=True, exist_ok=False)
    remote_trace = args.remote_root+'/logs/'+output.name+'-control.json'
    joint = args.joint
    proof = {'version': 1, 'completed': False, 'joint': joint,
             'control_config_sha256': CONTROL_CONFIG_SHA256, 'samples': [], 'acks': [],
             'amplitude_ticks': args.amplitude_ticks, 'cycles': args.cycles,
             'reference_shape': 'step' if args.step else 'shared quintic entry',
             'servo_pid_or_eeprom_changed': False}
    camera = None
    if args.observe_marker:
        from lerobot_arm_control.runtime.tabletop_client import TableLink

        directory = output/'camera'
        directory.mkdir()
        camera = TableLink(args.host, args.remote_root, directory,
                           server_args=['--camera-only', '--marker-only', '--stream-fps', '10'])
        proof['marker_samples'] = []
    link = Link(args.host, args.remote_root, 'hardware', hardware_control=True,
                control_log=remote_trace)
    last_seq, last_ping = None, 0
    last_camera_seq, last_marker_received = None, 0
    require_marker = False
    start = None

    def command(op, **payload):
        ack = link.wait_ack(link.send(op, **payload))
        proof['acks'].append(ack)
        if not ack['ok']:
            raise RuntimeError(ack.get('error', 'command rejected'))
        return ack.get('result')

    def observe(visit, reference, settled):
        nonlocal last_seq, last_ping, last_camera_seq, last_marker_received
        _, state, received, _, _ = link.snapshot()
        now = time.monotonic()
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or 'motor worker disconnected')
        if state is None:
            return None
        if now-received > .5:
            raise RuntimeError('motor feedback older than 500 ms')
        if now-last_ping > .1:
            link.send('ping')
            last_ping = now
        if state['control']['last_fault']:
            raise RuntimeError(state['control']['last_fault']['reason'])
        if start is not None and any(abs(state['raw'][other]-start['raw'][other]) > 4
                                     for other in state['raw'] if other != joint):
            raise RuntimeError('an uncommanded arm joint moved; trial stopped')
        if camera:
            _, packet, camera_received, _, error = camera.snapshot()
            if error or camera.process.poll() is not None:
                raise RuntimeError(error or 'marker camera disconnected')
            if packet:
                image_state = packet[0]
                capture_age = (now-camera_received
                               +image_state.get('capture_age_at_send_ms', 1000)/1000)
                marker = image_state.get('tool_marker', {})
                valid = (image_state.get('marker_visible') and marker.get('position_valid')
                         and capture_age < .5 and marker.get('id') == 8
                         and marker.get('source_short_edge_px', 0) >= 30)
                if valid:
                    last_marker_received = now-capture_age
                if image_state['sequence'] != last_camera_seq:
                    last_camera_seq = image_state['sequence']
                    if valid:
                        proof['marker_samples'].append({
                            'visit': visit, 'reference_raw': reference,
                            'settled_window': bool(settled and capture_age < .25),
                            'capture_age_s': capture_age,
                            'captured_unix_ns': image_state['captured_unix_ns'],
                            'motor_unix_ns': state['time_unix_ns'],
                            'measured_raw': state['raw'][joint],
                            'image_center_px': np.mean(marker['source_corners'], axis=0).tolist(),
                            'marker': marker})
            if require_marker and now-last_marker_received > .8:
                raise RuntimeError('fresh decoded tool marker lost for 800 ms; trial stopped')
        if visit is not None and state['seq'] != last_seq:
            proof['samples'].append({'visit': visit, 'reference_raw': reference,
                                     'raw': state['raw'][joint],
                                     'time_unix_ns': state['time_unix_ns'],
                                     'settled_window': settled,
                                     'servo_goal_raw': state['servo_goal_raw'][joint],
                                     'torque': state['telemetry']['Torque_Enable'][joint]})
        last_seq = state['seq']
        return state

    try:
        deadline = time.monotonic()+15
        while start is None:
            start = observe(None, None, False)
            if time.monotonic() > deadline:
                raise TimeoutError('no initial motor state')
            time.sleep(.01)
        proof['initial_state'] = start
        if any(start['telemetry']['Torque_Enable'].values()):
            raise ValueError('isolated joint trial requires initially unpowered, supported arm')
        hello = link.snapshot()[0]
        initial_raw = start['raw'][joint]
        raw = initial_raw if args.reference_raw is None else args.reference_raw
        if abs(raw-initial_raw) > args.amplitude_ticks:
            raise ValueError('Reference center is farther than one trial amplitude from the arm')
        identity = {**hello['fingerprint'], 'model': 'so101', 'mjcf_sha256': hashlib.sha256(
            (resource_path('assets/so101/so101.xml')).read_bytes()).hexdigest()}
        calibration = AngleCalibration(ROOT/'data/calibration/follower_mujoco.json', identity)
        if not np.allclose(calibration.angles(start['raw'], [0]*6), start['qpos'], atol=1e-10):
            raise ValueError('local and remote angle mappings differ')
        fit = calibration.fit(joint)
        targets = [raw, raw+args.amplitude_ticks, raw]*args.cycles
        low, high = hello['raw_limits'][joint]
        if any(not low+40 <= target <= high-40 for target in targets):
            raise ValueError('joint trial does not fit within raw limits and correction margin')
        if joint == 'shoulder_pan':
            limits = joint_ranges()[hello['joints'].index(joint)]
            if any(not limits[0] <= (target-fit['zero_raw'])*fit['radians_per_tick'] <= limits[1]
                   for target in targets):
                raise ValueError('shoulder pan trial is outside canonical model limits')
        if camera:
            deadline = time.monotonic()+10
            while sum(sample['visit'] is None for sample in proof['marker_samples']) < 8:
                observe(None, None, False)
                if time.monotonic() > deadline:
                    raise TimeoutError('need eight fresh decoded marker observations before motion')
                time.sleep(.01)
            if time.monotonic()-last_marker_received > .5:
                raise RuntimeError('marker is not fresh enough to begin motion')
            require_marker = True
        proof['targets_raw'] = targets
        command('enable_joint', joint=joint)
        previous = initial_raw
        for visit, target in enumerate(targets):
            duration = max(TRAJECTORY['min_entry_seconds'],
                           1.875*abs(target-previous)*abs(fit['radians_per_tick'])
                           /TRAJECTORY['entry_speed_rad_s'])
            began = time.monotonic()
            while not args.step and time.monotonic()-began < duration:
                fraction = smooth_fraction((time.monotonic()-began)/duration)
                reference = round(previous+(target-previous)*fraction)
                command('joint_target', joint=joint, angle=(reference-fit['zero_raw'])
                        *fit['radians_per_tick'])
                observe(visit, reference, False)
                time.sleep(.03)
            command('joint_target', joint=joint, angle=(target-fit['zero_raw'])
                    *fit['radians_per_tick'])
            began = time.monotonic()
            while time.monotonic()-began < args.dwell:
                elapsed = time.monotonic()-began
                observe(visit, target, elapsed >= args.dwell-.8)
                time.sleep(.01)
            print(json.dumps({'event': 'precision_visit', 'joint': joint, 'visit': visit,
                              'target_raw': target, 'measured_raw': link.snapshot()[1]['raw']
                              [joint]}), flush=True)
            previous = target
        proof['metrics'] = precision_metrics(proof['samples'])
        if camera:
            proof['marker_metrics'] = marker_metrics(proof['marker_samples'], range(len(targets)))
        proof['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        proof['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            if link.latest is not None and link.process.poll() is None:
                status = command('hold')
                if joint in status['enabled']:
                    # Both permitted joints are non-support joints of an initially supported arm.
                    command('release_joint', joint=joint)
                proof['final_ack'] = proof['acks'][-1]
        except Exception as exc:
            proof['cleanup_error'] = str(exc)
        link.close()
        if camera:
            camera.close()
        try:
            subprocess.run(['scp', args.host+':'+remote_trace, str(output/'control.json')],
                           check=True, timeout=20)
        except Exception as exc:
            proof['trace_copy_error'] = str(exc)
        (output/'result.json').write_text(json.dumps(proof, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: proof[key] for key in ('completed', 'metrics', 'error',
                                                    'cleanup_error') if key in proof}
                         | {'output': str(output)}), flush=True)
    return proof['completed']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='lerobot')
    parser.add_argument('--remote-root', default='/home/xy/Research/lerobot')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--joint', choices=['gripper', 'shoulder_pan'], default='gripper')
    parser.add_argument('--observe-marker', action='store_true',
                        help='Require fresh ID 8 camera observations and record image centers')
    parser.add_argument('--step', action='store_true',
                        help='Use abrupt references for comparison with smooth default')
    parser.add_argument('--amplitude-ticks', type=int, default=16)
    parser.add_argument('--reference-raw', type=int,
                        help='Fixed comparison center; defaults to the measured initial encoder')
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--dwell', type=float, default=2)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.joint == 'shoulder_pan' and not args.observe_marker:
        parser.error('shoulder pan trials require --observe-marker')
    if (not args.execute or not 4 <= args.amplitude_ticks <= 32 or not 1 <= args.cycles <= 5
            or not math.isfinite(args.dwell) or not 1.5 <= args.dwell <= 4):
        parser.error('requires --execute, 4..32 ticks, 1..5 cycles and 1.5..4 s dwell')
    raise SystemExit(0 if trial(args) else 1)


if __name__ == '__main__':
    main()
