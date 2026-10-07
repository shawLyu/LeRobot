"""Local orchestration of a finite, supervised gripper repeatability trial.

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

from .angle_calibration import AngleCalibration
from .control_defaults import CONTROL_CONFIG_SHA256, TRAJECTORY
from .parking import smooth_fraction
from .realtime_client import Link

ROOT = Path(__file__).resolve().parents[2]


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


def trial(args):
    output = args.output or ROOT/f'logs/gripper-precision-{time.time_ns()}'
    output.mkdir(parents=True, exist_ok=False)
    remote_trace = args.remote_root+'/logs/'+output.name+'-control.json'
    proof = {'version': 1, 'completed': False, 'joint': 'gripper',
             'control_config_sha256': CONTROL_CONFIG_SHA256, 'samples': [], 'acks': [],
             'amplitude_ticks': args.amplitude_ticks, 'cycles': args.cycles,
             'reference_shape': 'step' if args.step else 'shared quintic entry',
             'servo_pid_or_eeprom_changed': False}
    link = Link(args.host, args.remote_root, 'hardware', hardware_control=True,
                control_log=remote_trace)
    last_seq, last_ping = None, 0
    start = None

    def command(op, **payload):
        ack = link.wait_ack(link.send(op, **payload))
        proof['acks'].append(ack)
        if not ack['ok']:
            raise RuntimeError(ack.get('error', 'command rejected'))
        return ack.get('result')

    def observe(visit, reference, settled):
        nonlocal last_seq, last_ping
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
        if start is not None and any(abs(state['raw'][joint]-start['raw'][joint]) > 4
                                     for joint in state['raw'] if joint != 'gripper'):
            raise RuntimeError('an uncommanded arm joint moved; trial stopped')
        if visit is not None and state['seq'] != last_seq:
            proof['samples'].append({'visit': visit, 'reference_raw': reference,
                                     'raw': state['raw']['gripper'],
                                     'time_unix_ns': state['time_unix_ns'],
                                     'settled_window': settled,
                                     'servo_goal_raw': state['servo_goal_raw']['gripper'],
                                     'torque': state['telemetry']['Torque_Enable']['gripper']})
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
            raise ValueError('isolated gripper trial requires initially unpowered, supported arm')
        hello = link.snapshot()[0]
        initial_raw = start['raw']['gripper']
        raw = initial_raw if args.reference_raw is None else args.reference_raw
        if abs(raw-initial_raw) > args.amplitude_ticks:
            raise ValueError('Reference center is farther than one trial amplitude from the arm')
        identity = {**hello['fingerprint'], 'model': 'so101', 'mjcf_sha256': hashlib.sha256(
            (ROOT/'assets/so101/so101.xml').read_bytes()).hexdigest()}
        calibration = AngleCalibration(ROOT/'data/calibration/follower_mujoco.json', identity)
        if not np.allclose(calibration.angles(start['raw'], [0]*6), start['qpos'], atol=1e-10):
            raise ValueError('local and remote angle mappings differ')
        fit = calibration.fit('gripper')
        targets = [raw, raw+args.amplitude_ticks, raw]*args.cycles
        low, high = hello['raw_limits']['gripper']
        if any(not low+40 <= target <= high-40 for target in targets):
            raise ValueError('gripper trial does not fit within raw limits and correction margin')
        proof['targets_raw'] = targets
        command('enable_joint', joint='gripper')
        previous = initial_raw
        for visit, target in enumerate(targets):
            duration = max(TRAJECTORY['min_entry_seconds'],
                           1.875*abs(target-previous)*abs(fit['radians_per_tick'])
                           /TRAJECTORY['entry_speed_rad_s'])
            began = time.monotonic()
            while not args.step and time.monotonic()-began < duration:
                fraction = smooth_fraction((time.monotonic()-began)/duration)
                reference = round(previous+(target-previous)*fraction)
                command('joint_target', joint='gripper', angle=(reference-fit['zero_raw'])
                        *fit['radians_per_tick'])
                observe(visit, reference, False)
                time.sleep(.03)
            command('joint_target', joint='gripper', angle=(target-fit['zero_raw'])
                    *fit['radians_per_tick'])
            began = time.monotonic()
            while time.monotonic()-began < args.dwell:
                elapsed = time.monotonic()-began
                observe(visit, target, elapsed >= args.dwell-.8)
                time.sleep(.01)
            print(json.dumps({'event': 'gripper_visit', 'visit': visit,
                              'target_raw': target, 'measured_raw': link.snapshot()[1]['raw']
                              ['gripper']}), flush=True)
            previous = target
        proof['metrics'] = precision_metrics(proof['samples'])
        proof['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        proof['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            if link.latest is not None and link.process.poll() is None:
                status = command('hold')
                if 'gripper' in status['enabled']:
                    # Gripper is not a gravity-support joint; release only this isolated motor.
                    command('release_joint', joint='gripper')
                proof['final_ack'] = proof['acks'][-1]
        except Exception as exc:
            proof['cleanup_error'] = str(exc)
        link.close()
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
    parser.add_argument('--step', action='store_true',
                        help='Use abrupt references for comparison with smooth default')
    parser.add_argument('--amplitude-ticks', type=int, default=16)
    parser.add_argument('--reference-raw', type=int,
                        help='Fixed comparison center; defaults to the measured initial encoder')
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--dwell', type=float, default=2)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if (not args.execute or not 4 <= args.amplitude_ticks <= 32 or not 1 <= args.cycles <= 5
            or not math.isfinite(args.dwell) or not 1.5 <= args.dwell <= 4):
        parser.error('requires --execute, 4..32 ticks, 1..5 cycles and 1.5..4 s dwell')
    raise SystemExit(0 if trial(args) else 1)


if __name__ == '__main__':
    main()
