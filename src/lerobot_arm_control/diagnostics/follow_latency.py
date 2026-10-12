"""Estimate real leader-to-encoder tracking lag from one remote monotonic clock."""

import argparse
import gzip
import json
from pathlib import Path

import numpy as np

from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
from lerobot_arm_control.transport.realtime import JOINTS


def estimate_lag(reference_time, reference, measured_time, measured):
    """Fit delay, scale and offset; require a reversal so ramps cannot fake a fit."""
    rt, r, mt, m = map(np.asarray, (reference_time, reference, measured_time, measured))
    if len(rt) < 30 or len(mt) < 30 or np.ptp(r) < np.deg2rad(4):
        return None
    grid = np.arange(max(rt[0]+.5, mt[0]), min(rt[-1]-.1, mt[-1]), .01)
    if len(grid) < 100:
        return None
    # A straight ramp's delay is indistinguishable from static position error.
    sampled = np.interp(grid, rt, r)
    changes = np.diff(sampled[::10])
    if not (np.any(changes > np.deg2rad(.3)) and np.any(changes < -np.deg2rad(.3))):
        return None
    y = np.interp(grid, mt, m)
    variance = np.var(y)
    if variance < np.deg2rad(.25)**2:
        return None
    candidates = []
    for delay in np.arange(-.1, .502, .002):
        x = np.interp(grid-delay, rt, r)
        centered = x-x.mean()
        gain = np.dot(centered, y-y.mean()) / max(np.dot(centered, centered), 1e-12)
        residual = y-(gain*centered+y.mean())
        candidates.append((float(np.mean(residual**2)), float(delay), float(gain)))
    error, delay, gain = min(candidates)
    quality = 1-error/variance
    return {'lag_ms': max(0., delay*1000), 'signed_lag_ms': delay*1000,
            'gain': gain, 'r_squared': quality,
            'valid': bool(-.099 < delay < .499 and .65 <= gain <= 1.35 and quality >= .85)}


def analyze(trace, required_joints=JOINTS):
    if not required_joints or not set(required_joints) <= set(JOINTS):
        raise ValueError('invalid latency acceptance joints')
    frames = trace['frames']
    config = trace['metadata']['follow_config']
    result = {'definition': '4 s moving-window delay fit from leader encoder to follower encoder; '
              '100 ms bound includes worst encoder sampling interval',
              'control_config_sha256': trace['control_config_sha256'],
              'required_joints': list(required_joints), 'joints': {}}
    if not frames:
        return {**result, 'passed': False, 'error': 'empty trace'}
    rt = np.array([f['leader_time'] for f in frames])
    mt = np.array([f['follower_time'] for f in frames])
    desired = np.array([f['leader'] for f in frames])
    actual = np.array([f['actual'] for f in frames])
    nominal = np.array([f['reference'] for f in frames])
    lo, hi = np.deg2rad(config['leader_trigger_degrees'])
    a, b = np.deg2rad(config['follower_gripper_degrees'])
    desired[:, 5] = a+np.clip((desired[:, 5]-lo)/(hi-lo), 0, 1)*(b-a)
    desired = np.clip(desired, *np.array(joint_ranges()).T)
    following = np.array([f['stage'] == 'following' for f in frames])
    labels = [f.get('measurement_joint') for f in frames]
    guided = any(label is not None for label in labels)
    result['guided'] = guided
    for index, joint in enumerate(JOINTS):
        windows = []
        for start in np.arange(rt[0]+1, rt[-1]-4, 1):
            mask = (rt >= start) & (rt < start+4)
            if (mask.sum() < 100 or not following[mask].all()
                    or (guided and not np.all(np.array(labels)[mask] == joint))):
                continue
            measured = estimate_lag(rt[mask], desired[mask, index], mt[mask], actual[mask, index])
            if measured is None:
                continue
            gap = max(np.diff(rt[mask]).max(), np.diff(mt[mask]).max())*1000
            mapping = estimate_lag(rt[mask], desired[mask, index], rt[mask], nominal[mask, index])
            windows.append({'start_seconds': float(start-rt[0]), **measured,
                            'sampling_bound_ms': float(gap),
                            'upper_lag_ms': measured['lag_ms']+float(gap),
                            'mapping_lag_ms': mapping['lag_ms'] if mapping else None,
                            'speed_p95_rad_s': float(np.percentile(np.abs(
                                np.diff(desired[mask, index])/np.diff(rt[mask])), 95))})
        valid = [w for w in windows if w['valid']]
        result['joints'][joint] = {
            'windows': windows, 'valid_windows': len(valid),
            'worst_upper_lag_ms': max((w['upper_lag_ms'] for w in valid), default=None),
            'passed': len(valid) >= 2 and len(valid) == len(windows)
                      and all(w['upper_lag_ms'] < 100 for w in valid)}
    result['sensor_to_command_ms'] = {
        'p95': float(np.percentile([f['sensor_to_command_ms'] for f in frames], 95)),
        'max': max(f['sensor_to_command_ms'] for f in frames)}
    result['passed'] = all(result['joints'][j]['passed'] for j in required_joints)
    return result


def save_plot(trace, report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    frames = trace['frames']
    if not frames:
        return
    start = frames[0]['leader_time']
    config = trace['metadata']['follow_config']
    t = np.array([f['leader_time']-start for f in frames])
    measured_t = np.array([f['follower_time']-start for f in frames])
    leader = np.rad2deg([f['leader'] for f in frames])
    low, high = config['leader_trigger_degrees']
    closed, opened = config['follower_gripper_degrees']
    leader[:, 5] = closed+np.clip((leader[:, 5]-low)/(high-low), 0, 1)*(opened-closed)
    leader = np.clip(leader, *np.rad2deg(joint_ranges()).T)
    actual = np.rad2deg([f['actual'] for f in frames])
    reference = np.rad2deg([f['reference'] for f in frames])
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), constrained_layout=True)
    for i, (joint, ax) in enumerate(zip(JOINTS, axes.flat, strict=True)):
        ax.plot(t, leader[:, i], '--', color='#777777', label='Mapped live leader')
        ax.plot(t, reference[:, i], color='#bd8528', alpha=.75, label='Nominal target')
        ax.plot(measured_t, actual[:, i], color='#1870a0', label='Follower encoder')
        metric = report['joints'][joint]
        worst = metric['worst_upper_lag_ms']
        value = f'{worst:.1f} ms upper lag' if worst is not None else 'insufficient motion'
        title = f'{joint}: {value}'
        ax.set_title(title+' | '+('PASS' if metric['passed'] else 'REVIEW'))
        ax.set_xlabel('Remote monotonic time (s)')
        ax.set_ylabel('Angle (deg)')
        ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=8)
    fig.suptitle('Real leader → real follower | encoder tracking latency\n'
                 '4 s window fits; upper lag includes encoder sampling interval')
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    parser.add_argument('--joints', nargs='+', choices=JOINTS, default=JOINTS,
                        help='Explicit subset to accept; defaults to all six motors')
    args = parser.parse_args()
    opener = gzip.open if args.trace.suffix == '.gz' else open
    with opener(args.trace, 'rt') as stream:
        trace = json.load(stream)
    report = analyze(trace, required_joints=args.joints)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    if args.plot:
        save_plot(trace, report, args.plot)
    print(json.dumps({j: {k: v for k, v in m.items() if k != 'windows'}
                      for j, m in report['joints'].items()}, indent=2))
    if not report['passed']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
