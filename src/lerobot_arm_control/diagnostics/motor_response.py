"""Six independent small-step responses through the shared joint PID."""

import argparse
import csv
import json
import math
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL, CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.simulation_control import SimulationControl
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.diagnostics.response import motor_observation
from lerobot_arm_control.diagnostics.response_metrics import step_metrics
from lerobot_arm_control.runtime.realtime_client import Link
from lerobot_arm_control.transport.realtime import JOINTS, checked_command

ROOT = workspace_root()


def targets_for_pose(initial, limits, amplitude):
    """Choose the direction with more available travel; never clip a requested step."""
    targets = {}
    for i, joint in enumerate(JOINTS):
        lo, hi = limits[joint]
        q = initial[i]
        if not lo <= q <= hi:
            raise ValueError(f'{joint}: initial pose outside encoder limits')
        direction = 1 if hi-q >= q-lo else -1
        end = q+direction*amplitude
        if not lo+amplitude/4 <= end <= hi-amplitude/4:
            raise ValueError(f'{joint}: insufficient travel for a bounded response test')
        targets[joint] = end
    return targets


def collision_check(model, data, qadr, pose):
    data.qpos[qadr] = pose
    mujoco.mj_forward(model, data)
    if any(c.dist < -.001 for c in data.contact):
        raise ValueError('response pose has a model self/floor collision')


def validate_steps(initial, targets, limits):
    model = mujoco.MjModel.from_xml_path(str(ROOT/'assets/so101/scene_mujoco.xml'))
    data = mujoco.MjData(model)
    qadr = [int(model.joint(j).qposadr[0]) for j in JOINTS]
    samples = 0
    collision_check(model, data, qadr, initial)
    for i, joint in enumerate(JOINTS):
        bound = CONTROL['position_feedback']['joints'][joint]['max_correction_rad']
        lo = max(limits[joint][0], min(initial[i], targets[joint])-bound)
        hi = min(limits[joint][1], max(initial[i], targets[joint])+bound)
        for angle in np.linspace(lo, hi, 151):
            pose = list(initial)
            pose[i] = angle
            collision_check(model, data, qadr, pose)
            samples += 1
    return model, data, qadr, samples


def summarize(report):
    metrics = []
    rows = []
    for segment in report['segments']:
        frames = [f for f in report['frames'] if f['segment'] == segment['id']]
        if not frames:
            raise ValueError('missing response segment')
        i = JOINTS.index(segment['joint'])
        t = [f['t']-frames[0]['t'] for f in frames]
        q = [math.degrees(f['actual'][i]) for f in frames]
        result = step_metrics(t, q, math.degrees(segment['start']),
                              math.degrees(segment['target']))
        metrics.append({**segment, **result})
        for elapsed, frame in zip(t, frames, strict=True):
            rows.append([segment['joint'], segment['direction'], elapsed,
                         math.degrees(frame['reference'][i]), math.degrees(frame['actual'][i]),
                         math.degrees(frame['command'][i])])
    report['metrics'] = metrics
    report['performance_passed'] = len(metrics) == 12 and all(m['passed'] for m in metrics)
    return rows


def save_report(output, report):
    rows = []
    if report['completed']:
        rows = summarize(report)
    (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    with (output/'response.csv').open('w') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['joint', 'direction', 'time_s', 'reference_deg',
                         'actual_deg', 'command_deg'])
        writer.writerows(rows)
    if not rows:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), constrained_layout=True)
    for joint, ax in zip(JOINTS, axes.flat, strict=True):
        frames = [f for f in report['frames'] if any(
            s['id'] == f['segment'] and s['joint'] == joint for s in report['segments'])]
        i, t0 = JOINTS.index(joint), frames[0]['t']
        measured_label = 'Encoder angle' if report['backend'] == 'hardware' else 'Simulated angle'
        for key, label, color, style in [('reference', 'Reference', '#555555', '--'),
                                        ('actual', measured_label, '#1870a0', '-'),
                                        ('command', 'PID position command', '#c67728', ':')]:
            ax.plot([f['t']-t0 for f in frames],
                    [math.degrees(f[key][i]) for f in frames], style, label=label, color=color)
        results = [m for m in report['metrics'] if m['joint'] == joint]
        ax.set_title(joint+' — '+('PASS' if all(m['passed'] for m in results) else 'REVIEW'))
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Joint angle (deg)')
        ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=8)
    fig.suptitle(f"Six-motor step response | {report['backend']} | {report['config']['name']}\n"
                 f"60 Hz PID | {report['amplitude_deg']:g} deg | "
                 f"config {report['control_config_sha256'][:12]}")
    for extension in ('png', 'svg', 'pdf'):
        path = output/f'response.{extension}'
        fig.savefig(path, dpi=180)
        if extension == 'svg':
            path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
    plt.close(fig)


def hardware_trial(args, report):
    angle_path = ROOT/'data/calibration/follower_mujoco.json'
    saved = json.loads(angle_path.read_text())
    calibration = AngleCalibration(angle_path, saved['fingerprint'])
    remote_trace = args.remote_root+'/logs/'+args.output.name+'-motors.json'
    link = Link(args.host, args.remote_root, 'hardware', hardware_control=True,
                control_log=remote_trace)
    ping = 0.
    geometry = None

    def observe():
        nonlocal ping
        state = motor_observation(link)
        if state:
            now = time.monotonic()
            if now-ping >= .1:
                link.send('ping')
                ping = now
            if state['control']['stopped'] and report['segments']:
                raise RuntimeError('controller held: '+str(state['control']['stopped']))
            if geometry:
                collision_check(*geometry, state['qpos'])
                error = state['control']['error_rad']
                if any(abs(e) > math.radians(12) for e in error.values()):
                    raise RuntimeError('response tracking deviation exceeds 12 degrees')
        return state

    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            observe()
            time.sleep(.01)

    def command(operation, **fields):
        return checked_command(link, report['acks'], operation, **fields)

    try:
        deadline = time.monotonic()+15
        while not (state := observe()):
            if time.monotonic() > deadline:
                raise TimeoutError('initial feedback absent')
            time.sleep(.01)
        hello = link.snapshot()[0]
        if hello['joint_calibration_sha256'] != file_digest(angle_path):
            raise ValueError('local/remote joint calibration differs')
        report.update(hello=hello, initial_state=state)
        initial = state['qpos']
        limits = {j: tuple(sorted((v-calibration.fit(j)['zero_raw'])*
                                  calibration.fit(j)['radians_per_tick']
                                  for v in hello['raw_limits'][j])) for j in JOINTS}
        targets = targets_for_pose(initial, limits, math.radians(args.amplitude))
        model, data, qadr, checked = validate_steps(initial, targets, limits)
        geometry = model, data, qadr
        report['collision_samples'] = checked
        # Hold all load-bearing joints, then exercise one target at a time.
        command('enable_all')
        wait(1.)
        for i, joint in enumerate(JOINTS):
            for direction, target in [('out', targets[joint]), ('return', initial[i])]:
                start = observe()['qpos'][i]
                ack = command('joint_target', joint=joint, angle=target)
                segment = {'id': ack['id'], 'joint': joint, 'direction': direction,
                           'start': start, 'target': target}
                report['segments'].append(segment)
                print(json.dumps({'event': 'step', **segment}), flush=True)
                wait(args.dwell)
        report['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - always hold before persistence.
        report['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            if link.latest and link.process.poll() is None:
                report['final_control'] = command('hold')['result']
                report['final_state'] = link.snapshot()[1]
        except Exception as exc:  # noqa: BLE001 - close also attempts hold.
            report['cleanup_error'] = str(exc)
        link.close()
        try:
            subprocess.run(['scp', args.host+':'+remote_trace, str(args.output/'control.json')],
                           check=True, timeout=20)
            trace = json.loads((args.output/'control.json').read_text())
            report['trace_metadata'] = trace.get('metadata', {})
            final = report['trace_metadata'].get('final_native_parameters', {})
            original = report.get('hello', {}).get('native_parameters', {})
            report['native_parameters_restored'] = bool(original) and all(
                final.get(register) == values for register, values in original.items())
            if not report['native_parameters_restored']:
                report['cleanup_error'] = 'native parameter restoration not verified'
            for frame in trace['frames']:
                if frame['command_id'] not in {s['id'] for s in report['segments']}:
                    continue
                q = calibration.angles(frame['raw'], [0.]*6)
                report['frames'].append({'segment': frame['command_id'],
                    't': frame['time_monotonic'], 'actual': q,
                    'reference': [frame['nominal_qpos'][j] for j in JOINTS],
                    'command': calibration.angles(frame['goal_raw'], q)})
        except Exception as exc:  # noqa: BLE001 - retain partial evidence.
            report['error'] = 'trace retrieval: '+str(exc)
            report['completed'] = False


def simulation_trial(args, report):
    model = mujoco.MjModel.from_xml_path(str(ROOT/'assets/so101/scene_mujoco.xml'))
    data = mujoco.MjData(model)
    initial = np.array([0., -.5, .8, -.3, 0., .3])
    data.qpos[:] = initial
    controller = SimulationControl(model, data)
    for _ in range(round(2/model.opt.timestep)):
        controller.step(initial)
    targets = targets_for_pose(initial, controller.limits, math.radians(args.amplitude))
    _, _, _, report['collision_samples'] = validate_steps(initial, targets, controller.limits)
    reference = initial.copy()
    for i, joint in enumerate(JOINTS):
        for direction, target in [('out', targets[joint]), ('return', initial[i])]:
            segment = {'id': len(report['segments']), 'joint': joint, 'direction': direction,
                       'start': float(data.qpos[i]), 'target': target}
            report['segments'].append(segment)
            reference[i] = target
            for _ in range(round(args.dwell/model.opt.timestep)):
                before = controller.last_control
                controller.step(reference)
                if before != controller.last_control:
                    report['frames'].append({'segment': segment['id'], 't': float(data.time),
                        'actual': data.qpos.tolist(), 'reference': reference.tolist(),
                        'command': data.ctrl.tolist()})
    report['completed'] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('hardware', 'mujoco'), default='mujoco')
    parser.add_argument('--execute', action='store_true', help='Authorize finite real motor steps')
    add_remote_arguments(parser)
    parser.add_argument('--amplitude', type=float, default=3., help='Step size in degrees, 1..5')
    parser.add_argument('--dwell', type=float, default=4.,
                        help='Each step observation, 3..10 seconds')
    parser.add_argument('--output', type=Path, default=ROOT/f'logs/motor-response-{time.time_ns()}')
    args = parser.parse_args()
    if not 1 <= args.amplitude <= 5 or not 3 <= args.dwell <= 10:
        parser.error('amplitude must be 1..5 degrees and dwell 3..10 seconds')
    if args.backend == 'hardware' and not args.execute:
        parser.error('hardware response requires --execute')
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'version': 1, 'backend': args.backend, 'completed': False,
              'control_config_sha256': CONTROL_CONFIG_SHA256, 'config': CONTROL,
              'amplitude_deg': args.amplitude, 'dwell_seconds': args.dwell,
              'measurement': 'single pose, small steps; not a full workspace guarantee',
              'segments': [], 'frames': [], 'acks': []}
    try:
        (hardware_trial if args.backend == 'hardware' else simulation_trial)(args, report)
    finally:
        save_report(args.output, report)
    print(json.dumps({k: v for k, v in report.items() if k in
                      ('completed', 'performance_passed', 'error', 'cleanup_error', 'metrics')}))
    passed = report.get('performance_passed') and not report.get('cleanup_error')
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
