"""Local collision checks and a finite, supervised five-joint response benchmark."""

import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.joint_path import ACTIVE, plan_digest
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.runtime.realtime_client import Link
from lerobot_arm_control.runtime.tabletop_client import TableLink
from lerobot_arm_control.transport.realtime import JOINTS, checked_command

ROOT = workspace_root()


def motor_observation(link):
    """Acknowledgements are separate from worker errors in Link.snapshot()."""
    _, state, received, _, _ = link.snapshot()
    if link.error or link.process.poll() is not None:
        raise RuntimeError(link.error or 'motor worker exited')
    if state and (time.monotonic()-received > .5 or state['control']['last_fault']):
        raise RuntimeError('stale motor feedback or motor fault')
    return state


def check_plan(plan, calibration):
    model = mujoco.MjModel.from_xml_path(str(ROOT/'assets/so101/scene_mujoco.xml'))
    data = mujoco.MjData(model)
    previous = plan['initial_raw']
    checked = 0
    for point in plan['waypoints']:
        target = {**point['raw'], 'gripper': previous['gripper']}
        for fraction in np.linspace(0, 1, 201):
            raw = {j: previous[j]+fraction*(target[j]-previous[j]) for j in JOINTS}
            data.qpos[:] = calibration.angles(raw, [0.]*6)
            mujoco.mj_forward(model, data)
            if any(contact.dist < -.001 for contact in data.contact):
                raise ValueError('joint path has a model self/floor collision')
            checked += 1
        previous = target
    return checked


def metrics(trace, calibration):
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    actual, nominal = mujoco.MjData(model), mujoco.MjData(model)
    mount = np.array(json.loads((ROOT/'configs/tabletop.json').read_text())[
        'robot']['tool_marker']['cad_reference']['marker_pose_in_gripper']['position_m'])
    body = model.body('gripper').id
    groups = {}
    for frame in trace['frames']:
        path = frame.get('joint_path')
        if not path or path['phase'] not in ('moving', 'dwelling') or path['segment'] == 0:
            continue
        reference = frame['nominal_qpos']
        q = calibration.angles(frame['raw'], [0.]*6)
        target = [reference.get(j, q[i]) for i, j in enumerate(JOINTS)]
        actual.qpos[:], nominal.qpos[:] = q, target
        mujoco.mj_forward(model, actual)
        mujoco.mj_forward(model, nominal)
        a = actual.xpos[body]+actual.xmat[body].reshape(3, 3)@mount
        b = nominal.xpos[body]+nominal.xmat[body].reshape(3, 3)@mount
        group = groups.setdefault(path['segment'], {'moving': [], 'settled': [], 'rows': []})
        error = {'xyz_mm': float(np.linalg.norm(a-b)*1000),
                 'joint_error_deg': np.rad2deg(np.array(target[:5])-q[:5]).tolist()}
        segment = path['segments'][path['segment']]
        if path['phase'] == 'moving':
            group['moving'].append(error)
        elif path['segment_elapsed_seconds'] >= segment['seconds']+segment['dwell_seconds']-.8:
            group['settled'].append(error)
        group['rows'].append((path['segment_elapsed_seconds'], frame['raw'], segment))
    summary = []
    for segment_id, group in groups.items():
        moving, settled = group['moving'], group['settled']
        if not moving or not settled:
            raise ValueError('complete dynamic and settled windows are required')
        rows, t90 = group['rows'], {}
        segment = rows[0][2]
        for joint in ACTIVE:
            start = calibration.fit(joint)['zero_raw']+segment['start'][joint]/calibration.fit(
                joint)['radians_per_tick']
            end = calibration.fit(joint)['zero_raw']+segment['end'][joint]/calibration.fit(
                joint)['radians_per_tick']
            reached = [t for t, raw, _ in rows if (raw[joint]-start)/(end-start) >= .9]
            t90[joint] = min(reached) if reached else None
        summary.append({'segment': segment_id, 'planned_seconds': segment['seconds'],
                        'time_to_90_percent_seconds': t90,
                        'moving_rms_mm': float(np.sqrt(np.mean([e['xyz_mm']**2 for e in moving]))),
                        'moving_max_mm': max(e['xyz_mm'] for e in moving),
                        'settled_rms_mm': float(np.sqrt(np.mean(
                            [e['xyz_mm']**2 for e in settled]))),
                        'settled_max_mm': max(e['xyz_mm'] for e in settled),
                        'joint_moving_rms_deg': np.sqrt(np.mean(np.square(
                            [e['joint_error_deg'] for e in moving]), axis=0)).tolist()})
    moving = [e for g in groups.values() for e in g['moving']]
    settled = [e for g in groups.values() for e in g['settled']]
    return {'segments': summary,
            'planned_moving_seconds': sum(s['planned_seconds'] for s in summary),
            'moving_rms_mm': float(np.sqrt(np.mean([e['xyz_mm']**2 for e in moving]))),
            'moving_max_mm': max(e['xyz_mm'] for e in moving),
            'settled_rms_mm': float(np.sqrt(np.mean([e['xyz_mm']**2 for e in settled]))),
            'settled_max_mm': max(e['xyz_mm'] for e in settled),
            'measurement': 'encoder FK tracking of nominal CAD marker; no absolute optical claim'}


def trial(args):
    args.output.mkdir(parents=True, exist_ok=False)
    angle_path = ROOT/'data/calibration/follower_mujoco.json'
    saved = json.loads(angle_path.read_text())
    calibration = AngleCalibration(angle_path, saved['fingerprint'])
    proof = {'completed': False, 'control_config_sha256': CONTROL_CONFIG_SHA256,
             'config': json.loads((ROOT/'configs/control.json').read_text()),
             'samples': [], 'acks': [], 'max_camera_age_s': 0.,
             'measurement_scope': 'encoder response benchmark; camera is recorded separately'}
    remote_trace = args.remote_root+'/logs/'+args.output.name+'-control.json'
    camera = TableLink(args.host, args.remote_root, args.output/'camera',
                       server_args=['--camera-only', '--marker-only', '--omit-native-images',
                                    '--stream-fps', '10'])
    link = Link(args.host, args.remote_root, 'hardware', hardware_control=True,
                control_log=remote_trace)
    last_seq, last_ping = None, 0.

    def observe():
        nonlocal last_seq, last_ping
        now = time.monotonic()
        state = motor_observation(link)
        if state:
            if now-last_ping > .1:
                link.send('ping')
                last_ping = now
            if state['seq'] != last_seq:
                proof['samples'].append(state)
                last_seq = state['seq']
        _, packet, received, _, error = camera.snapshot()
        if error or camera.process.poll() is not None:
            raise RuntimeError(error or 'camera worker exited')
        if packet:
            age = now-received+packet[0].get('capture_age_at_send_ms', 1000)/1000
            proof['max_camera_age_s'] = max(proof['max_camera_age_s'], age)
        return state, packet

    def command(operation, **fields):
        return checked_command(link, proof['acks'], operation, **fields)['result']

    try:
        deadline = time.monotonic()+15
        while True:
            state, packet = observe()
            recent = proof['samples'][-8:]
            stable = len(recent) == 8 and all(
                max(s['raw'][j] for s in recent)-min(s['raw'][j] for s in recent) <= 2
                for j in JOINTS)
            if state and packet and stable:
                break
            if time.monotonic() > deadline:
                raise TimeoutError('initial motor/camera feedback absent')
            time.sleep(.01)
        hello = link.snapshot()[0]
        if hello['joint_calibration_sha256'] != hashlib.sha256(angle_path.read_bytes()).hexdigest():
            raise ValueError('local and remote encoder mapping differs')
        initial = state['raw']
        anchor = json.loads(args.anchor.read_text()) if args.anchor else initial.copy()
        waypoints = [{'raw': {j: anchor[j] for j in ACTIVE}, 'dwell_seconds': 3}]
        for amplitude in args.amplitudes:
            offsets = [amplitude, round(amplitude*.625), -amplitude,
                       round(amplitude*.625), round(amplitude*1.25)]
            waypoints.append({'raw': {j: anchor[j]+delta for j, delta in zip(
                ACTIVE, offsets, strict=True)}, 'dwell_seconds': 3})
            waypoints.append({'raw': {j: anchor[j] for j in ACTIVE}, 'dwell_seconds': 3})
        plan = {'version': 1, 'initial_raw': initial, 'waypoints': waypoints}
        checked = check_plan(plan, calibration)
        validation = {'checked': True, 'plan_sha256': plan_digest(plan),
                      'control_config_sha256': CONTROL_CONFIG_SHA256,
                      'model_sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest(),
                      'joint_calibration_sha256': hello['joint_calibration_sha256']}
        proof.update(plan=plan, local_collision_samples=checked, initial_state=state)
        command('joint_path_start', plan=plan, validation=validation)
        deadline, last_segment = time.monotonic()+125, None
        while time.monotonic() < deadline:
            state, _ = observe()
            path = state.get('joint_path')
            if path:
                if last_segment != path['segment']:
                    print(json.dumps({'event': 'response_segment', 'segment': path['segment'],
                                      'phase': path['phase'], 'raw': state['raw']}), flush=True)
                    last_segment = path['segment']
                if path['phase'] == 'completed':
                    proof['completed'] = True
                    break
                if path['phase'].startswith('stopped'):
                    raise RuntimeError(path['phase'])
            time.sleep(.01)
        else:
            raise TimeoutError('finite joint path did not complete')
    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - hold on any orchestration failure.
        proof['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            if link.latest and link.process.poll() is None:
                proof['final_control'] = command('hold')
                proof['final_state'] = link.snapshot()[1]
        except Exception as exc:  # noqa: BLE001 - still close both resources.
            proof['cleanup_error'] = str(exc)
        try:
            link.close()
        finally:
            camera.close()
        subprocess.run(['scp', args.host+':'+remote_trace, str(args.output/'control.json')],
                       check=True, timeout=20)
        if proof['completed'] and not proof.get('cleanup_error'):
            proof['metrics'] = metrics(
                json.loads((args.output/'control.json').read_text()), calibration)
        (args.output/'result.json').write_text(json.dumps(proof, indent=2)+'\n')
        print(json.dumps({k: v for k, v in proof.items() if k in
                          ('completed', 'error', 'cleanup_error', 'metrics')}), flush=True)
    return proof['completed'] and not proof.get('cleanup_error')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_remote_arguments(parser)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--anchor', type=Path,
                        help='Saved six-joint raw pose for A/B trials; default is live pose')
    parser.add_argument('--output', type=Path,
                        default=ROOT/f'logs/response-{time.time_ns()}')
    parser.add_argument('--amplitudes', type=int, nargs='+', default=[64, 128, 256])
    args = parser.parse_args()
    if not args.execute or not 1 <= len(args.amplitudes) <= 6 or any(
            not 16 <= amplitude <= 256 for amplitude in args.amplitudes):
        parser.error('requires --execute and 1..6 amplitudes in 16..256 counts')
    raise SystemExit(0 if trial(args) else 1)


if __name__ == "__main__":
    main()
