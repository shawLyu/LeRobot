"""Finite local orchestration of supervised five-joint camera calibration poses."""

import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, POSITION_FEEDBACK
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.runtime.realtime_client import Link
from lerobot_arm_control.runtime.tabletop_client import TableLink
from lerobot_arm_control.transport.realtime import JOINTS, checked_command

ACTIVE = JOINTS[:5]
ROOT = workspace_root()


def close_workers(link, camera):
    """Attempt both closures even when restoration in one worker fails."""
    failures = []
    for name, worker in (('motors', link), ('camera', camera)):
        try:
            worker.close()
            if worker.process.returncode != 0:
                failures.append(f'{name}: worker exit {worker.process.returncode}')
        except Exception as exc:  # noqa: BLE001 - still close the other hardware resource.
            failures.append(f'{name}: {exc}')
    return '; '.join(failures)


def validate_plan(plan, initial, limits):
    """A photographed start and small, single-joint moves; never clamp a plan."""
    expected = plan.get('expected_initial_raw', {})
    if set(expected) != set(JOINTS) or any(type(v) is not int for v in expected.values()):
        raise ValueError('Plan requires six integer initial encoder positions')
    if any(abs(initial[j]-expected[j]) > 4 for j in JOINTS):
        raise ValueError('Arm moved since the inspected starting photograph')
    poses = plan.get('poses', [])
    if not 1 <= len(poses) <= 40:
        raise ValueError('Plan requires 1..40 finite poses')
    previous = {j: initial[j] for j in ACTIVE}
    for pose in poses:
        target = pose.get('raw', {})
        if set(target) != set(ACTIVE) or any(type(v) is not int for v in target.values()):
            raise ValueError('Each pose requires five integer raw targets')
        changed = [j for j in ACTIVE if target[j] != previous[j]]
        if len(changed) > 1 or any(abs(target[j]-previous[j]) > 32 for j in changed):
            raise ValueError('Each transition may move only one joint by at most 32 counts')
        if any(not limits[j][0]+8 <= target[j] <= limits[j][1]-8 for j in ACTIVE):
            raise ValueError('Pose is too close to a physical encoder limit')
        if any(abs(target[j]-initial[j]) > 160 for j in ACTIVE):
            raise ValueError('Finite scan exceeds its 160-count start envelope')
        previous = target
    return poses


def acquire(args):
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    plan = json.loads(args.plan.read_text())
    provenance = {}
    inputs = {'control': ROOT/'configs/control.json',
                       'tabletop': ROOT/'configs/tabletop.json',
                       'angles': ROOT/'data/calibration/follower_mujoco.json',
                       'model': MODEL}
    if not args.relative_only:
        inputs['base'] = ROOT/'data/calibration/follower_table_base.json'
    for name, path in inputs.items():
        provenance[name] = file_digest(path)
        shutil.copyfile(path, output/('input-'+name+path.suffix))
    trace = args.remote_root+'/logs/'+output.name+'-control.json'
    proof = {'version': 1, 'completed': False, 'control_config_sha256':
             CONTROL_CONFIG_SHA256, 'plan': plan, 'samples': [], 'poses': [], 'acks': [],
             'storage_unlocked': False, 'temporary_locked_p_coefficients':
             POSITION_FEEDBACK['native_stream'].get('locked_p_coefficients', {}),
             'input_sha256': provenance,
             'validation_scope': ('relative motion only; absolute base unvalidated'
                                  if args.relative_only else 'absolute marker alignment'),
             'minimum_marker_short_edge_px': 30 if args.relative_only else 48}
    camera = TableLink(args.host, args.remote_root, output/'camera',
                       server_args=['--camera-only', '--marker-only', '--stream-fps', '10'])
    link = Link(args.host, args.remote_root, 'hardware', hardware_control=True,
                control_log=trace)
    initial = None
    target = None
    last_ping = last_marker = 0.
    last_seq = camera_seq = None
    phase = None
    require_marker = False
    error_since = None
    marker_count = 0

    def command(op, **payload):
        return checked_command(link, proof['acks'], op, **payload).get('result')

    def observe(settled=False):
        nonlocal last_ping, last_marker, last_seq, camera_seq, error_since, marker_count
        now = time.monotonic()
        _, state, received, _, _ = link.snapshot()
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or 'motor worker exited')
        if state is None:
            return None
        if now-received > .5 or state['control']['last_fault']:
            raise RuntimeError('stale motor feedback or controller fault')
        if now-last_ping > .1:
            link.send('ping')
            last_ping = now
        if initial and abs(state['raw']['gripper']-initial['raw']['gripper']) > 4:
            raise RuntimeError('uncommanded gripper moved')
        if target:
            error = max(abs(state['raw'][j]-target[j]) for j in ACTIVE)
            error_since = (error_since or now) if error > 12 else None
            if error_since and now-error_since > .75:
                raise RuntimeError('joint tracking error exceeded 12 counts for 750 ms')
        if last_seq != state['seq']:
            last_seq = state['seq']
            proof['samples'].append({'phase': phase, 'settled': settled, 'state': state})
        _, packet, received, _, failure = camera.snapshot()
        if failure or camera.process.poll() is not None:
            raise RuntimeError(failure or 'camera worker exited')
        if packet:
            image = packet[0]
            age = now-received+image.get('capture_age_at_send_ms', 1000)/1000
            marker = image.get('tool_marker', {})
            valid = (image.get('marker_visible') and marker.get('position_valid')
                     and marker.get('id') == 8 and marker.get('source_short_edge_px', 0)
                     >= proof['minimum_marker_short_edge_px'] and age < .5)
            if valid:
                last_marker = now-age
            if camera_seq != image['sequence']:
                camera_seq = image['sequence']
                if valid:
                    marker_count += 1
        if require_marker and now-last_marker > .8:
            raise RuntimeError('fresh decoded ID 8 lost for 800 ms')
        return state

    try:
        deadline = time.monotonic()+20
        while initial is None:
            initial = observe()
            if time.monotonic() > deadline:
                raise TimeoutError('initial motor feedback absent')
            time.sleep(.01)
        hello = link.snapshot()[0]
        if hello['control_config_sha256'] != CONTROL_CONFIG_SHA256:
            raise RuntimeError('shared control hashes differ')
        proof['initial_state'] = initial
        if any(initial['telemetry']['Status'].values()):
            raise RuntimeError('initial servo fault')
        poses = validate_plan(plan, initial['raw'], hello['raw_limits'])
        identity = {**hello['fingerprint'], 'model': 'so101', 'mjcf_sha256':
                    file_digest(MODEL)}
        calibration = AngleCalibration(ROOT/'data/calibration/follower_mujoco.json', identity)
        if hello.get('joint_calibration_sha256') != provenance['angles'] or any(
                not np.allclose([hello['joint_angle_mapping'][j][key]
                                 for key in ('zero_raw', 'radians_per_tick')],
                                [calibration.fit(j)[key]
                                 for key in ('zero_raw', 'radians_per_tick')],
                                rtol=1e-9, atol=1e-12)
                for j in JOINTS):
            raise RuntimeError('Local and remote joint mapping or calibration hashes differ')
        if not np.allclose(calibration.angles(initial['raw'], [0.]*6), initial['qpos']):
            raise RuntimeError('local and remote angle calibration differs')
        proof['fingerprint'] = identity
        deadline = time.monotonic()+12
        while marker_count < 8:
            observe()
            if time.monotonic() > deadline:
                raise TimeoutError('eight fresh marker observations required')
            time.sleep(.01)
        camera_hello = camera.snapshot()[0]
        if (camera_hello['config'] != json.loads((ROOT/'configs/tabletop.json').read_text())
                or camera_hello['control_config_sha256'] != CONTROL_CONFIG_SHA256):
            raise RuntimeError('Camera worker configuration differs from the local capture inputs')
        require_marker = True
        for joint in ACTIVE:
            command('enable_joint', joint=joint)
            observe()
        target = {j: initial['raw'][j] for j in ACTIVE}
        for phase, pose in enumerate(poses):
            previous = target.copy()
            changed = [j for j in ACTIVE if pose['raw'][j] != previous[j]]
            began = time.monotonic()
            duration = max(1., max((abs(pose['raw'][j]-previous[j]) for j in changed),
                                   default=0)/32*3.)
            while time.monotonic()-began < duration:
                fraction = smooth_fraction((time.monotonic()-began)/duration)
                for joint in changed:
                    target[joint] = round(previous[joint]+fraction*(
                        pose['raw'][joint]-previous[joint]))
                    fit = calibration.fit(joint)
                    command('joint_target', joint=joint, angle=(target[joint]-fit['zero_raw'])
                            *fit['radians_per_tick'])
                observe()
                time.sleep(.025)
            target = pose['raw'].copy()
            for joint in changed:
                fit = calibration.fit(joint)
                command('joint_target', joint=joint, angle=(target[joint]-fit['zero_raw'])
                        *fit['radians_per_tick'])
            for joint in changed:
                command('settle_joint', joint=joint)
            began = time.monotonic()
            while time.monotonic()-began < args.dwell_seconds:
                state = observe(settled=time.monotonic()-began >= args.dwell_seconds-5)
                time.sleep(.01)
            proof['poses'].append({'phase': phase, 'name': pose['name'],
                                   'target_raw': target.copy(), 'observed_raw': state['raw']})
            print(json.dumps({'event': 'calibration_pose', **proof['poses'][-1]}), flush=True)
        proof['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - hold on every unexpected fault.
        proof['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            if link.latest and link.process.poll() is None:
                proof['final_control'] = command('hold')
                proof['final_state'] = link.snapshot()[1]
        except Exception as exc:  # noqa: BLE001 - continue required worker cleanup.
            proof['cleanup_error'] = str(exc)
        # Loaded joints remain supported; the worker restores temporary RAM settings.
        failure = close_workers(link, camera)
        if failure:
            proof['cleanup_error'] = '; '.join(filter(None, [proof.get('cleanup_error'), failure]))
        (output/'result.json').write_text(json.dumps(proof, indent=2)+'\n')
        try:
            subprocess.run(['scp', args.host+':'+trace, str(output/'control.json')],
                           check=True, timeout=20)
        except (OSError, subprocess.SubprocessError) as exc:
            proof['trace_copy_error'] = str(exc)
            (output/'result.json').write_text(json.dumps(proof, indent=2)+'\n')
        print(json.dumps({key: proof[key] for key in ('completed', 'error', 'cleanup_error')
                          if key in proof}), flush=True)
    return proof['completed'] and not proof.get('cleanup_error')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--relative-only', action='store_true',
                        help='Record relative motion without a base; no absolute validation')
    parser.add_argument('--dwell-seconds', type=int, choices=(8, 15), default=8,
                        help='Observation dwell; changes neither gains nor trajectory speed')
    add_remote_arguments(parser)
    args = parser.parse_args()
    if not args.execute:
        parser.error('physical pose acquisition requires --execute')
    raise SystemExit(0 if acquire(args) else 1)


if __name__ == '__main__':
    main()
