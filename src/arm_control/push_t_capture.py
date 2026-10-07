"""One fresh remote, read-only observation for a subsequent local Push-T rollout."""

import copy
import hashlib
import json
import math
import shutil
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from .angle_calibration import effective_scale, validate_correction
from .tabletop_client import ROOT, TableLink, atomic_json
from .tabletop_scene import scene_xml

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


def session_name():
    return datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d-%H%M%S-%f')


def finite_vector(value, length):
    return (isinstance(value, (list, tuple)) and len(value) == length
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value))


def observation_problem(state, arrival_age, output):
    """Check freshness using durations, without assuming synchronized machine clocks."""
    if state.get('units') != 'm' or not state.get('marker_visible'):
        return 'waiting for metric table localization'
    age = state.get('capture_age_at_send_ms', math.inf)/1000 + arrival_age
    if not math.isfinite(age) or age > .5:
        return 'camera observation older than 500 ms'
    for name in ('black_t', 'wood_t'):
        obj = state.get('objects', {}).get(name, {})
        if (not obj.get('observed') or not obj.get('pose_valid')
                or obj.get('tracking_status') not in ('observed', 'partial')):
            return f'waiting for a current visible {name}; cached poses are not initialization'
        if (not finite_vector(obj.get('xy'), 2) or not math.isfinite(obj.get('yaw', math.inf))
                or obj.get('observation_age_s', math.inf) > .5):
            return f'invalid or stale {name} pose'
    robot = state.get('robot', {})
    if robot.get('status') != 'live' or robot.get('read_only') is not True:
        return 'waiting for live read-only joint feedback'
    motor_age = robot.get('sample_age_at_send_ms', math.inf)/1000 + arrival_age
    if not math.isfinite(motor_age) or motor_age > .5 or not finite_vector(robot.get('qpos'), 6):
        return 'invalid or stale joint feedback'
    base = robot.get('registration', {})
    fixed = (base.get('status') == 'fixed' and base.get('fixed_to_table') is True
             and base.get('calibrated') is True
             and base.get('pose_source') == 'fixed_marker_calibration')
    if ((not fixed and base.get('status') not in (
            'visual_estimate', 'visual_cached', 'tracked', 'held'))
            or not finite_vector(base.get('position'), 3)
            or not finite_vector(base.get('quaternion'), 4)
            or abs(sum(v*v for v in base['quaternion'])-1) > .01
            or (not fixed and base.get('pose_age_s', math.inf) > 10)):
        return 'waiting for the robot base registration'
    if not state.get('appearance', {}).get('version') or not (output/'appearance').is_dir():
        return 'waiting for textures and lighting from this capture session'
    return None


def calibration_bytes(hello, path):
    """Match the saved encoder calibration to the remote read-only identity and fits."""
    raw = path.read_bytes()
    saved = json.loads(raw)
    remote = hello.get('robot', {})
    if (remote.get('read_only') is not True or saved.get('version') != 1
            or saved.get('fingerprint') != remote.get('fingerprint')):
        raise ValueError('Saved local joint calibration differs from remote hardware identity')
    correction = remote.get('kinematic_correction')
    offsets = (validate_correction(correction, hashlib.sha256(raw).hexdigest(),
                                   saved['fingerprint'])
               if correction else dict.fromkeys(JOINTS, 0))
    for joint in JOINTS:
        points = saved['points'][joint]
        zero, thirty, sixty = (points[str(angle)]['raw'] for angle in (0, 30, 60))
        span = thirty-zero
        if not .9*(4096/12) <= abs(span) <= 1.1*(4096/12):
            raise ValueError(f'{joint}: invalid saved encoder calibration')
        scale = math.radians(30)/span
        error = abs((sixty-zero)*scale-math.radians(60))
        scale = effective_scale(joint, scale, correction)
        fit = remote.get('fits', {}).get(joint, {})
        if (error > math.radians(2) or not math.isclose(fit.get('zero_raw', math.inf),
                zero-offsets[joint]/scale, rel_tol=1e-12, abs_tol=1e-12)
                or not math.isclose(fit.get('radians_per_tick', math.inf), scale,
                                    rel_tol=1e-12, abs_tol=1e-12)):
            raise ValueError(f'{joint}: local and remote joint calibrations differ')
    return raw


def capture_snapshot(output, *, host='lerobot', remote_root='/home/xy/Research/lerobot',
                     timeout=90, calibration=ROOT/'data/calibration/follower_mujoco.json'):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Capture timeout must be positive and finite')
    output = Path(output).resolve()
    link = TableLink(host, remote_root, output)
    started, last_report = time.monotonic(), -math.inf
    selected = hello = None
    reason = 'waiting for the new remote camera session'
    try:
        while time.monotonic()-started < timeout:
            metadata, latest, received_at, count, error = link.snapshot()
            if error:
                raise RuntimeError(error)
            if link.process.poll() is not None:
                raise RuntimeError('Remote capture ended before a valid initialization arrived')
            if latest is not None:
                state = latest[0]
                arrival_age = time.monotonic()-received_at
                reason = observation_problem(state, arrival_age, output)
                if reason is None:
                    calibration_data = calibration_bytes(metadata, calibration)
                    selected, hello = copy.deepcopy(state), copy.deepcopy(metadata)
                    camera_age = state['capture_age_at_send_ms']/1000+arrival_age
                    motor_age = state['robot']['sample_age_at_send_ms']/1000+arrival_age
                    break
            if time.monotonic()-last_report > 2:
                print(json.dumps({'capture': reason, 'frames': count}), flush=True)
                last_report = time.monotonic()
            time.sleep(.02)
        if selected is None:
            raise TimeoutError(f'Fresh initialization timed out: {reason}. '
                               f'Capture saved at {output}')
    finally:
        # No connection remains while simulating. Remote disconnect preserves motor torque.
        link.close()
    initialization = {
        'kind': 'fresh_remote_observation', 'snapshot': str(output),
        'source_host': host, 'sequence': selected['sequence'],
        'captured_unix_ns': selected['captured_unix_ns'],
        'camera_age_at_selection_s': camera_age, 'motor_age_at_selection_s': motor_age,
        'real_robot_read_only': True, 'remote_closed_before_simulation': True,
        'base_calibrated': selected['robot']['registration'].get('calibrated', False),
        'joint_calibration_sha256': hashlib.sha256(calibration_data).hexdigest(),
        'object_tracking': {name: selected['objects'][name]['tracking_status']
                            for name in ('black_t', 'wood_t')},
    }
    selected['initialization'] = initialization
    atomic_json(output/'state.json', selected)
    atomic_json(output/'calibration.json', hello)
    atomic_json(output/'initialization.json', initialization)
    (output/'joint_calibration.json').write_bytes(calibration_data)
    for key, filename in (('frame', 'frame.jpg'), ('annotated', 'annotated.jpg'),
                          ('native_frame', 'native_frame.jpg')):
        if key in selected:
            shutil.copyfile(output/selected[key], output/filename)
    (output/'scene.xml').write_text(scene_xml(selected, hello['config'], 'robot_assets'))
    print(json.dumps({'initialization': initialization,
                      'objects': {name: {'xy': selected['objects'][name]['xy'],
                                         'yaw': selected['objects'][name]['yaw']}
                                  for name in ('black_t', 'wood_t')},
                      'qpos': selected['robot']['qpos']}), flush=True)
    return output
