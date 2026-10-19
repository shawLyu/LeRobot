"""Remote-only bounded joint streaming. The feedback worker exclusively owns the bus."""

import copy
import math
import threading
import time

import numpy as np

from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.tasks.circle.circle_trajectory import load_profile
from lerobot_arm_control.transport.realtime import JOINTS

ROOT = workspace_root()
ENTRY_PROFILE, ENTRY_PROFILE_HASH = load_profile()


def calibrated_base(base):
    return base.get('calibrated') is True and (
        base.get('pose_source') == 'known_marker_mounts' or (
            base.get('status') == 'fixed' and base.get('fixed_to_table') is True
            and base.get('pose_source') == 'fixed_marker_calibration'))


def observation_issue(state, received, now, *, require_base=False, initialized=False):
    if state is None:
        return 'waiting for a camera observation'
    camera_age = now-received+state.get('capture_age_at_send_ms', math.inf)/1000
    if (state.get('units') != 'm' or not state.get('marker_visible')
            or not math.isfinite(camera_age) or camera_age > .5):
        return 'table localization lost or camera older than 500 ms'
    for name in ('wood_t', 'black_t'):
        obj = state.get('objects', {}).get(name, {})
        cached_target = initialized and name == 'black_t' and (
            obj.get('tracking_status') == 'static_cached')
        visible = obj.get('observed') and obj.get('tracking_status') in ('observed', 'partial')
        if not obj.get('pose_valid') or not (cached_target or visible):
            return f'{name} has no current valid observation'
        if not np.isfinite([*obj.get('xy', [math.inf]*2), obj.get('yaw', math.inf)]).all():
            return f'{name} pose is not finite'
    robot = state.get('robot', {})
    motor_age = now-received+robot.get('sample_age_at_send_ms', math.inf)/1000
    if (robot.get('status') != 'live' or len(robot.get('qpos', [])) != 6
            or not np.isfinite(robot.get('qpos', [])).all()
            or not math.isfinite(motor_age) or motor_age > .5):
        return 'joint feedback lost or stale'
    base = robot.get('registration', {})
    fixed = (base.get('status') == 'fixed' and base.get('fixed_to_table') is True
             and base.get('calibrated') is True
             and base.get('pose_source') == 'fixed_marker_calibration')
    if (len(base.get('position', [])) != 3 or len(base.get('quaternion', [])) != 4
            or not np.isfinite([*base.get('position', []), *base.get('quaternion', [])]).all()
            or base.get('status') not in (
                'tracked', 'held', 'visual_estimate', 'visual_cached', 'fixed')
            or (not fixed and base.get('pose_age_s', 0) > (1 if require_base else 10))):
        return 'robot base registration missing or stale'
    if require_base and not fixed and (not base.get('calibrated') or
                                      base.get('pose_source') != 'known_marker_mounts'):
        return 'real execution requires measured marker-to-base mounts; visual estimate only'
    return None


def same_scene(reference, current, *, wood_tolerance=.003, joint_tolerance=.035):
    a, b = reference['robot']['registration'], current['robot']['registration']
    if (np.linalg.norm(np.subtract(a['position'], b['position'])) > .002
            or abs(np.dot(a['quaternion'], b['quaternion'])) < math.cos(.015)):
        return False
    if np.max(np.abs(np.subtract(reference['robot']['qpos'], current['robot']['qpos']))) > (
            joint_tolerance):
        return False
    for name in ('wood_t', 'black_t'):
        a, b = reference['objects'][name], current['objects'][name]
        yaw = math.atan2(math.sin(a['yaw']-b['yaw']), math.cos(a['yaw']-b['yaw']))
        if np.linalg.norm(np.subtract(a['xy'], b['xy'])) > wood_tolerance or abs(yaw) > .1:
            return False
    return True


def recovery_issue(state, received, now):
    """Visual landmarks may be hidden during retreat; fresh images and encoders are required."""
    if state is None:
        return 'no camera frame during visibility recovery'
    age = now-received+state.get('capture_age_at_send_ms', math.inf)/1000
    robot = state.get('robot', {})
    motor_age = now-received+robot.get('sample_age_at_send_ms', math.inf)/1000
    if not math.isfinite(age) or age > .5:
        return 'camera stream stale during visibility recovery'
    if (robot.get('status') != 'live' or not math.isfinite(motor_age) or motor_age > .5
            or len(robot.get('qpos', [])) != 6 or not np.isfinite(robot['qpos']).all()):
        return 'motor feedback stale during visibility recovery'
    return None


def joint_segments(initial, action, limits, *, speed=.15, push_speed=.004,
                   entry_tolerance=0.):
    """Circle's quintic entry timing; slower, bounded contact waypoints for pushing."""
    start = np.asarray(initial, float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError('invalid measured joint seed')
    if joint_limit_issue(start, limits, tolerance=entry_tolerance):
        raise ValueError('measured joint seed violates calibrated CAD joint limits')
    segments = []
    for index, value in enumerate(action.joints):
        end = np.asarray(value, float)
        if end.shape != (6,) or not np.isfinite(end).all():
            raise ValueError('invalid six-joint push waypoint')
        if np.any(end < limits[:, 0]+.003) or np.any(end > limits[:, 1]-.003):
            raise ValueError('push waypoint violates calibrated CAD joint limits')
        entering = index < action.approach_count
        joint_speed = ENTRY_PROFILE['entry_speed_rad_s'] if entering else speed
        minimum = ENTRY_PROFILE['min_entry_seconds'] if entering else .06
        duration = max(minimum, 1.875*float(np.max(np.abs(end-start)))/joint_speed)
        if index >= action.approach_count:
            duration = max(duration, action.distance/(len(action.joints)-action.approach_count)
                           /push_speed)
        segments.append({'start': start.tolist(), 'end': end.tolist(), 'seconds': duration,
                         'speed_rad_s': joint_speed})
        start = end
    if not segments:
        raise ValueError('empty push trajectory')
    return segments


def joint_limit_issue(initial, limits, *, tolerance=0.):
    """Keep observing an invalid physical seed without clipping measured encoder angles."""
    seed = np.asarray(initial, float)
    limits = np.asarray(limits, float)
    if seed.shape != (6,) or not np.isfinite(seed).all():
        return 'invalid measured joint seed'
    if not math.isfinite(tolerance) or not 0 <= tolerance <= math.radians(5):
        raise ValueError('invalid measured-start tolerance')
    outside = (seed < limits[:, 0]-tolerance) | (seed > limits[:, 1]+tolerance)
    violations = [f'{JOINTS[i]}={math.degrees(seed[i]):.2f} deg '
                  f'outside [{math.degrees(limits[i, 0]):.2f}, '
                  f'{math.degrees(limits[i, 1]):.2f}] deg'
                  for i in np.flatnonzero(outside)]
    return '; '.join(violations) if violations else None


class PushMotor:
    """Submissions cross threads; all preflight, enable, writes and hold run on the bus owner."""
    def __init__(self, bus, calibration, model_ranges, raw_limits, *, clock=time.monotonic,
                 task='push_t'):
        self.control = ManualControl(bus, calibration, raw_limits, clock=clock)
        if self.control.position_feedback:
            self.control.position_feedback.limits = {
                joint: (max(self.control.limits[joint][0], model_ranges[i][0]),
                        min(self.control.limits[joint][1], model_ranges[i][1]))
                for i, joint in enumerate(JOINTS)}
        self.clock = clock
        self.task = task
        self.lock = threading.Lock()
        self.observation, self.observed_at = None, 0
        self.pending, self.stop_reason = None, None
        self.job = None
        self.progress = {'phase': 'idle', 'writes_enabled': False}

    def observe(self, state, received):
        with self.lock:
            self.observation, self.observed_at = state, received

    def submit(self, state, segments, *, recovery=False, stage=None):
        with self.lock:
            if self.pending or self.job:
                raise RuntimeError('another push is active')
            self.pending = (copy.deepcopy(state), copy.deepcopy(segments), recovery, stage)
            self.progress = {'phase': 'pending', 'writes_enabled': False}

    def cancel(self, reason):
        with self.lock:
            self.stop_reason = reason
            self.progress = {**self.progress, 'phase': 'stopping'}

    def status(self):
        with self.lock:
            return dict(self.progress)

    def _status(self, phase, **fields):
        with self.lock:
            self.progress = {'phase': phase, 'writes_enabled': bool(self.control.enabled), **fields}

    def _hold(self, reason):
        self.job = None
        with self.lock:
            self.pending = None
        if self.control.enabled:
            self.control.stop(reason)
        self._status('stopped', reason=reason)

    def tick(self, positions, telemetry):
        with self.lock:
            observation, received = self.observation, self.observed_at
            pending, self.pending = self.pending, None
            stop, self.stop_reason = self.stop_reason, None
        if stop:
            self._hold(stop)
            return
        if not self.job and not pending:
            return
        recovery = pending[2] if pending else self.job.get('recovery', False)
        stage = pending[3] if pending else self.job.get('stage')
        if self.task == 'pick_t':
            from lerobot_arm_control.transport.pick_t_protocol import pick_observation_issue

            reference = pending[0] if pending else self.job['reference']
            issue = pick_observation_issue(observation, received, self.clock(), stage, reference)
        else:
            issue = (recovery_issue(observation, received, self.clock()) if recovery else
                     observation_issue(observation, received, self.clock(), require_base=True,
                                       initialized=True))
        if issue:
            self._hold(issue)
            return
        if pending:
            reference, segments, recovery, stage = pending
            base = reference['robot']['registration']
            if not calibrated_base(base):
                self._hold('no calibrated base reference for real motion')
                return
            if self.task == 'pick_t':
                from lerobot_arm_control.transport.pick_t_protocol import (
                    CARRYING,
                    stage_segments_issue,
                )

                issue = stage_segments_issue(stage, segments)
                if issue:
                    self._hold(issue)
                    return
                scene = copy.deepcopy(observation)
                if stage in CARRYING or (stage in ('descend', 'close') and
                                        not scene['objects'].get('wood_t', {}).get('observed')):
                    scene['objects']['wood_t'] = reference['objects']['wood_t']
            else:
                scene = observation
            if not recovery and not same_scene(reference, scene):
                self._hold('scene changed before motor preflight')
                return
            fresh = [self.control.angle(joint, positions[joint]) for joint in JOINTS]
            differences = [abs(a-b) for a, b in zip(fresh, segments[0]['start'])]
            carrying = self.task == 'pick_t' and stage in ('lift', 'transfer', 'lower', 'open')
            if max(differences[:5]) > .035 or differences[5] > (.08 if carrying else .035):
                self._hold('arm moved before motor preflight')
                return
            # Check every waypoint before any goal or torque write, including raw limits.
            for segment in segments:
                for joint, value in zip(JOINTS, segment['end']):
                    low, high = self.control.limits[joint]
                    if not math.isfinite(value) or not low <= value <= high:
                        raise ValueError(f'{joint}: motor waypoint outside encoder limits')
            self.control.enable_all()
            segments[0]['start'] = [self.control.targets[j] for j in JOINTS]
            segments[0]['seconds'] = max(segments[0]['seconds'], 1.875*max(
                abs(a-b) for a, b in zip(segments[0]['start'], segments[0]['end']))
                /segments[0].get('speed_rad_s', .15))
            telemetry = {**telemetry, **{key: self.control.bus.sync_read(key, normalize=False)
                                        for key in ('Status', 'Torque_Enable')}}
            self.job = {'reference': reference, 'segments': segments, 'index': 0,
                        'started': self.clock(), 'job_started': self.clock(), 'recovery': recovery,
                        'stage': stage}
        job = self.job
        if recovery and self.clock()-job['job_started'] > 8:
            self._hold('visibility recovery exceeded 8 seconds')
            return
        base_now = observation['robot'].get('registration', {})
        base_start = job['reference']['robot']['registration']
        if base_now.get('calibrated') and (
                np.linalg.norm(np.subtract(base_now['position'], base_start['position'])) > .002
                or abs(np.dot(base_now['quaternion'], base_start['quaternion'])) < math.cos(.015)):
            self._hold('robot base moved during push')
            return
        target_now = observation.get('objects', {}).get('black_t', {})
        target_start = job['reference']['objects']['black_t']
        if target_now.get('observed') and np.linalg.norm(np.subtract(
                target_now['xy'], target_start['xy'])) > .003:
            self._hold('target moved during push')
            return
        if self.task == 'pick_t' and target_now.get('observed'):
            yaw_delta = target_now['yaw']-target_start['yaw']
            if abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))) > .05:
                self._hold('placement target rotated during pick/place')
                return
        segment = job['segments'][job['index']]
        elapsed = self.clock()-job['started']
        f = min(1., elapsed/segment['seconds'])
        f = smooth_fraction(f)
        target = np.asarray(segment['start'])+f*np.subtract(segment['end'], segment['start'])
        actual = np.array([self.control.angle(joint, positions[joint]) for joint in JOINTS])
        error = float(np.max(np.abs(target-actual)))
        if error > .12:
            self._hold(f'joint tracking error {error:.3f} rad exceeds 0.12 rad')
            return
        if (any(telemetry['Status'].values())
                or not all(telemetry['Torque_Enable'].values())):
            # Confirm an adverse bulk sample directly; enabling may have changed torque
            # since that sample was taken. Genuine faults still prevent every goal write.
            telemetry = {**telemetry, **{key: self.control.bus.sync_read(key, normalize=False)
                                        for key in ('Status', 'Torque_Enable')}}
            faults = {j: value for j, value in telemetry['Status'].items() if value}
            disabled = [j for j, value in telemetry['Torque_Enable'].items() if not value]
            if faults or disabled:
                self._hold(f'motor fault or lost torque: Status={faults}; disabled={disabled}')
                return
        self.control.targets.update(zip(JOINTS, target))
        self.control.heartbeat()
        self.control.step(positions, telemetry)
        if self.control.stopped:
            self._hold('position controller stopped')
            return
        self._status('running', waypoint=job['index'], tracking_error_rad=error, recovery=recovery)
        if elapsed >= segment['seconds']:
            closing = self.task == 'pick_t' and stage == 'close'
            gripping = self.task == 'pick_t' and stage in ('close', 'lift', 'transfer', 'lower')
            arm_error = float(np.max(np.abs(target[:5]-actual[:5])))
            arrived = (arm_error <= .02 and 0 <= actual[5]-target[5] <= .08
                       if gripping else error <= .02)
            if not arrived:
                if elapsed > segment['seconds']+3:
                    self._hold('waypoint not reached within 3 seconds')
                return
            if closing and elapsed < segment['seconds']+.6:
                return
            if closing and actual[5]-target[5] < .006:
                self._hold('empty grasp: jaw reached closed target without contact')
                return
            job['index'] += 1
            job['started'] = self.clock()
            if job['index'] == len(job['segments']):
                self.control.stop('push completed: hold at contact height')
                self.job = None
                self._status('completed')

    def close(self):
        self._hold('remote session closed: hold')
        self.control.close()
