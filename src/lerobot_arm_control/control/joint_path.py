"""Finite five-joint paths evaluated on the motor worker's clock; no simulation."""

import hashlib
import json
import math

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, TRAJECTORY
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.robot.so101.joints import JOINTS

ACTIVE = JOINTS[:5]


def plan_digest(plan):
    return hashlib.sha256(json.dumps(plan, sort_keys=True, allow_nan=False).encode()).hexdigest()


class JointPath:
    def __init__(self):
        self.phase = 'ready'
        self.segment = 0
        self.segments = []
        self.error_since = None

    @property
    def active(self):
        return self.phase in ('moving', 'dwelling')

    def cancel(self, reason):
        if self.active:
            self.phase = 'stopped: '+reason

    def start(self, control, plan, validation):
        if control.parking.busy or (control.trajectory and control.trajectory.active):
            raise ValueError('another automatic motion is running')
        if not isinstance(plan, dict) or plan.get('version') != 1:
            raise ValueError('invalid joint path version')
        initial = plan.get('initial_raw', {})
        if set(initial) != set(JOINTS) or any(type(v) is not int for v in initial.values()):
            raise ValueError('joint path requires six integer initial encoder positions')
        expected = {'checked': True, 'plan_sha256': plan_digest(plan),
                    'control_config_sha256': CONTROL_CONFIG_SHA256,
                    'model_sha256': control.calibration.fingerprint['mjcf_sha256'],
                    'joint_calibration_sha256': file_digest(control.calibration.path)}
        if validation != expected:
            raise ValueError('joint path requires matching local collision validation and inputs')
        points = plan.get('waypoints', [])
        if not 1 <= len(points) <= 20:
            raise ValueError('joint path requires 1..20 finite waypoints')
        previous = {j: control.angle(j, initial[j]) for j in ACTIVE}
        segments = []
        for point in points:
            raw, dwell = point.get('raw', {}), point.get('dwell_seconds')
            if (set(raw) != set(ACTIVE) or any(type(v) is not int for v in raw.values())
                    or type(dwell) not in (int, float) or not math.isfinite(dwell)
                    or not 1 <= dwell <= 5):
                raise ValueError('waypoints require five integer targets and 1..5 second dwell')
            for joint in ACTIVE:
                low, high = control.raw_limits[joint]
                if not low+8 <= raw[joint] <= high-8 or abs(raw[joint]-initial[joint]) > 400:
                    raise ValueError('joint path exceeds physical limits or 400-count envelope')
            target = {j: control.angle(j, raw[j]) for j in ACTIVE}
            duration = max(TRAJECTORY['min_entry_seconds'], 1.875*max(
                abs(target[j]-previous[j]) for j in ACTIVE)/TRAJECTORY['entry_speed_rad_s'])
            segments.append({'start': previous, 'end': target, 'seconds': duration,
                             'dwell_seconds': dwell})
            previous = target
        if sum(s['seconds']+s['dwell_seconds'] for s in segments) > 120:
            raise ValueError('joint path exceeds 120 seconds')
        # All checks, including fresh readback, precede every enable/goal write.
        for joint in ACTIVE:
            control.preflight(joint)
        fresh = control.bus.sync_read('Present_Position', normalize=False)
        if any(abs(fresh[j]-initial[j]) > 4 for j in JOINTS):
            raise ValueError('arm moved after local collision validation')
        for joint in ACTIVE:
            control.enable(joint)
        segments[0]['start'] = {j: control.angle(j, control.measured[j]) for j in ACTIVE}
        self.control, self.initial = control, initial
        self.segments, self.segment = segments, 0
        self.began = self.segment_began = control.clock()
        self.phase, self.error_since = 'moving', None
        self.settled_reference = False
        control.trajectory = self

    def advance_hardware(self, positions, telemetry):
        if not self.active:
            return
        control, now = self.control, self.control.clock()
        if any(j not in control.enabled or j in control.stopped for j in ACTIVE):
            control.stop('joint path lost enabled joint')
            return
        if abs(positions['gripper']-self.initial['gripper']) > 4:
            control.stop('uncommanded gripper moved')
            return
        segment = self.segments[self.segment]
        elapsed = now-self.segment_began
        fraction = smooth_fraction(min(1., elapsed/segment['seconds']))
        references = {j: segment['start'][j]+fraction*(
            segment['end'][j]-segment['start'][j]) for j in ACTIVE}
        error = max(abs(control.raw(j, references[j])-positions[j]) for j in ACTIVE)
        self.error_since = ((now if self.error_since is None else self.error_since)
                            if error > 80 else None)
        if self.error_since is not None and now-self.error_since > .4:
            control.stop('joint path tracking error over 80 counts for 400 ms')
            return
        control.targets.update(references)
        if elapsed >= segment['seconds']:
            self.phase = 'dwelling'
            if not self.settled_reference:
                control.position_feedback.finish_reference(ACTIVE)
                self.settled_reference = True
        if elapsed >= segment['seconds']+segment['dwell_seconds']:
            if self.segment+1 == len(self.segments):
                self.phase = 'completed'
                control.stop('joint path completed: hold')
            else:
                self.segment += 1
                self.segment_began = now
                self.phase, self.settled_reference = 'moving', False

    def status(self):
        return {'phase': self.phase, 'segment': self.segment,
                'elapsed_seconds': self.control.clock()-self.began if self.segments else 0.,
                'segment_elapsed_seconds': (self.control.clock()-self.segment_began
                                            if self.segments else 0.),
                'segments': self.segments, 'control_config_sha256': CONTROL_CONFIG_SHA256}
