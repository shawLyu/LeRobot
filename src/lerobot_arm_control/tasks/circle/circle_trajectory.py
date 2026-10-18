"""Validated Cartesian circle and worker-clock playback for the SO-101."""

import json
import math
from copy import copy, deepcopy
from pathlib import Path

from lerobot_arm_control.control.control_defaults import (
    CIRCLE_FEEDBACK_PROFILES,
    CONTROL_CONFIG_SHA256,
    CONTROL_HZ,
    POSITION_FEEDBACK,
    TRAJECTORY,
)
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.transport.realtime import JOINTS

ROOT = workspace_root()
PLAN = resource_path("data/trajectories/circle.json")


def load_profile():
    profile = deepcopy(TRAJECTORY)
    keys = ("entry_speed_rad_s", "min_entry_seconds", "ramp_seconds",
            "reset_speed_rad_s", "period_seconds", 'max_circle_angular_speed_deg_s',
            'max_circle_joint_speed_rad_s', 'max_circle_joint_acceleration_rad_s2')
    if (profile.get("version") != 1 or profile.get("control_hz") != CONTROL_HZ
            or any(type(profile.get(key)) not in (int, float)
                   or not math.isfinite(profile[key]) or profile[key] <= 0 for key in keys)):
        raise ValueError("invalid shared circle control profile")
    return profile, CONTROL_CONFIG_SHA256


class Circle:
    def __init__(self, path=PLAN, *, simulation=False):
        self.simulation = simulation
        profile, self.profile_hash = load_profile()
        self.entry_speed = profile["entry_speed_rad_s"]
        self.ramp_seconds = profile["ramp_seconds"]
        self.min_entry_seconds = profile["min_entry_seconds"]
        self.control_hz = profile["control_hz"]
        saved = json.loads(Path(path).read_text())
        if (saved.get("version") != 1 or saved.get("joints") != list(JOINTS)
                or saved.get("model_sha256") != file_digest(
                    resource_path('assets/so101/so101.xml'))):
            raise ValueError("circle identity does not match the model")
        self.plan_hash = file_digest(path)
        self.model_hash = saved["model_sha256"]
        self.points = saved["qpos"]
        self.period = profile["period_seconds"]
        # Faster circle phase is an explicit, bounded request in the hashed
        # plan. Entry speed and ramp retain the shared smoothness limits.
        timing = saved.get('execution_profile', {})
        if (not isinstance(timing, dict)
                or set(timing) - {'entry_speed_rad_s', 'ramp_seconds', 'angular_speed_deg_s',
                                 'controller_profile'}):
            raise ValueError('invalid circle execution profile')
        for key, value in timing.items():
            if key == 'controller_profile':
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid circle execution profile')
        if (timing.get('entry_speed_rad_s', self.entry_speed) > self.entry_speed
                or timing.get('ramp_seconds', self.ramp_seconds) < self.ramp_seconds):
            raise ValueError('circle plan may only slow the shared profile')
        self.entry_speed = timing.get('entry_speed_rad_s', self.entry_speed)
        self.ramp_seconds = timing.get('ramp_seconds', self.ramp_seconds)
        self.requested_angular_speed = timing.get('angular_speed_deg_s')
        selected = timing.get('controller_profile')
        if selected is not None and (not isinstance(selected, str)
                                     or selected not in CIRCLE_FEEDBACK_PROFILES):
            raise ValueError('unknown circle controller profile')
        self.feedback_profile = deepcopy(
            CIRCLE_FEEDBACK_PROFILES[selected] if selected else POSITION_FEEDBACK)
        self.motion_budget = {
            'max_joint_speed_rad_s': profile['max_circle_joint_speed_rad_s'],
            'max_joint_acceleration_rad_s2': profile['max_circle_joint_acceleration_rad_s2'],
        }
        self.plane = saved.get('plane', 'XZ')
        self.table_z = saved.get('table_z_m')
        self.calibration_hash = saved.get('joint_calibration_sha256')
        self.execution_ready = saved.get('execution_ready', True)
        if type(self.execution_ready) is not bool:
            raise ValueError('invalid circle execution readiness')
        self.center = saved["center_m"]
        self.radius = saved["radius_m"]
        if (not isinstance(self.center, list) or len(self.center) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       for v in self.center)
                or type(self.radius) not in (int, float)
                or not math.isfinite(self.radius) or not 0 < self.radius < 1):
            raise ValueError('invalid circle geometry')
        self.phase = "ready"
        self.started = None
        if (type(saved["period_seconds"]) not in (int, float)
                or not math.isfinite(saved["period_seconds"]) or saved["period_seconds"] <= 0
                or type(self.period) not in (int, float) or not math.isfinite(self.period)
                or self.period <= 0 or len(self.points) < 100
                or any(len(q) != 6 or any(type(v) not in (int, float) or not math.isfinite(v)
                                         for v in q) for q in self.points)):
            raise ValueError("invalid circle samples")
        if self.points[0] != self.points[-1]:
            raise ValueError("circle must close")
        self.period = max(self.period, saved['period_seconds'])
        if self.requested_angular_speed is not None:
            if (self.plane != 'XY'
                    or self.requested_angular_speed > profile['max_circle_angular_speed_deg_s']
                    or not math.isclose(saved['period_seconds'],
                                        360/self.requested_angular_speed, abs_tol=1e-9)):
                raise ValueError('circle angular speed or period exceeds the reviewed profile')
            self.period = saved['period_seconds']
        if self.ramp_seconds > self.period and self.requested_angular_speed is None:
            raise ValueError('circle ramp cannot exceed the lap period')
        if (self.plane not in ('XY', 'XZ')
                or self.plane == 'XY' and (
                    type(self.table_z) not in (int, float)
                    or not math.isfinite(self.table_z) or not -.05 <= self.table_z <= 0
                    or not isinstance(self.calibration_hash, str)
                    or len(self.calibration_hash) != 64
                    or any(q[4:] != self.points[0][4:] for q in self.points))):
            raise ValueError('horizontal circle requires a table plane in base coordinates')
        peak = max(abs(b - a) * (len(self.points) - 1) / self.period
                   for left, right in zip(self.points, self.points[1:])
                   for a, b in zip(left, right))
        self.planned_peak_speed = peak
        if (self.requested_angular_speed is not None
                and peak > self.motion_budget['max_joint_speed_rad_s']):
            raise ValueError('circle exceeds the shared joint speed budget')

    def check_motion_budget(self, initial):
        """Check the actual sampled entry/ramp/lap commands before any enable."""
        if self.requested_angular_speed is None:
            return None
        trial = copy(self)
        # Include a complete constant-speed lap even when the gentle ramp lasts
        # longer than one revolution. Finite execution checks overlap separately.
        laps = math.ceil(self.ramp_seconds/self.period)+1
        trial.start(initial, 0, laps=laps)
        duration = trial.entry_seconds + laps*trial.period + trial.ramp_seconds
        dt = 1/self.control_hz
        previous = list(initial)
        velocity = [0.]*len(JOINTS)
        peak_speed = peak_acceleration = 0.
        for tick in range(1, math.ceil(duration/dt)+2):
            target = trial.sample(tick*dt)
            updated = [(b-a)/dt for a, b in zip(previous, target)]
            peak_speed = max(peak_speed, *(abs(v) for v in updated))
            peak_acceleration = max(peak_acceleration, *(abs(b-a)/dt
                                    for a, b in zip(velocity, updated)))
            previous, velocity = target, updated
        if (peak_speed > self.motion_budget['max_joint_speed_rad_s']
                or peak_acceleration > self.motion_budget['max_joint_acceleration_rad_s2']):
            raise ValueError('circle entry/ramp exceeds the shared speed/acceleration budget')
        return {'peak_joint_speed_rad_s': peak_speed,
                'peak_control_frame_acceleration_rad_s2': peak_acceleration}

    @property
    def active(self):
        return self.phase in ("entering", "running")

    def validate_laps(self, laps):
        if laps is not None and (type(laps) is not int or laps <= 0):
            raise ValueError("laps must be a positive integer")
        if laps is not None and self.ramp_seconds > laps*self.period:
            raise ValueError('finite circle is too short for non-overlapping speed ramps')

    def start(self, initial, now, *, laps=None):
        self.validate_laps(laps)
        self.laps = laps
        self.initial = list(initial)
        self.entry_seconds = max(self.min_entry_seconds, 1.875 * max(
            abs(a - b) for a, b in zip(initial, self.points[0])) / self.entry_speed)
        self.started = now
        self.phase = "entering"

    def cycles(self, elapsed):
        # Integrate a quintic angular-speed ramp; thereafter loop at constant speed.
        ramp = self.ramp_seconds
        if elapsed < ramp:
            u = elapsed / ramp
            cycles = ramp * (u**6 - 3 * u**5 + 2.5 * u**4) / self.period
        elif self.laps is not None and elapsed >= self.laps * self.period:
            u = min(1.0, (elapsed - self.laps * self.period) / ramp)
            cycles = (self.laps - ramp / (2 * self.period)
                      + ramp * (u - (u**6 - 3 * u**5 + 2.5 * u**4)) / self.period)
        else:
            cycles = (elapsed - ramp / 2) / self.period
        return cycles

    def speed_fraction(self, now):
        elapsed = max(0., now-self.started-self.entry_seconds)
        if elapsed < self.ramp_seconds:
            return smooth_fraction(elapsed/self.ramp_seconds)
        if self.laps is not None and elapsed >= self.laps*self.period:
            return 1-smooth_fraction(min(1., (elapsed-self.laps*self.period)/self.ramp_seconds))
        return 1.

    def sample(self, now):
        elapsed = max(0.0, now - self.started)
        if elapsed < self.entry_seconds:
            f = smooth_fraction(elapsed / self.entry_seconds)
            return [a + (b - a) * f for a, b in zip(self.initial, self.points[0])]
        elapsed -= self.entry_seconds
        self.phase = ("completed" if self.laps is not None
                      and elapsed >= self.laps * self.period + self.ramp_seconds else "running")
        cycles = self.cycles(elapsed)
        index = (cycles % 1) * (len(self.points) - 1)
        low = int(index)
        f = index - low
        return [a + (b - a) * f for a, b in zip(self.points[low], self.points[low + 1])]

    def cancel(self, reason):
        if self.active:
            self.phase = "stopped: " + reason

    def start_hardware(self, control, validation, *, laps=None):
        self.validate_laps(laps)
        if self.simulation:
            raise ValueError("simulation speed profile cannot control hardware")
        if not self.execution_ready:
            raise ValueError('candidate circle has not been cleared for hardware execution')
        if control.parking.busy or self.active:
            raise ValueError("another automatic motion is running")
        if (self.calibration_hash is not None
                and file_digest(control.calibration.path) != self.calibration_hash):
            raise ValueError('circle joint calibration changed; replan and validate again')
        if (not isinstance(validation, dict) or validation.get("checked") is not True
                or validation.get("model_sha256") != self.model_hash
                or validation.get("plan_sha256") != self.plan_hash
                or validation.get("control_config_sha256") != self.profile_hash):
            raise ValueError("circle requires matching local collision validation")
        initial = validation.get("initial_qpos")
        if (not isinstance(initial, list) or len(initial) != len(JOINTS)
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in initial)):
            raise ValueError("invalid local circle entry pose")
        self.check_motion_budget(initial)
        if control.position_feedback.profile != self.feedback_profile:
            raise ValueError('circle controller differs from the reviewed profile')
        for q in self.points:
            for joint, angle in zip(JOINTS, q):
                low, high = control.limits[joint]
                if not low <= angle <= high:
                    raise ValueError(f"circle exceeds calibrated limits: {joint}")
        positions = {joint: control.preflight(joint) for joint in JOINTS}
        fresh = [control.angle(joint, positions[joint]) for joint in JOINTS]
        if max(abs(a-b) for a, b in zip(initial, fresh)) > .003:
            raise ValueError("arm moved after local circle validation; validate again")
        control.enable_all()
        fresh = [control.angle(joint, control.measured[joint]) for joint in JOINTS]
        if max(abs(a-b) for a, b in zip(initial, fresh)) > .003:
            control.stop("arm moved after local circle validation")
            raise ValueError("arm moved after local circle validation; validate again")
        self.control = control
        self.start(fresh, control.clock(), laps=laps)

    def advance_hardware(self, positions, telemetry):
        if not self.active:
            return
        control = self.control
        if control.stopped or set(control.enabled) != set(JOINTS):
            self.cancel("motor is not enabled")
            return
        now = control.clock()
        target = self.sample(now)
        if 'transition_acceleration' in self.feedback_profile.get('native_stream', {}):
            control.schedule_stream_acceleration(self.speed_fraction(now))
        if self.phase == "completed":
            control.stop("circle completed: hold")
            return
        control.targets.update(zip(JOINTS, target))

    def status(self, now=None):
        return {"phase": self.phase, "radius_m": self.radius,
                "plane": self.plane, "center_m": self.center,
                'controller_profile': self.feedback_profile['name'],
                'angular_speed_deg_s': 360/self.period,
                'motion_budget': self.motion_budget,
                "period_seconds": self.period, "max_speed_rad_s": None,
                "entry_planned_speed_rad_s": self.entry_speed,
                "planned_peak_speed_rad_s": self.planned_peak_speed,
                "ramp_seconds": self.ramp_seconds, "control_hz": self.control_hz,
                "profile_sha256": self.profile_hash,
                "entry_seconds": getattr(self, "entry_seconds", None),
                "tracking_error_stop_rad": None, "entry_arrival_timeout_seconds": None,
                "initial_qpos": getattr(self, "initial", None),
                "laps": getattr(self, "laps", None),
                "elapsed_seconds": (max(0, now - self.started)
                                    if now is not None and self.started is not None else None)}
