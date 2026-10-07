"""Validated Cartesian circle and worker-clock playback for the SO-101."""

import hashlib
import json
import math
from copy import deepcopy
from pathlib import Path

from .control_defaults import CONTROL_CONFIG_SHA256, CONTROL_HZ, TRAJECTORY
from .parking import smooth_fraction
from .realtime import JOINTS

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "data/trajectories/circle.json"


def load_profile():
    profile = deepcopy(TRAJECTORY)
    keys = ("entry_speed_rad_s", "min_entry_seconds", "ramp_seconds",
            "reset_speed_rad_s", "period_seconds")
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
                or saved.get("model_sha256") != hashlib.sha256(
                    (ROOT / "assets/so101/so101.xml").read_bytes()).hexdigest()):
            raise ValueError("circle identity does not match the model")
        self.plan_hash = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        self.model_hash = saved["model_sha256"]
        self.points = saved["qpos"]
        self.period = profile["period_seconds"]
        self.center = saved["center_m"]
        self.radius = saved["radius_m"]
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
        peak = max(abs(b - a) * (len(self.points) - 1) / self.period
                   for left, right in zip(self.points, self.points[1:])
                   for a, b in zip(left, right))
        self.planned_peak_speed = peak

    @property
    def active(self):
        return self.phase in ("entering", "running")

    @staticmethod
    def validate_laps(laps):
        if laps is not None and (type(laps) is not int or laps <= 0):
            raise ValueError("laps must be a positive integer")

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
        if control.parking.busy or self.active:
            raise ValueError("another automatic motion is running")
        if (not isinstance(validation, dict) or validation.get("checked") is not True
                or validation.get("model_sha256") != self.model_hash
                or validation.get("plan_sha256") != self.plan_hash
                or validation.get("control_config_sha256") != self.profile_hash):
            raise ValueError("circle requires matching local collision validation")
        initial = validation.get("initial_qpos")
        if (not isinstance(initial, list) or len(initial) != len(JOINTS)
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in initial)):
            raise ValueError("invalid local circle entry pose")
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
        target = self.sample(control.clock())
        if self.phase == "completed":
            control.stop("circle completed: hold")
            return
        control.targets.update(zip(JOINTS, target))

    def status(self, now=None):
        return {"phase": self.phase, "radius_m": self.radius,
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
