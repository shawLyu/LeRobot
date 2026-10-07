"""Teach an observed unpowered rest pose, then slowly return and release torque."""

import json
import math
from collections import deque
from pathlib import Path

from .control_defaults import CONTROL_HZ, TRAJECTORY
from .realtime import JOINTS

STILL_SECONDS = 2.0
ARRIVAL_TICKS = 5
RESET_PLANNED_SPEED = TRAJECTORY["reset_speed_rad_s"]


def smooth_fraction(progress):
    """Quintic progress with zero velocity and acceleration at both endpoints."""
    progress = max(0.0, min(1.0, progress))
    return max(0.0, min(1.0, progress**3 * (10 + progress * (-15 + 6 * progress))))


class Parking:
    def __init__(self, control, path=None):
        self.control = control
        self.path = Path(path) if path else None
        self.pose = None
        self.observed_pose = None
        self.release_verified = False
        self.history = deque(maxlen=math.ceil((STILL_SECONDS + 1) * CONTROL_HZ))
        self.phase = "not configured"
        self.notice = "Support the arm, release all joints, then save the rest pose (P)."
        self.arrived_at = None
        self.settling_hold = False
        self.planned_speed = RESET_PLANNED_SPEED
        self.identity = {
            "arm": getattr(control.calibration, "fingerprint", {}),
            "angle_mappings": {joint: control.calibration.fit(joint) for joint in JOINTS},
        }
        if self.path and self.path.exists():
            saved = json.loads(self.path.read_text())
            if saved.get("version") not in (1, 2) or saved.get("identity") != self.identity:
                self.notice = "Rest pose identity changed; teach it again."
            else:
                self.configure(saved.get("observed_raw", saved["raw"]))
                if saved["version"] == 2:
                    self.release_verified &= saved.get("release_verified", False)
                if saved["version"] == 1:
                    self.write_pose()
                self.phase = "ready"
                self.notice = ("Reset returns to the nearest in-bounds pose. "
                               + ("Verified support: auto release." if self.release_verified
                                  else "Boundary support unverified: keep torque ON."))

    @property
    def active(self):
        return self.phase in ("moving", "settling")

    @property
    def busy(self):
        return self.active or self.phase == "verifying release"

    def validate(self, raw):
        if not isinstance(raw, dict) or set(raw) != set(JOINTS):
            raise ValueError("rest pose must contain all six joints")
        for joint in JOINTS:
            low, high = self.control.rest_limits[joint]
            if (type(raw[joint]) is not int
                    or not low <= self.control.angle(joint, raw[joint]) <= high):
                raise ValueError(f"rest pose outside calibrated limits: {joint}")

    def configure(self, observed):
        self.validate(observed)
        self.observed_pose = dict(observed)
        self.pose = {joint: self.control.bounded_raw(joint, observed[joint]) for joint in JOINTS}
        # A projected endpoint has not been observed supported with torque OFF.
        self.release_verified = self.pose == self.observed_pose

    def observe(self, positions, telemetry):
        now = self.control.clock()
        self.history.append((now, dict(positions), {key: dict(values)
                                                   for key, values in telemetry.items()}))
        if self.phase == "verifying release":
            if any(telemetry["Torque_Enable"].values()):
                self.phase, self.notice = "release incomplete", "Some motors still have torque."
            elif any(abs(positions[joint] - self.pose[joint]) > 8 for joint in JOINTS):
                self.phase = "released pose moved"
                self.notice = ("The arm moved after release. Support it and "
                               "teach the rest pose again.")
            elif now - self.arrived_at >= STILL_SECONDS:
                self.phase = "unpowered rest"
                self.notice = "All torque OFF; encoder position remained stable for 2 seconds."

    def save(self):
        if self.active:
            raise ValueError("Reset is running; Space cancels it")
        if self.path is None:
            raise ValueError("rest pose storage is not configured")
        now = self.control.clock()
        samples = [sample for sample in self.history if now - sample[0] <= STILL_SECONDS + 0.1]
        if len(samples) < 20 or samples[-1][0] - samples[0][0] < STILL_SECONDS:
            raise ValueError("wait for 2 seconds of still, unpowered feedback")
        for _, _, telemetry in samples:
            if any(telemetry["Torque_Enable"].values()) or any(telemetry["Status"].values()):
                raise ValueError("rest pose requires all six torques OFF and no motor faults")
        for joint in JOINTS:
            positions = [raw[joint] for _, raw, _ in samples]
            if max(positions) - min(positions) > 4:
                raise ValueError("arm is moving; remove your hands only after it is supported")
        pose = {joint: int(samples[-1][1][joint]) for joint in JOINTS}
        self.configure(pose)
        self.write_pose()
        self.phase = "ready"
        self.notice = ("Nearest in-bounds Reset pose saved. "
                       + ("Supported: auto release." if self.release_verified
                          else "Boundary support unverified: torque will remain ON."))

    def write_pose(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        saved = {"version": 2, "identity": self.identity, "raw": self.pose,
                 "observed_raw": self.observed_pose, "release_verified": self.release_verified,
                 "unpowered_observation_seconds": STILL_SECONDS}
        temporary.write_text(json.dumps(saved, indent=2) + "\n")
        temporary.replace(self.path)

    def start(self):
        if self.busy:
            return
        if self.pose is None:
            raise ValueError("Teach a physically supported rest pose with P before Reset")
        self.validate(self.pose)
        # Validate every motor before the first torque write.
        positions = {joint: self.control.preflight(joint)
                     for joint in JOINTS}
        at_rest = all(abs(positions[joint] - self.pose[joint]) <= ARRIVAL_TICKS for joint in JOINTS)
        if not at_rest:
            self.control.enable_all()
        else:
            for joint in self.control.enabled:
                self.control.stopped.pop(joint, None)
        if not at_rest:
            positions = dict(self.control.measured)  # Fresh synchronized enable positions.
        self.start_angles = {joint: self.control.angle(joint, positions[joint]) for joint in JOINTS}
        distance = max(abs(self.start_angles[joint] - self.control.angle(joint, self.pose[joint]))
                       for joint in JOINTS)
        self.started_at = self.control.clock()
        # The quintic profile's peak normalized velocity is 1.875.
        self.duration = 0.0 if at_rest else max(0.5, 1.875 * distance / self.planned_speed)
        self.control.heartbeat()
        self.arrived_at = None
        self.settling_hold = False
        self.phase = "settling" if at_rest else "moving"
        self.notice = "Remote smooth return. Space cancels and keeps support torque ON."

    def cancel(self, reason):
        if self.active:
            self.phase, self.notice = "cancelled: hold", reason

    def advance(self, positions, telemetry):
        if not self.active:
            return
        now = self.control.clock()
        if not self.control.confirm_status(telemetry, JOINTS):
            return
        arrived = all(
            abs(positions[joint] - self.pose[joint]) <= ARRIVAL_TICKS
            and self.control.limits[joint][0] <= self.control.angle(joint, positions[joint])
            <= self.control.limits[joint][1]
            for joint in JOINTS
        )
        progress = min(1.0, (now - self.started_at) / self.duration) if self.duration else 1.0
        blend = smooth_fraction(progress)
        for joint in JOINTS:
            destination = self.control.angle(joint, self.pose[joint])
            planned = self.start_angles[joint] + blend * (destination - self.start_angles[joint])
            self.control.targets[joint] = planned
        if progress < 1.0 or not arrived:
            if self.settling_hold:
                for joint in self.control.enabled:
                    if self.control.stopped.get(joint) == "Reset settling: hold":
                        self.control.stopped.pop(joint)
                self.settling_hold = False
            self.phase, self.arrived_at = "moving", None
            return
        self.phase = "settling"
        if not self.settling_hold:
            # Once inside the existing arrival tolerance, hold the measured pose
            # while verifying stillness. Continuing PI integration here can hunt
            # around quantized feedback and repeatedly restart the settling timer.
            self.control.stop("Reset settling: hold")
            self.phase = "settling"
            self.settling_hold = True
            self.arrived_at = now
        if any(telemetry.get("Moving", {}).values()):
            self.arrived_at = now
            return
        if now - self.arrived_at < STILL_SECONDS:
            return
        if not self.release_verified:
            self.control.stop("Boundary reached: hold; support has not been verified")
            self.phase = "boundary hold"
            self.notice = "At nearest boundary; torque ON. Verify support before releasing."
            return
        # No Goal_Position writes after this point: they could re-enable torque.
        self.control.enabled.clear()
        self.control.stopped.clear()
        self.phase = "verifying release"
        self.arrived_at = now
        errors = []
        for joint in JOINTS:
            try:
                self.control.bus.write("Torque_Enable", joint, 0, normalize=False)
            except Exception as exc:
                errors.append(f"{joint}: {exc}")
        torque = self.control.bus.sync_read("Torque_Enable", normalize=False)
        for joint, value in torque.items():
            if value:
                self.control.enabled.add(joint)
                self.control.stopped[joint] = "Reset release incomplete"
        if errors or any(torque.values()):
            self.phase = "release incomplete"
            self.notice = "; ".join(errors) or "Torque remains ON."
        else:
            self.notice = "All torque OFF; observing stability."

    def status(self):
        remaining = max((abs(self.control.angle(joint, self.control.measured[joint])
                             - self.control.angle(joint, self.pose[joint])) for joint in JOINTS),
                        default=0) if self.pose and len(self.control.measured) == 6 else None
        return {"configured": self.pose is not None, "phase": self.phase, "notice": self.notice,
                "reset_raw": self.pose, "observed_raw": self.observed_pose,
                "release_verified": self.release_verified,
                "trajectory": "remote quintic", "duration_seconds": getattr(self, "duration", None),
                "planned_speed_rad_s": self.planned_speed,
                "remaining_degrees": math.degrees(remaining) if remaining is not None else None}
