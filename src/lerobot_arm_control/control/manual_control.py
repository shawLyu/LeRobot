"""LeRobot-style synchronized position control with explicit hold and release."""

import hashlib
import math
import time
from collections import deque

from lerobot_arm_control.control.control_defaults import (
    CONTROL,
    CONTROL_HZ,
    DEFAULTS,
    POSITION_FEEDBACK,
    READ_RETRIES,
)
from lerobot_arm_control.control.gripper_contact import GripperContact
from lerobot_arm_control.control.motion_stall import MotionStall
from lerobot_arm_control.control.position_feedback import PositionFeedback
from lerobot_arm_control.transport.realtime import JOINTS, WATCHDOG_SECONDS
from lerobot_arm_control.visualization.ui import motor_fault_text


class ManualControl:
    def __init__(self, bus, calibration, raw_limits, clock=time.monotonic,
                 rest_path=None):
        if calibration.count() != 6:
            raise ValueError("real control requires all six calibrated joints")
        self.bus = bus
        self.calibration = calibration
        self.clock = clock
        self.enabled = set()
        self.stopped = {}
        self.measured = {}
        self.targets = {}
        self.sent = {}
        self.command_angles = {}
        self.original_acceleration = {}
        self.native_acceleration = {}
        self.original_velocity = {}
        self.native_velocity = {}
        self.original_p = {}
        self.native_p = {}
        self.trajectory = None
        self.periods = deque(maxlen=200)
        self.last_fault = None
        self.status_warning = None
        self.faulted_joints = set()
        self.fault_hold_attempted = set()
        self.initialized = False
        self.last_tick = clock()
        self.last_heartbeat = clock()
        self.limits = {}
        self.rest_limits = {}
        self.raw_limits = {joint: tuple(raw_limits[joint]) for joint in JOINTS}
        for joint in JOINTS:
            fit = calibration.fit(joint)
            self.limits[joint] = tuple(sorted(self.angle(joint, value)
                                             for value in raw_limits[joint]))
            if (not 0 <= raw_limits[joint][0] < raw_limits[joint][1] <= 4095
                    or not math.isfinite(fit["radians_per_tick"])
                    or self.limits[joint][0] >= self.limits[joint][1]):
                raise ValueError(f"invalid control limits: {joint}")
            self.rest_limits[joint] = self.limits[joint]
        self.position_feedback = PositionFeedback(POSITION_FEEDBACK, self.limits)
        self.gripper_contact = GripperContact(CONTROL['gripper_contact'])
        self.motion_stall = MotionStall(CONTROL['motion_stall'])
        for envelope in CONTROL['hardware_command_envelopes']:
            if envelope['port'] != getattr(calibration, 'fingerprint', {}).get('port'):
                continue
            digest = hashlib.sha256(calibration.path.read_bytes()).hexdigest()
            if digest != envelope['calibration_sha256']:
                raise ValueError('hardware command envelope calibration changed; review limits')
            for joint, bounds in envelope['raw_limits'].items():
                self.set_command_limits(joint, tuple(sorted(self.angle(joint, raw)
                                                           for raw in bounds)))
        from lerobot_arm_control.control.parking import Parking

        self.parking = Parking(self, rest_path)

    def angle(self, joint, raw):
        fit = self.calibration.fit(joint)
        return (raw - fit["zero_raw"]) * fit["radians_per_tick"]

    def raw(self, joint, angle):
        fit = self.calibration.fit(joint)
        return round(angle / fit["radians_per_tick"] + fit["zero_raw"])

    def heartbeat(self):
        self.last_heartbeat = self.clock()

    def prepare_position_stream(self, joints):
        stream = self.position_feedback.profile.get('native_stream')
        if stream is None:
            return
        # Fresh hold goals must already be confirmed. Lock=1 keeps optional P
        # changes volatile on ST firmware; never unlock storage or drop torque.
        current = {joint: int(self.bus.read("Acceleration", joint, normalize=False))
                   for joint in joints}
        maximum = {joint: int(self.bus.read("Maximum_Acceleration", joint, normalize=False))
                   for joint in joints}
        if any(not 0 <= current[joint] <= 254 or not 1 <= maximum[joint] <= 254
               for joint in joints):
            raise ValueError("invalid native acceleration configuration")
        acceleration = stream["acceleration"]
        requested = {joint: min(acceleration, maximum[joint]) for joint in joints}
        for joint in joints:
            self.original_acceleration.setdefault(joint, current[joint])
        changed = {joint: value for joint, value in requested.items() if current[joint] != value}
        if changed:
            self.bus.sync_write("Acceleration", changed, normalize=False)
        if any(int(self.bus.read("Acceleration", joint, normalize=False)) != value
               for joint, value in requested.items()):
            raise RuntimeError("native acceleration could not be confirmed")
        self.native_acceleration.update(requested)
        for joint in joints:
            if int(self.bus.read("Goal_Time", joint, normalize=False)) != 0:
                raise ValueError("native velocity profile requires Goal_Time=0")
            velocity = int(self.bus.read("Goal_Velocity", joint, normalize=False))
            maximum = int(self.bus.read("Maximum_Velocity_Limit", joint, normalize=False))
            factor = int(self.bus.read("Velocity_Unit_factor", joint, normalize=False))
            if not 0 <= velocity <= 32767 or maximum <= 0 or factor <= 0:
                raise ValueError("invalid native velocity configuration")
            self.original_velocity.setdefault(joint, velocity)
            requested_velocity = min(stream["velocity_ticks_s"], maximum * factor)
            self.bus.write("Goal_Velocity", joint, requested_velocity, normalize=False)
            observed_velocity = int(self.bus.read("Goal_Velocity", joint, normalize=False))
            if observed_velocity != requested_velocity:
                raise RuntimeError(f"{joint}: native velocity could not be confirmed")
            self.native_velocity[joint] = requested_velocity
        requested_p = {j: value for j, value in stream.get('locked_p_coefficients', {}).items()
                       if j in joints}
        if any(int(self.bus.read('Lock', j, normalize=False)) != 1 for j in requested_p):
            raise RuntimeError('temporary P adjustment requires every storage lock closed')
        for joint, value in requested_p.items():
            original = int(self.bus.read('P_Coefficient', joint, normalize=False))
            self.original_p.setdefault(joint, original)
            self.bus.write('P_Coefficient', joint, value, normalize=False)
            if (int(self.bus.read('P_Coefficient', joint, normalize=False)) != value
                    or int(self.bus.read('Lock', joint, normalize=False)) != 1):
                raise RuntimeError(f'{joint}: temporary P could not be confirmed')
            self.native_p[joint] = value

    def restore_position_stream(self, joints):
        for joint in joints:
            if joint in self.original_p:
                if int(self.bus.read('Lock', joint, normalize=False)) != 1:
                    raise RuntimeError(f'{joint}: refusing P restore with storage unlocked')
                original = self.original_p[joint]
                self.bus.write('P_Coefficient', joint, original, normalize=False)
                if int(self.bus.read('P_Coefficient', joint, normalize=False)) != original:
                    raise RuntimeError(f'{joint}: temporary P restore failed')
                self.original_p.pop(joint)
                self.native_p.pop(joint, None)
            if joint in self.original_velocity:
                original = self.original_velocity[joint]
                self.bus.write("Goal_Velocity", joint, original, normalize=False)
                if int(self.bus.read("Goal_Velocity", joint, normalize=False)) != original:
                    raise RuntimeError(f"{joint}: native velocity restore failed")
                self.original_velocity.pop(joint)
                self.native_velocity.pop(joint, None)
            if joint in self.original_acceleration:
                original = self.original_acceleration[joint]
                self.bus.write("Acceleration", joint, original, normalize=False)
                if int(self.bus.read("Acceleration", joint, normalize=False)) != original:
                    raise RuntimeError(f"{joint}: native acceleration restore failed")
                self.original_acceleration.pop(joint)
                self.native_acceleration.pop(joint, None)

    def bounded_raw(self, joint, raw):
        """Nearest integer encoder target inside the ordinary motion bounds."""
        low, high = self.raw_limits[joint]
        return max(low, min(high, int(raw)))

    def set_command_limits(self, joint, bounds):
        """Bound the compensated command without changing calibration or held poses."""
        if self.enabled:
            raise ValueError('command limits must be configured before enabling')
        low, high = bounds
        native_low, native_high = self.rest_limits[joint]
        if (not all(math.isfinite(v) for v in bounds) or low >= high
                or low < native_low-1e-9 or high > native_high+1e-9):
            raise ValueError(f'{joint}: command range outside calibrated travel')
        fit = self.calibration.fit(joint)
        ticks = sorted(v/fit['radians_per_tick']+fit['zero_raw'] for v in bounds)
        # Round inward; allow only floating-point noise at an exact encoder endpoint.
        raw_bounds = (math.ceil(ticks[0]-1e-9), math.floor(ticks[1]+1e-9))
        existing = self.raw_limits[joint]
        if (raw_bounds[0] >= raw_bounds[1] or raw_bounds[0] < existing[0]
                or raw_bounds[1] > existing[1]):
            raise ValueError(f'{joint}: command range has no usable encoder travel')
        self.raw_limits[joint] = raw_bounds
        self.limits[joint] = tuple(sorted(self.angle(joint, raw) for raw in raw_bounds))
        self.position_feedback.reset([joint])
        self.motion_stall.reset()
        if joint == 'gripper':
            self.gripper_contact.reset()

    def preflight(self, joint, *, limits=None):
        if joint not in JOINTS:
            raise ValueError("unknown joint")
        try:
            status = int(self.bus.read("Status", joint, normalize=False))
        except RuntimeError as exc:
            if '[RxPacketError]' not in str(exc):
                raise
            status = int(self.bus.sync_read('Status', normalize=False)[joint])
            if not status:
                raise
        if status:
            self.faulted_joints.add(joint)
            raise ValueError(f"{joint}: {motor_fault_text(status)} (Status={status})")
        if int(self.bus.read("Operating_Mode", joint, normalize=False)) != 0:
            raise ValueError("joint must already be in position mode")
        voltage = int(self.bus.read("Present_Voltage", joint, normalize=False))
        minimum = int(self.bus.read("Min_Voltage_Limit", joint, normalize=False))
        maximum = int(self.bus.read("Max_Voltage_Limit", joint, normalize=False))
        if not minimum <= voltage <= maximum:
            raise ValueError(f"voltage raw {voltage} outside configured {minimum}..{maximum}")
        raw = int(self.bus.read("Present_Position", joint, normalize=False))
        angle = self.angle(joint, raw)
        if not 0 <= raw <= 4095:
            raise ValueError("invalid single-turn encoder feedback")
        if limits is not None and not limits[0] <= angle <= limits[1]:
            raise ValueError("position outside requested hold limits")
        return raw

    def enable(self, joint, *, limits=None):
        if self.parking.busy:
            raise ValueError("Reset is running; Space cancels it")
        raw = self.preflight(joint, limits=limits)
        self.position_feedback.reset([joint])
        self.motion_stall.reset()
        if joint == 'gripper':
            self.gripper_contact.reset()
        angle = self.angle(joint, raw)
        # Goal writes can auto-enable torque. Never activate a stored, stale goal.
        self.enabled.add(joint)
        self.measured[joint] = raw
        self.targets[joint] = angle
        self.sent[joint] = raw
        self.command_angles[joint] = angle
        self.stopped[joint] = "enabling"
        try:
            self.bus.write("Goal_Position", joint, raw, normalize=False)
            if int(self.bus.read("Goal_Position", joint, normalize=False)) != raw:
                raise RuntimeError("fresh goal could not be confirmed")
            self.prepare_position_stream([joint])
            self.bus.write("Torque_Enable", joint, 1, normalize=False)
            if int(self.bus.read("Torque_Enable", joint, normalize=False)) != 1:
                raise RuntimeError("joint torque did not enable")
        except Exception:
            self.stop("enable failed: hold")
            raise
        self.stopped.pop(joint, None)
        self.faulted_joints.discard(joint)
        self.fault_hold_attempted.discard(joint)
        if not self.faulted_joints:
            self.last_fault = None
        self.heartbeat()

    def enable_all(self):
        if self.parking.busy:
            raise ValueError("Reset is running; Space cancels it")
        # No writes until every motor has passed mode, fault, voltage and position checks.
        for joint in JOINTS:
            self.preflight(joint)
        positions = self.bus.sync_read("Present_Position", normalize=False)
        for joint in JOINTS:
            if not 0 <= positions[joint] <= 4095:
                raise ValueError(f"{joint}: invalid single-turn encoder feedback")
        self.position_feedback.reset()
        self.gripper_contact.reset()
        self.motion_stall.reset()
        for joint, raw in positions.items():
            self.enabled.add(joint)
            self.measured[joint] = raw
            self.targets[joint] = self.angle(joint, raw)
            self.sent[joint] = raw
            self.command_angles[joint] = self.targets[joint]
            self.stopped[joint] = "enabling all"
        try:
            self.bus.sync_write("Goal_Position", positions, normalize=False)
            # Sync writes have no per-motor ACK. Confirm fresh goals before explicit torque ON.
            if self.bus.sync_read("Goal_Position", normalize=False) != positions:
                raise RuntimeError("fresh goals could not be confirmed for all motors")
            self.prepare_position_stream(JOINTS)
            self.bus.sync_write("Torque_Enable", {joint: 1 for joint in JOINTS}, normalize=False)
            if not all(self.bus.sync_read("Torque_Enable", normalize=False).values()):
                raise RuntimeError("not all motor torques enabled")
        except Exception:
            self.stop("enable all failed: hold")
            raise
        self.stopped.clear()
        self.faulted_joints.clear()
        self.fault_hold_attempted.clear()
        self.last_fault = None
        self.heartbeat()

    def target(self, joint, angle):
        if self.parking.active:
            raise ValueError("Reset is running; Space cancels it")
        if joint not in self.enabled or joint in self.stopped:
            raise ValueError("press E to enable this joint before moving")
        if type(angle) not in (int, float) or not math.isfinite(angle):
            raise ValueError("target must be a finite angle")
        low, high = self.limits[joint]
        self.targets[joint] = max(low, min(high, float(angle)))
        self.heartbeat()

    def settle_target(self, joint):
        """Keep the nominal goal while clearing compensation from a completed ramp."""
        if self.parking.active or joint not in self.enabled or joint in self.stopped:
            raise ValueError('settling requires an enabled, running joint')
        self.position_feedback.finish_reference([joint])
        self.heartbeat()

    def stop(self, reason="manual hold", *, fault_telemetry=None):
        self.position_feedback.reset()
        self.gripper_contact.reset()
        self.motion_stall.reset()
        if self.trajectory:
            self.trajectory.cancel(reason)
        self.parking.cancel(reason)
        # Latch every joint before bus I/O, including when a read/write fails.
        self.stopped.update({joint: reason for joint in self.enabled})
        self.command_angles.clear()
        errors = []
        for joint in sorted(self.enabled):
            if joint in self.faulted_joints:
                # For overload only, replace the stale squeezing goal once with
                # fresh feedback, preserving torque state and native protection.
                # Sensor/other alarms cannot authorize a position hold write.
                raw = self.measured.get(joint)
                if (fault_telemetry is not None and joint not in self.fault_hold_attempted
                        and fault_telemetry['Status'][joint] == 32
                        and fault_telemetry['Torque_Enable'][joint] == 1
                        and 0 <= self.clock()-self.last_tick <= .05
                        and type(raw) is int and 0 <= raw <= 4095):
                    self.fault_hold_attempted.add(joint)
                    try:
                        self.bus.sync_write('Goal_Position', {joint: raw}, normalize=False)
                        if self.bus.sync_read('Goal_Position', normalize=False)[joint] != raw:
                            raise RuntimeError('overload hold goal not confirmed')
                        self.targets[joint] = self.angle(joint, raw)
                        self.sent[joint] = raw
                        self.last_fault.setdefault('overload_hold_raw', {})[joint] = raw
                    except Exception as exc:
                        errors.append(f'{joint}: overload hold failed: {exc}')
                continue
            try:
                if int(self.bus.read("Torque_Enable", joint, normalize=False)) != 1:
                    # Goal writes must never reactivate an externally disabled joint.
                    self.enabled.remove(joint)
                    continue
                raw = int(self.bus.read("Present_Position", joint, normalize=False))
                self.measured[joint] = raw
                self.targets[joint] = self.angle(joint, raw)
                self.sent[joint] = raw
                self.bus.write("Goal_Position", joint, raw, normalize=False)
            except Exception as exc:
                errors.append(f"{joint}: {exc}")
        if errors:
            raise RuntimeError("hold could not be confirmed: " + "; ".join(errors))

    def release(self, joint):
        if self.parking.active:
            self.stop("Reset cancelled for manual release")
        if joint not in self.enabled:
            raise ValueError("joint is not enabled by this session")
        raw = int(self.bus.read("Present_Position", joint, normalize=False))
        self.bus.write("Goal_Position", joint, raw, normalize=False)
        self.restore_position_stream([joint])
        # Torque off is last: later goal writes could re-enable it.
        self.enabled.remove(joint)
        self.stopped.pop(joint, None)
        self.targets.pop(joint, None)
        self.sent.pop(joint, None)
        self.command_angles.pop(joint, None)
        self.position_feedback.reset([joint])
        if joint == 'gripper':
            self.gripper_contact.reset()
        self.bus.write("Torque_Enable", joint, 0, normalize=False)
        if int(self.bus.read("Torque_Enable", joint, normalize=False)) != 0:
            raise RuntimeError("joint torque did not release")

    def trip(self, reason, telemetry):
        self.stopped.update({joint: reason for joint in self.faulted_joints})
        self.last_fault = {"reason": reason, "raw": dict(self.measured),
                           "sent_raw": dict(self.sent),
                           "telemetry": {key: dict(values) for key, values in telemetry.items()}}
        try:
            self.stop(reason, fault_telemetry=telemetry)
        except RuntimeError as exc:
            # A faulted device may reject a hold read too. Keep the latch and
            # monitoring alive, while making failed holds visible to the operator.
            self.last_fault['hold_error'] = str(exc)

    def confirm_status(self, telemetry, joints):
        """Recheck bulk status flags, including joints held while Reset settles."""
        for joint in joints:
            reported = int(telemetry["Status"][joint])
            if not reported:
                continue
            packet_error = None
            try:
                status = int(self.bus.read("Status", joint, normalize=False,
                                           num_retry=READ_RETRIES))
            except RuntimeError as exc:
                if '[RxPacketError]' not in str(exc):
                    raise
                status = reported
                packet_error = str(exc)
            telemetry["Status"][joint] = status
            if status:
                self.faulted_joints.update(j for j, value in telemetry['Status'].items() if value)
                self.trip(f"{joint}: {motor_fault_text(status)} (Status={status}): hold",
                          telemetry)
                if packet_error:
                    self.last_fault['packet_error'] = packet_error
                return False
            self.status_warning = {"joint": joint, "reported_status": reported,
                                   "confirmed_status": status, "time_unix_ns": time.time_ns()}
        return True

    def step(self, positions, telemetry):
        now = self.clock()
        elapsed = max(0, now - self.last_tick)
        if elapsed:
            self.periods.append(elapsed)
        self.last_tick = now
        self.measured.update({joint: int(raw) for joint, raw in positions.items()})
        self.parking.observe(positions, telemetry)
        if not self.initialized:
            self.initialized = True
            # A previous window may leave load-bearing motors holding with torque on.
            # Adopt them for explicit release/hold, without re-enabling limp motors.
            for joint in JOINTS:
                if telemetry["Torque_Enable"][joint] == 1 and joint not in self.enabled:
                    self.enabled.add(joint)
                    self.sent[joint] = int(positions[joint])
                    self.targets[joint] = self.angle(joint, positions[joint])
                    self.command_angles[joint] = self.targets[joint]
                    self.stopped[joint] = "pre-existing torque: press E to resume"
            if not self.confirm_status(telemetry, JOINTS):
                return
            for joint in sorted(self.enabled):
                if self.stopped.get(joint, '').startswith('pre-existing torque'):
                    self.bus.write("Goal_Position", joint, self.sent[joint], normalize=False)
        if not self.enabled:
            self.parking.advance(positions, telemetry)
            return
        if now - self.last_heartbeat > WATCHDOG_SECONDS:
            if any(joint not in self.stopped for joint in self.enabled):
                self.stop("heartbeat timeout: hold")
            return
        for joint in sorted(self.enabled):
            if joint in self.stopped:
                continue
            if not self.confirm_status(telemetry, [joint]):
                return
            if telemetry["Torque_Enable"][joint] != 1:
                self.trip(f"{joint} lost torque: hold", telemetry)
                return
        self.parking.advance(positions, telemetry)
        if self.trajectory:
            self.trajectory.advance_hardware(positions, telemetry)
        updates = {}
        for joint in sorted(self.enabled):
            if joint in self.stopped:
                continue
            target = self.targets[joint]
            target = self.position_feedback.target(
                joint, target, self.angle(joint, positions[joint]), elapsed)
            if joint == 'gripper':
                was_contact = self.gripper_contact.hold_angle is not None
                held = self.gripper_contact.update(
                    self.targets[joint], self.angle(joint, positions[joint]), target,
                    telemetry.get('Present_Load', {}).get(joint), now)
                if held is not None:
                    # Freeze both nominal closure and compensation at contact;
                    # the native position loop holds without additional squeeze.
                    self.position_feedback.reset([joint])
                    target = held
                elif was_contact:
                    self.position_feedback.reset([joint])
                    target = self.position_feedback.target(
                        joint, self.targets[joint], self.angle(joint, positions[joint]), elapsed)
            elif self.motion_stall.update(
                    joint, self.targets[joint], self.angle(joint, positions[joint]), target,
                    telemetry.get('Present_Load', {}).get(joint), now):
                self.trip(f'{joint}: loaded motion stall: hold', telemetry)
                return
            raw = self.bounded_raw(joint, self.raw(joint, target))
            self.command_angles[joint] = self.angle(joint, raw)
            updates[joint] = raw
        if updates:
            # Official send_action sends synchronized targets on each action frame.
            self.bus.sync_write("Goal_Position", updates, normalize=False)
            self.sent.update(updates)

    def status(self):
        return {"enabled": sorted(self.enabled), "stopped": dict(self.stopped),
                "target_qpos": {joint: self.targets.get(
                    joint, self.angle(joint, self.measured[joint])) for joint in self.measured},
                "error_rad": {joint: self.targets[joint] - self.angle(
                    joint, self.measured[joint]) for joint in self.enabled},
                "limits": self.limits, "max_speed_rad_s": None,
                "controller_profile": self.position_feedback.profile.get('name', 'measured'),
                "control_hz": CONTROL_HZ,
                "max_relative_target": DEFAULTS["max_relative_target"],
                "tracking_error_stop_rad": None,
                "position_feedback": self.position_feedback.status(),
                "gripper_contact": self.gripper_contact.status(),
                "status_warning": self.status_warning,
                "faulted_joints": sorted(self.faulted_joints),
                "pending_native_restore": sorted(self.faulted_joints & (
                    self.original_acceleration.keys() | self.original_velocity.keys()
                    | self.original_p.keys())),
                "command_qpos": dict(self.command_angles),
                "sent_qpos": {joint: self.angle(joint, raw) for joint, raw in self.sent.items()},
                "native_acceleration": dict(self.native_acceleration),
                "native_velocity_ticks_s": dict(self.native_velocity),
                "native_locked_p_coefficients": dict(self.native_p),
                "update_rate_hz": len(self.periods) / sum(self.periods) if self.periods else None,
                "max_period_ms": max(self.periods, default=0) * 1000,
                "boundary_entry_qpos": {joint: self.angle(joint, self.bounded_raw(joint, raw))
                                        for joint, raw in self.measured.items()},
                "last_fault": self.last_fault, "parking": self.parking.status()}

    def close(self):
        if self.enabled:
            # Preserve support for gravity-loaded joints; explicit X releases torque.
            self.stop("connection closed: hold")
        self.restore_position_stream([joint for joint in self.original_acceleration
                                      if joint not in self.faulted_joints])
