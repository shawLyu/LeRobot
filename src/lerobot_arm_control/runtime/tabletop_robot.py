"""Remote read-only SO101 feedback, with the existing measured joint calibration."""

import hashlib
import threading
import time
from collections import deque
from contextlib import ExitStack

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import (
    CONTROL_CONFIG,
    CONTROL_CONFIG_SHA256,
    CONTROL_HZ,
)
from lerobot_arm_control.control.servo_feedback import read_control_feedback
from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.robot.so101.driver import connected
from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
from lerobot_arm_control.runtime.common import defer_cyclic_gc, next_deadline
from lerobot_arm_control.transport.realtime import JOINTS

ROOT = workspace_root()


def start_additional_feedback(config, *, camera_only=False):
    """Read each additional arm only with its independent calibration."""
    feedback = {}
    if camera_only:
        return feedback
    for entry in config.get('additional_robots', []):
        if not entry.get('enabled') or not entry.get('joint_calibration'):
            continue
        own_mapping = (ROOT/entry['joint_calibration']).exists()
        robot = RobotFeedback(entry)
        if own_mapping:
            robot.start()
        else:
            path = entry['joint_calibration']
            robot.latest.update(status='uncalibrated', read_only=True,
                                error=f'Six-joint calibration required: {path}')
            robot.metadata['joint_calibration'] = entry['joint_calibration']
        feedback[entry['arm']] = robot
    return feedback


def calibration_identity(bus, arm, model_path):
    low = bus.sync_read("Min_Position_Limit", normalize=False)
    high = bus.sync_read("Max_Position_Limit", normalize=False)
    return {"port": arm.port, "motors": arm.motors,
            "raw_limits": {name: [int(low[name]), int(high[name])] for name in JOINTS},
            "homing_offsets": bus.sync_read("Homing_Offset", normalize=False),
            "phase": bus.sync_read("Phase", normalize=False),
            "mode": bus.sync_read("Operating_Mode", normalize=False),
            "model": "so101", "mjcf_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest()}


class RobotFeedback:
    def __init__(self, config, *, hardware_control=False, task='push_t'):
        self.config = config
        self.hardware_control = hardware_control
        self.task = task
        self.motor = None
        self.stopped = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.latest = {"status": "connecting", "joints": list(JOINTS)}
        self.samples = deque(maxlen=200)
        self.sampled_at = 0
        self.metadata = {"model": "so101", "joints": list(JOINTS), "read_only": True}
        self.worker = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.worker.start()
        self.ready.wait(timeout=8)

    def _run(self):
        try:
            arm = load_arms()[self.config["arm"]]
            model_path = resource_path('assets/so101/so101.xml')
            calibration_path = ROOT / self.config["joint_calibration"]
            with connected(arm) as bus, ExitStack() as lifecycle:
                identity = calibration_identity(bus, arm, model_path)
                calibration = AngleCalibration(calibration_path, identity)
                if calibration.count() != len(JOINTS):
                    raise ValueError("All six measured joint calibrations are required")
                fits = {name: calibration.fit(name) for name in JOINTS}
                self.metadata.update(joint_calibration=str(calibration_path),
                    joint_calibration_sha256=hashlib.sha256(
                        calibration_path.read_bytes()).hexdigest(),
                    kinematic_correction=calibration.correction,
                    calibration_mode='independent',
                    calibration_source_arm=self.config['arm'], independently_calibrated=True)
                self.metadata.update(fingerprint=identity, fits=fits)
                if self.hardware_control:
                    from lerobot_arm_control.tasks.push_t.push_t_motor import (
                        ENTRY_PROFILE_HASH,
                        PushMotor,
                    )

                    self.motor = PushMotor(bus, calibration, joint_ranges(model_path),
                                           identity['raw_limits'], task=self.task)
                    self.metadata.update(control_hz=CONTROL_HZ,
                                         control_config=str(CONTROL_CONFIG),
                                         control_config_sha256=CONTROL_CONFIG_SHA256,
                                         trajectory_profile_sha256=ENTRY_PROFILE_HASH)
                    defer_cyclic_gc(lifecycle)
                    lifecycle.callback(self.motor.close)
                self.metadata['motion_available'] = self.hardware_control
                telemetry, telemetry_at, sequence = {}, 0, 0
                next_control = time.monotonic()
                while not self.stopped.is_set():
                    started = time.monotonic()
                    if self.motor:
                        telemetry = read_control_feedback(bus)
                        raw = telemetry.pop('Present_Position')
                    else:
                        raw = bus.sync_read("Present_Position", normalize=False)
                        if started - telemetry_at >= .5:
                            telemetry = {register: bus.sync_read(register, normalize=False)
                                         for register in ("Torque_Enable", "Status",
                                                          "Present_Voltage", "Present_Temperature")}
                            telemetry_at = time.monotonic()
                    sampled_ns = time.time_ns()
                    if self.motor:
                        self.motor.tick(raw, telemetry)
                    sequence += 1
                    packet = {"status": "live", "sequence": sequence, "raw": raw,
                              "qpos": [(raw[name]-fits[name]['zero_raw'])*
                                       fits[name]['radians_per_tick'] for name in JOINTS],
                              "calibration_mode": self.metadata['calibration_mode'],
                              "calibration_source_arm": self.metadata['calibration_source_arm'],
                              "independently_calibrated": self.metadata['independently_calibrated'],
                              "joints": list(JOINTS), "sampled_unix_ns": sampled_ns,
                              "telemetry": telemetry,
                              "read_only": not (self.motor and
                                                self.motor.status()['writes_enabled']),
                              "read_ms": (time.monotonic()-started)*1000}
                    if self.motor:
                        packet['control'] = self.motor.control.status()
                    with self.lock:
                        self.latest = packet
                        self.sampled_at = started
                        self.samples.append(packet)
                    self.ready.set()
                    rate = (CONTROL_HZ if self.hardware_control
                            else self.config.get('feedback_hz', 20))
                    next_control = next_deadline(next_control, time.monotonic(), 1/rate)
                    self.stopped.wait(max(0, next_control-time.monotonic()))
        except Exception as exc:
            with self.lock:
                self.latest = {"status": "error", "error": str(exc), "joints": list(JOINTS)}
        finally:
            self.ready.set()

    def snapshot(self):
        with self.lock:
            state = dict(self.latest)
            sampled_at = self.sampled_at
        if "sampled_unix_ns" in state:
            state["sample_age_at_send_ms"] = (time.monotonic()-sampled_at)*1000
            if state["sample_age_at_send_ms"] > 500:
                state["status"] = "stale"
        return state

    def metadata_snapshot(self):
        state = self.snapshot()
        with self.lock:
            metadata = dict(self.metadata)
        metadata['status'] = state['status']
        if 'error' in state:
            metadata.update(error=state['error'], motion_available=False)
        return metadata

    def drain_samples(self):
        with self.lock:
            samples = list(self.samples)
            self.samples.clear()
        return samples

    def close(self):
        self.stopped.set()
        if self.worker.ident is not None:
            self.worker.join(timeout=3)
