"""Receive local Push-T plans; no planning or physics runs on the hardware host."""

import copy
import json
import math
import threading
import time
from pathlib import Path

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, TRAJECTORY
from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
from lerobot_arm_control.tasks.push_t.push_t_motor import (
    observation_issue,
    recovery_issue,
    same_scene,
)
from lerobot_arm_control.transport.pick_t_protocol import (
    CARRYING,
    STAGES,
    pick_observation_issue,
    segment_digest,
    stage_segments_issue,
)
from lerobot_arm_control.transport.realtime import JOINTS, WATCHDOG_SECONDS


def validate_segments(segments, *, ranges=None, entry_tolerance=0.):
    if not isinstance(segments, list) or not 1 <= len(segments) <= 128:
        raise ValueError("plan must contain 1..128 joint segments")
    ranges = joint_ranges() if ranges is None else ranges
    if not math.isfinite(entry_tolerance) or not 0 <= entry_tolerance <= math.radians(5):
        raise ValueError('invalid measured-start tolerance')
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise ValueError("invalid joint segment")
        for key in ("start", "end"):
            values = segment.get(key)
            if (not isinstance(values, list) or len(values) != len(JOINTS)
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError("segment requires six finite joint angles")
            tolerance = entry_tolerance if index == 0 and key == 'start' else 0.
            if any(not low-tolerance <= v <= high+tolerance
                   for v, (low, high) in zip(values, ranges, strict=True)):
                raise ValueError("segment exceeds SO101 joint limits")
        duration, speed = segment.get("seconds"), segment.get("speed_rad_s")
        if (type(duration) not in (int, float) or not math.isfinite(duration)
                or not .06 <= duration <= 120 or type(speed) not in (int, float)
                or not math.isfinite(speed) or not 0 < speed <= TRAJECTORY["entry_speed_rad_s"]):
            raise ValueError("invalid segment duration or speed")
        peak = 1.875 * max(abs(a-b) for a, b in zip(segment["start"], segment["end"]))
        if peak > speed * duration + 1e-8:
            raise ValueError("segment exceeds shared entry speed")
        if index and segment["start"] != segments[index-1]["end"]:
            raise ValueError("joint segments must be continuous")
    return copy.deepcopy(segments)


class PushReceiver:
    def __init__(self, robot, output, *, clock=time.monotonic, task='push_t'):
        self.robot, self.output, self.clock = robot, Path(output), clock
        self.output.mkdir(parents=True, exist_ok=False)
        self.lock = threading.Lock()
        self.state, self.received = None, 0
        self.last_command = clock()
        self.job_id = None
        self.task, self.next_stage, self.advanced_job = task, 0, None

    def observe(self, state):
        received = self.clock()
        with self.lock:
            self.state, self.received = copy.deepcopy(state), received
        if self.robot.motor:
            self.robot.motor.observe(state, received)
        self.check_heartbeat()

    def check_heartbeat(self):
        if (self.robot.motor and self.clock() - self.last_command > WATCHDOG_SECONDS
                and self.robot.motor.status()["phase"] in ("pending", "running")):
            self.robot.motor.cancel("local planner heartbeat timeout: hold")

    def handle(self, command):
        if not isinstance(command, dict) or command.get("version") != 1:
            raise ValueError("unsupported execution protocol")
        if command.get("control_config_sha256") != CONTROL_CONFIG_SHA256:
            raise ValueError("local/remote control configuration differs")
        operation = command.get("op")
        if self.task == 'pick_t':
            operations = {'pick_execute': 'push_execute', 'pick_stop': 'push_stop',
                          'pick_result': 'push_result', 'ping': 'ping'}
            if operation not in operations:
                raise ValueError('unsupported pick/place operation')
            operation = operations[operation]
        if operation not in ("ping", "push_stop", "push_execute", "push_result"):
            raise ValueError("unsupported remote operation")
        if operation == "push_stop":
            if self.robot.motor:
                reason = command.get('reason', 'operator stop: hold')
                if not isinstance(reason, str) or not reason or len(reason) > 512:
                    raise ValueError('invalid stop reason')
                self.robot.motor.cancel(reason)
        elif operation == "push_execute":
            if not self.robot.motor:
                raise ValueError("hardware execution disabled")
            if command.get("model_sha256") != self.robot.metadata["fingerprint"]["mjcf_sha256"]:
                raise ValueError("local/remote SO101 model differs")
            segments = validate_segments(command.get("segments"),
                ranges=self.robot.metadata.get('model_joint_ranges'),
                entry_tolerance=(self.robot.metadata.get('entry_tolerance_rad', 0.)
                                 if self.task == 'push_t' else 0.))
            reference = command.get("reference")
            recovery = command.get("recovery", False)
            if type(recovery) is not bool:
                raise ValueError("invalid recovery flag")
            with self.lock:
                state, received = self.state, self.received
            now = self.clock()
            stage = command.get('stage')
            if self.task == 'pick_t':
                progress = self.robot.motor.status()
                if progress['phase'] == 'completed' and self.job_id != self.advanced_job:
                    self.next_stage += 1
                    self.advanced_job = self.job_id
                if self.next_stage >= len(STAGES) or stage != STAGES[self.next_stage]:
                    raise ValueError('pick/place stage is out of order')
                if recovery:
                    raise ValueError('pick/place cannot bypass observations through recovery')
                expected = {'checked': True, 'segments_sha256': segment_digest(segments),
                            'model_sha256': command['model_sha256'],
                            'control_config_sha256': CONTROL_CONFIG_SHA256}
                if command.get('validation') != expected:
                    raise ValueError('pick/place requires matching local collision validation')
                issue = stage_segments_issue(stage, segments) or pick_observation_issue(
                    state, received, now, stage, reference)
            else:
                issue = (recovery_issue(state, received, now) if recovery else
                         observation_issue(state, received, now, require_base=True,
                                           initialized=True))
            if issue:
                raise ValueError(issue)
            if (not isinstance(reference, dict)
                    or type(reference.get("captured_unix_ns")) is not int
                    or not 0 <= state["captured_unix_ns"] - reference["captured_unix_ns"]
                    <= 750_000_000):
                raise ValueError("local plan reference is stale")
            scene = copy.deepcopy(state)
            if self.task == 'pick_t' and (stage in CARRYING or (
                    stage in ('descend', 'close') and
                    not scene['objects'].get('wood_t', {}).get('observed'))):
                scene['objects']['wood_t'] = reference['objects']['wood_t']
            if not recovery and not same_scene(reference, scene):
                raise ValueError("scene changed after local planning")
            if self.task == 'pick_t':
                self.robot.motor.submit(reference, segments, stage=stage)
            else:
                self.robot.motor.submit(reference, segments, recovery=recovery)
            self.job_id = command["id"]
            self._record("plans", command)
        elif operation == "push_result":
            result = command.get("result")
            if not isinstance(result, dict):
                raise ValueError("local result must be an object")
            self._record("local-results", result)
        self.last_command = self.clock()
        return self.status()

    def _record(self, name, value):
        with (self.output / (name + ".jsonl")).open("a") as log:
            log.write(json.dumps(value, allow_nan=False) + "\n")

    def status(self):
        return {"planning_location": "local", "execution_location": "remote",
                "execution_enabled": self.robot.motor is not None, "job_id": self.job_id,
                "motor": self.robot.motor.status() if self.robot.motor else None,
                "task": self.task,
                "output": str(self.output)}

    def close(self):
        if self.robot.motor:
            self.robot.motor.cancel("local session closed: hold")
