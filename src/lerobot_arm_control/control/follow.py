"""Joint-space leader/follower mapping, following LeRobot get_action/send_action."""

import json
import math

from lerobot_arm_control.control.control_defaults import CONTROL, CONTROL_HZ
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
from lerobot_arm_control.transport.realtime import JOINTS


def load_follow_config(path=None):
    path = path or workspace_root()/'configs/follow.json'
    config = json.loads(path.read_text())
    if config.get('version') != 1 or config['leader'] == config['follower']:
        raise ValueError('follow requires two different configured arms')
    if 'controller_profile' in config or 'hz' in config:
        raise ValueError('controller and frequency are shared in configs/control.json')
    config.update(controller_profile=CONTROL['name'], hz=CONTROL_HZ)
    for name, low, high in (('hz', 1, 60), ('entry_speed_rad_s', 0, 2),
                            ('alignment_tolerance_degrees', 0, 2),
                            ('max_sample_age_seconds', 0, .5)):
        value = config[name]
        if type(value) not in (int, float) or not math.isfinite(value) or not low < value <= high:
            raise ValueError(f'invalid follow parameter: {name}')
    for name in ('leader_trigger_degrees', 'follower_gripper_degrees'):
        values = config[name]
        if (len(values) != 2 or any(type(v) not in (int, float) or not math.isfinite(v)
                                  for v in values) or values[0] >= values[1]):
            raise ValueError(f'invalid follow range: {name}')
    initialization = config['initialization']
    if type(initialization['auto_start']) is not bool:
        raise ValueError('initialization auto_start must be boolean')
    for name, maximum in (('max_correction_degrees', 2),
                          ('max_closed_gripper_recovery_degrees', 5), ('margin_degrees', 1),
                          ('target_tolerance_degrees', .5), ('timeout_seconds', 10)):
        value = initialization[name]
        if (type(value) not in (int, float) or not math.isfinite(value)
                or not 0 < value <= maximum):
            raise ValueError(f'invalid initialization parameter: {name}')
    if initialization['target_tolerance_degrees'] >= initialization['margin_degrees']:
        raise ValueError('initialization tolerance must be smaller than its margin')
    return config


def hardware_follow_limits(model_ranges, encoder_limits):
    """Use verified physical jaw travel; keep CAD bounds on arm joints."""
    limits = [tuple(encoder_limits[joint]) if joint == 'gripper' else
              (max(bounds[0], encoder_limits[joint][0]),
               min(bounds[1], encoder_limits[joint][1]))
              for joint, bounds in zip(JOINTS, model_ranges, strict=True)]
    if any(low >= high for low, high in limits):
        raise ValueError('follower has no usable calibrated motion range')
    return limits


def initialization_target(config, measured, limits, encoder_limits):
    """Recover only small model-limit violations, always within physical travel."""
    target = JointFollower.validate(measured)
    margin = math.radians(config['initialization']['margin_degrees'])
    for index, joint in enumerate(JOINTS):
        value = target[index]
        if not encoder_limits[joint][0] <= value <= encoder_limits[joint][1]:
            raise ValueError(f'{joint}: outside calibrated encoder travel; reposition manually')
        low, high = limits[index]
        if value < low or value > high:
            # The physical jaw closes beyond the CAD lower stop. Permit only
            # a small opening recovery; arm joints and upper-stop recovery
            # retain the original two-degree bound.
            bound = ('max_closed_gripper_recovery_degrees'
                     if joint == 'gripper' and value < low else 'max_correction_degrees')
            maximum = math.radians(config['initialization'][bound])
            if max(low-value, value-high)+margin > maximum or high-low <= 2*margin:
                raise ValueError(f'{joint}: too far outside model range; reposition manually')
            target[index] = low+margin if value < low else high-margin
    return target


class StartupFollow:
    """Consume startup permission once; pauses and faults cancel it permanently."""

    def __init__(self, enabled):
        self.pending = enabled

    def update(self, *, ready, inhibited=False, manual_command=False):
        if inhibited or manual_command:
            self.pending = False
        if self.pending and ready:
            self.pending = False
            return True
        return False


class JointFollower:
    def __init__(self, config, initial, limits=None):
        self.config = config
        self.limits = limits or joint_ranges()
        self.target = self.validate(initial)
        self.holding = False
        self.tracking = False

    @staticmethod
    def validate(qpos):
        if len(qpos) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in qpos):
            raise ValueError('follow requires six finite joint angles')
        return list(qpos)

    def hold(self, measured):
        self.target = self.validate(measured)
        self.holding = True
        self.tracking = False

    def resume(self, measured):
        self.target = self.validate(measured)
        self.holding = False
        self.tracking = False

    def step(self, leader, elapsed, *, sample_age=0, map_trigger=True):
        desired = self.validate(leader)
        if (not math.isfinite(elapsed) or elapsed < 0
                or not math.isfinite(sample_age) or sample_age < 0):
            raise ValueError('invalid follow timing')
        if sample_age > self.config['max_sample_age_seconds']:
            self.holding = True
        if self.holding:
            return list(self.target)
        if map_trigger:
            low, high = map(math.radians, self.config['leader_trigger_degrees'])
            fraction = max(0, min(1, (desired[5]-low)/(high-low)))
            closed, opened = map(math.radians, self.config['follower_gripper_degrees'])
            desired[5] = closed+fraction*(opened-closed)
        desired = [max(lo, min(hi, q)) for q, (lo, hi) in zip(desired, self.limits, strict=True)]
        # Slew only while aligning after startup/resume. Applying this ramp to
        # every live action queues an ever older pose when the leader moves faster.
        if map_trigger and all(abs(q-r) <= math.radians(self.config['alignment_tolerance_degrees'])
                               for q, r in zip(desired, self.target, strict=True)):
            self.tracking = True
        if self.tracking and map_trigger:
            self.target = desired
        else:
            maximum = self.config['entry_speed_rad_s']*min(
                elapsed, self.config['max_sample_age_seconds'])
            for index, goal in enumerate(desired):
                previous = self.target[index]
                self.target[index] = previous+max(-maximum, min(maximum, goal-previous))
        return list(self.target)
