"""Joint-space leader/follower mapping, following LeRobot get_action/send_action."""

import json
import math

from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
from lerobot_arm_control.transport.realtime import JOINTS


def load_follow_config(path=None):
    path = path or workspace_root()/'configs/follow.json'
    config = json.loads(path.read_text())
    if config.get('version') != 1 or config['leader'] == config['follower']:
        raise ValueError('follow requires two different configured arms')
    if config.get('controller_profile') != 'native_position':
        raise ValueError('follow requires the native_position controller profile')
    for name, low, high in (('hz', 1, 60), ('max_speed_rad_s', 0, 2),
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
    for name, maximum in (('max_correction_degrees', 2), ('margin_degrees', 1),
                          ('target_tolerance_degrees', .5), ('timeout_seconds', 10)):
        value = initialization[name]
        if (type(value) not in (int, float) or not math.isfinite(value)
                or not 0 < value <= maximum):
            raise ValueError(f'invalid initialization parameter: {name}')
    if initialization['target_tolerance_degrees'] >= initialization['margin_degrees']:
        raise ValueError('initialization tolerance must be smaller than its margin')
    return config


def native_position_profile():
    """Send nominal targets through the servo's existing position controller."""
    return {'name': 'native_position', 'version': 1, 'joints': {
        joint: {'kp': 0, 'ki': 0, 'max_correction_rad': 0, 'correction_rate_rad_s': 0}
        for joint in JOINTS}}


def initialization_target(config, measured, limits, encoder_limits):
    """Recover only small model-limit violations, always within physical travel."""
    target = JointFollower.validate(measured)
    maximum = math.radians(config['initialization']['max_correction_degrees'])
    margin = math.radians(config['initialization']['margin_degrees'])
    for index, joint in enumerate(JOINTS):
        value = target[index]
        if not encoder_limits[joint][0] <= value <= encoder_limits[joint][1]:
            raise ValueError(f'{joint}: outside calibrated encoder travel; reposition manually')
        low, high = limits[index]
        if value < low or value > high:
            if max(low-value, value-high)+margin > maximum or high-low <= 2*margin:
                raise ValueError(f'{joint}: too far outside model range; reposition manually')
            target[index] = low+margin if value < low else high-margin
    return target


class JointFollower:
    def __init__(self, config, initial, limits=None):
        self.config = config
        self.limits = limits or joint_ranges()
        self.target = self.validate(initial)
        self.holding = False

    @staticmethod
    def validate(qpos):
        if len(qpos) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in qpos):
            raise ValueError('follow requires six finite joint angles')
        return list(qpos)

    def hold(self, measured):
        self.target = self.validate(measured)
        self.holding = True

    def resume(self, measured):
        self.target = self.validate(measured)
        self.holding = False

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
        maximum = self.config['max_speed_rad_s']*min(elapsed, self.config['max_sample_age_seconds'])
        for index, bounds in enumerate(self.limits):
            goal = max(bounds[0], min(bounds[1], desired[index]))
            previous = self.target[index]
            self.target[index] = previous+max(-maximum, min(maximum, goal-previous))
        return list(self.target)
