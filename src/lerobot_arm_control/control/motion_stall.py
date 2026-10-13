"""Latch a loaded arm joint that cannot advance toward its commanded position."""

import math


class MotionStall:
    def __init__(self, parameters):
        self.parameters = dict(parameters)
        expected = {'minimum_load_raw', 'stall_seconds', 'command_error_rad',
                    'reference_error_rad', 'motion_tolerance_rad'}
        if (set(parameters) != expected
                or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
                       for v in parameters.values())
                or not 1 <= parameters['minimum_load_raw'] <= 1023
                or not .1 <= parameters['stall_seconds'] <= 1
                or any(parameters[k] > .2 for k in expected if k.endswith('_rad'))):
            raise ValueError('invalid loaded-stall settings')
        self.reset()

    def reset(self):
        self.candidates = {}

    def update(self, joint, reference, measured, command, load, now):
        p = self.parameters
        error = command-measured
        blocked = (load is not None and abs(load) >= p['minimum_load_raw']
                   and abs(error) >= p['command_error_rad']
                   and abs(reference-measured) >= p['reference_error_rad']
                   and (reference-measured)*error > 0)
        if not blocked:
            self.candidates.pop(joint, None)
            return False
        direction = 1 if error > 0 else -1
        previous = self.candidates.get(joint)
        if (previous is None or now < previous[0] or previous[2] != direction
                or abs(measured-previous[1]) > p['motion_tolerance_rad']):
            self.candidates[joint] = (now, measured, direction)
            return False
        return now-previous[0] >= p['stall_seconds']
