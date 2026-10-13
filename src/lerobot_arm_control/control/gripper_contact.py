"""Detect a loaded, stalled closing jaw; hold position until the user opens it."""

import math


class GripperContact:
    def __init__(self, parameters):
        self.parameters = dict(parameters)
        required = {'minimum_load_raw', 'stall_seconds', 'tracking_error_rad',
                    'motion_tolerance_rad', 'release_margin_rad'}
        if (set(parameters) != required
                or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
                       for v in parameters.values())
                or not 1 <= parameters['minimum_load_raw'] <= 1023
                or not .05 <= parameters['stall_seconds'] <= .5
                or any(parameters[key] > .1 for key in required
                       if key.endswith('_rad'))):
            raise ValueError('invalid gripper contact settings')
        self.reset()

    def reset(self):
        self.since = None
        self.anchor = None
        self.hold_angle = None

    def update(self, reference, measured, command, load, now):
        p = self.parameters
        if self.hold_angle is not None:
            if reference > self.hold_angle+p['release_margin_rad']:
                self.reset()
            else:
                return self.hold_angle
        blocked = (load is not None and abs(load) >= p['minimum_load_raw']
                   and measured-reference >= p['tracking_error_rad']
                   and measured-command >= p['tracking_error_rad'])
        if not blocked:
            self.since = self.anchor = None
        elif (self.since is None or now < self.since
              or abs(measured-self.anchor) > p['motion_tolerance_rad']):
            self.since, self.anchor = now, measured
        elif now-self.since >= p['stall_seconds']:
            self.hold_angle = measured
            return measured
        return None

    def status(self):
        return {'active': self.hold_angle is not None, 'hold_angle_rad': self.hold_angle,
                'parameters': self.parameters}
