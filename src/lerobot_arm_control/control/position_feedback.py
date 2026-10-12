"""Shared bounded position PID around the native servo position loop.

All quantities are model radians. Position correction preserves the nominal
trajectory. Optional native P adjustments require locked, volatile storage and
are restored by ManualControl; torque and protection settings are unchanged.
"""

import math
from copy import deepcopy

from lerobot_arm_control.transport.realtime import JOINTS


class PositionFeedback:
    def __init__(self, profile, limits):
        if not isinstance(profile, dict):
            raise ValueError("invalid position feedback profile")
        self.profile = deepcopy(profile)
        if self.profile.get("version") != 1 or set(self.profile.get("joints", {})) != set(JOINTS):
            raise ValueError("invalid position feedback profile")
        stream = self.profile.get("native_stream")
        if stream is not None and (not {"acceleration", "velocity_ticks_s"} <= set(stream)
                                   <= {"acceleration", "velocity_ticks_s", "locked_p_coefficients"}
                                   or any(type(stream[k]) is not int for k in
                                          ("acceleration", "velocity_ticks_s"))
                                   or not 1 <= stream["acceleration"] <= 254
                                   or not 1 <= stream["velocity_ticks_s"] <= 3000):
            raise ValueError("invalid native streaming profile")
        coefficients = stream.get('locked_p_coefficients', {}) if stream else {}
        if (not isinstance(coefficients, dict) or not set(coefficients) <= set(JOINTS[:5])
                or any(type(v) is not int or not 16 <= v <= 32
                       for v in coefficients.values())):
            raise ValueError('invalid temporary locked P coefficients')
        self.limits = limits
        self.integral = dict.fromkeys(JOINTS, 0.0)
        self.correction = dict.fromkeys(JOINTS, 0.0)
        self.feedback_correction = dict.fromkeys(JOINTS, 0.0)
        self.velocity_feedforward = dict.fromkeys(JOINTS, 0.0)
        self.reference_direction = dict.fromkeys(JOINTS, 0)
        self.moving_samples = dict.fromkeys(JOINTS, 0)
        self.previous_reference = dict.fromkeys(JOINTS)
        self.reference_velocity = dict.fromkeys(JOINTS, 0.0)
        self.filtered_error = dict.fromkeys(JOINTS, 0.0)
        self.previous_measurement = dict.fromkeys(JOINTS)
        self.measured_velocity = dict.fromkeys(JOINTS, 0.0)
        self.derivative = dict.fromkeys(JOINTS, 0.0)
        for parameters in self.profile["joints"].values():
            required = {"kp", "ki", "max_correction_rad", "correction_rate_rad_s"}
            optional = {"kd", "derivative_filter_seconds", "velocity_feedforward_s",
                        "error_filter_seconds", "error_deadband_rad", "feedforward_rate_rad_s",
                        "reference_filter_seconds",
                        "friction_feedforward_rad", "friction_velocity_rad_s"}
            if (not required <= set(parameters) <= required | optional
                    or any(type(v) not in (int, float) or not math.isfinite(v) or v < 0
                           for v in parameters.values())
                    or parameters["max_correction_rad"] > 0.2
                    or parameters["correction_rate_rad_s"] > 0.3
                    or parameters.get("velocity_feedforward_s", 0) > 0.5
                    or parameters.get("feedforward_rate_rad_s", 4) > 6
                    or not .01 <= parameters.get("reference_filter_seconds", .05) <= .5
                    or parameters.get("error_filter_seconds", 0) > 0.5
                    or parameters.get("error_deadband_rad", 0) > 0.02
                    or parameters.get("kd", 0) > .5
                    or (parameters.get("kd", 0) > 0 and
                        not .01 <= parameters.get("derivative_filter_seconds", 0) <= .5)
                    or parameters.get("friction_feedforward_rad", 0) > 0.08
                    or (parameters.get("friction_feedforward_rad", 0) > 0
                        and not .01 <= parameters.get("friction_velocity_rad_s", 0) <= .15)):
                raise ValueError("invalid position correction gains or envelope")

    def reset(self, joints=JOINTS):
        for joint in joints:
            self.integral[joint] = self.correction[joint] = 0.0
            self.feedback_correction[joint] = self.velocity_feedforward[joint] = 0.0
            self.reference_direction[joint] = self.moving_samples[joint] = 0
            self.previous_reference[joint] = None
            self.reference_velocity[joint] = 0.0
            self.filtered_error[joint] = 0.0
            self.previous_measurement[joint] = None
            self.measured_velocity[joint] = self.derivative[joint] = 0.0

    def finish_reference(self, joints):
        """End trajectory feedforward while retaining learned static load compensation."""
        for joint in joints:
            self.previous_reference[joint] = None
            self.reference_velocity[joint] = self.velocity_feedforward[joint] = 0.0
            self.reference_direction[joint] = self.moving_samples[joint] = 0

    def target(self, joint, reference, measured, elapsed):
        if not all(math.isfinite(v) for v in (reference, measured, elapsed)) or elapsed < 0:
            raise ValueError("invalid position feedback input")
        # Do not accumulate through a stalled control tick. The remote watchdog
        # still handles connection failure; this is anti-windup, not an error abort.
        dt = min(elapsed, 0.05)
        p = self.profile["joints"][joint]
        previous_measurement = self.previous_measurement[joint]
        self.previous_measurement[joint] = measured
        # Derivative on measurement avoids a kick on target steps. Estimate in
        # model radians, so reversed encoder calibrations have the correct sign.
        if elapsed > .1:
            self.measured_velocity[joint] = 0.0
        elif previous_measurement is not None and elapsed > 0:
            velocity = (measured-previous_measurement)/elapsed
            tau = p.get('derivative_filter_seconds', .05)
            alpha = -math.expm1(-dt/tau) if tau else 1.
            self.measured_velocity[joint] += alpha*(velocity-self.measured_velocity[joint])
        self.derivative[joint] = -p.get('kd', 0)*self.measured_velocity[joint]
        low, high = self.limits[joint]
        reference = max(low, min(high, reference))
        previous = self.previous_reference[joint]
        # Differentiate the reference, not the measured encoder. A short filter
        # attenuates sample noise; the existing correction envelope also bounds
        # interactive target steps. Use actual elapsed time across missed ticks.
        direction = 0
        if previous is not None and 0 < elapsed <= .1:
            direction = 1 if reference > previous else -1 if reference < previous else 0
            velocity = (reference - previous) / elapsed
            alpha = -math.expm1(-dt / p.get("reference_filter_seconds", .05))
            self.reference_velocity[joint] += alpha * (velocity - self.reference_velocity[joint])
        self.previous_reference[joint] = reference
        self.moving_samples[joint] = (self.moving_samples[joint]+1
                                     if direction and direction == self.reference_direction[joint]
                                     else int(bool(direction)))
        self.reference_direction[joint] = direction
        # Two consecutive moving references distinguish a stream from an isolated
        # position step. Stop/reversal cancels prediction instead of kicking past it.
        velocity_feedforward = (p.get("velocity_feedforward_s", 0)*self.reference_velocity[joint]
                                if self.moving_samples[joint] >= 2 else 0.)
        feedforward = 0.

        if p.get("friction_feedforward_rad", 0):
            # Smooth Coulomb-friction approximation using planned direction.
            # No sign switch on encoder noise, and no discontinuity at reversal.
            feedforward += p["friction_feedforward_rad"] * math.tanh(
                self.reference_velocity[joint] / p["friction_velocity_rad_s"])
        bound = p["max_correction_rad"]
        lower, upper = max(-bound, low - reference), min(bound, high - reference)
        error = reference - measured
        error = math.copysign(max(0.0, abs(error) - p.get("error_deadband_rad", 0)), error)
        filter_seconds = p.get("error_filter_seconds", 0)
        if filter_seconds:
            alpha = -math.expm1(-dt / filter_seconds)
            self.filtered_error[joint] += alpha * (error - self.filtered_error[joint])
            error = self.filtered_error[joint]
        proposed = self.integral[joint] + p["ki"] * error * (dt if elapsed <= .1 else 0)
        demanded = (p["kp"]*error+proposed+self.derivative[joint]
                    +feedforward+velocity_feedforward)
        # Freeze accumulation when it would push farther into magnitude/native
        # saturation. Allow unwinding when the error changes direction.
        if not ((demanded > upper and error > 0) or (demanded < lower and error < 0)):
            self.integral[joint] = max(-bound, min(bound, proposed))
        demanded = max(lower, min(upper, p["kp"] * error + self.integral[joint]
                                  + self.derivative[joint] + feedforward))
        step = p["correction_rate_rad_s"] * dt
        current = self.feedback_correction[joint]
        current += max(-step, min(step, demanded - current))
        self.feedback_correction[joint] = max(lower, min(upper, current))
        ff_step = p.get('feedforward_rate_rad_s', 4.)*dt
        ff = self.velocity_feedforward[joint]
        ff += max(-ff_step, min(ff_step, velocity_feedforward-ff))
        self.velocity_feedforward[joint] = max(-bound, min(bound, ff))
        self.correction[joint] = max(lower, min(upper, current+self.velocity_feedforward[joint]))
        return reference + self.correction[joint]

    def status(self):
        return {"profile": self.profile, "correction_rad": dict(self.correction),
                "velocity_feedforward_rad": dict(self.velocity_feedforward),
                "integral_rad": dict(self.integral), "derivative_rad": dict(self.derivative),
                "measured_velocity_rad_s": dict(self.measured_velocity)}
