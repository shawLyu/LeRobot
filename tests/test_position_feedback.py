import math

import pytest

from lerobot_arm_control.control.position_feedback import PositionFeedback
from lerobot_arm_control.transport.realtime import JOINTS


def feedback():
    return PositionFeedback({"version": 1, "joints": {
        j: {"kp": 0.5, "ki": 0.8, "max_correction_rad": 0.12,
            "correction_rate_rad_s": 0.08} for j in JOINTS}},
        {j: [-1.0, 1.0] for j in JOINTS})


def test_corrects_loaded_position_loop_without_increasing_nominal_target_step():
    controller = feedback()
    q = 0.0
    corrections = []
    for _ in range(2400):
        command = controller.target("elbow_flex", 0.3, q, 1 / 60)
        q += (command - q - 0.05) / 0.15 / 60
        corrections.append(command - 0.3)
    assert abs(q - 0.3) < 0.001
    assert max(abs(v) for v in corrections) <= 0.12
    assert max(abs(b - a) for a, b in zip(corrections, corrections[1:])) <= 0.08 / 60 + 1e-12


def test_saturation_does_not_wind_up_and_reset_clears_old_correction():
    controller = feedback()
    for _ in range(1000):
        assert controller.target("elbow_flex", 1.0, 0.0, 1 / 60) == 1.0
    assert controller.integral["elbow_flex"] == 0
    controller.target("elbow_flex", 0.0, 0.1, 1 / 60)
    assert controller.correction["elbow_flex"] < 0
    controller.reset()
    assert controller.correction["elbow_flex"] == 0
    assert controller.integral["elbow_flex"] == 0


def test_nonfinite_input_rejected_and_long_tick_does_not_accumulate_unboundedly():
    controller = feedback()
    with pytest.raises(ValueError):
        controller.target("elbow_flex", math.nan, 0, 1 / 60)
    controller.target("elbow_flex", 0.1, 0, 100)
    assert controller.correction["elbow_flex"] <= 0.08 * 0.05


def test_velocity_feedforward_reduces_lag_without_encoder_derivative():
    import numpy as np

    def replay(feedforward):
        controller = feedback()
        controller.profile["joints"]["elbow_flex"].update(
            kp=2, ki=3, correction_rate_rad_s=0.18, velocity_feedforward_s=feedforward)
        q = 0.0
        errors = []
        for i in range(1800):
            reference = 0.1 * math.sin(i / 60 * 2 * math.pi / 6)
            command = controller.target("elbow_flex", reference, round(q / .0015) * .0015, 1 / 60)
            q += (command - q - 0.04) / 0.3 / 60
            if i > 600:
                errors.append(reference - q)
        controller.reset()
        assert controller.previous_reference["elbow_flex"] is None
        assert controller.reference_velocity["elbow_flex"] == 0
        return np.sqrt(np.mean(np.square(errors)))

    assert replay(0.30) < replay(0) * 0.3


def test_feedforward_target_jump_obeys_rate_envelope_and_native_boundary():
    controller = feedback()
    controller.profile["joints"]["elbow_flex"]["velocity_feedforward_s"] = .3
    controller.target("elbow_flex", 0, 0, 1 / 60)
    assert controller.target("elbow_flex", .5, 0, 1 / 60) <= .5 + .08 / 60
    assert controller.target("elbow_flex", 1, 0, 1 / 60) == 1


def test_error_filter_rejects_encoder_tick_alternation_and_resets():
    def amplitude(filter_seconds):
        controller = feedback()
        controller.profile["joints"]["elbow_flex"].update(
            kp=2, ki=0, correction_rate_rad_s=.3, error_filter_seconds=filter_seconds)
        commands = [controller.target("elbow_flex", 0, .0015 * (-1)**i, 1 / 60)
                    for i in range(600)]
        controller.reset()
        assert controller.filtered_error["elbow_flex"] == 0
        return max(commands[-120:]) - min(commands[-120:])

    assert amplitude(.12) < amplitude(0) * .15


def test_encoder_deadband_does_not_integrate_small_stationary_errors():
    controller = feedback()
    controller.profile["joints"]["elbow_flex"]["error_deadband_rad"] = .003
    for i in range(600):
        assert controller.target("elbow_flex", 0, .0015 * (-1)**i, 1 / 60) == 0
    assert controller.integral["elbow_flex"] == 0


def test_finishing_ramp_unwinds_correction_within_existing_rate_limit():
    controller = feedback()
    joint = 'elbow_flex'
    controller.integral[joint] = .03
    controller.correction[joint] = .02
    controller.reference_velocity[joint] = .1
    controller.finish_reference([joint])
    assert controller.integral[joint] == 0
    assert controller.reference_velocity[joint] == 0
    assert controller.correction[joint] == .02
    command = controller.target(joint, 0, 0, 1/60)
    assert command > 0
    assert abs(command-.02) <= .08/60+1e-12


def test_smooth_direction_feedforward_stays_bounded_through_reversal():
    controller = feedback()
    controller.profile["joints"]["elbow_flex"].update(
        kp=0, ki=0, friction_feedforward_rad=.032, friction_velocity_rad_s=.04)
    commands = []
    references = [.1 * math.sin(i / 60 * math.pi) for i in range(240)]
    for r in references:
        commands.append(controller.target("elbow_flex", r, r, 1 / 60) - r)
    assert max(abs(v) for v in commands) <= .032
    assert min(commands) < -.02 < .02 < max(commands)
    assert max(abs(b - a) for a, b in zip(commands, commands[1:])) <= .08 / 60 + 1e-12
