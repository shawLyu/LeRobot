import math

import pytest

from lerobot_arm_control.diagnostics.response_metrics import step_metrics


def test_first_order_response_has_expected_rise_and_settling():
    times = [i/60 for i in range(241)]
    for direction in (1, -1):
        measured = [10+direction*3*(1-math.exp(-t/.2)) for t in times]
        result = step_metrics(times, measured, 10, 10+direction*3)
        assert result['rise_time_10_90_s'] == pytest.approx(.2*math.log(9), abs=.002)
        assert result['settling_time_s'] == pytest.approx(.2*math.log(6), abs=1/60)
        assert result['overshoot_percent'] == 0
        assert result['passed']


def test_stuck_or_late_motor_cannot_pass_as_zero_rise_time():
    times = [i/60 for i in range(241)]
    result = step_metrics(times, [0.]*len(times), 0, 3)
    assert result['rise_time_10_90_s'] is None
    assert result['settling_time_s'] is None
    assert not result['passed']
    late = [0 if t < 3.8 else 3 for t in times]
    assert step_metrics(times, late, 0, 3)['settling_time_s'] is None


def test_overshoot_and_feedback_dropout_are_reported():
    times = [i/60 for i in range(241)]
    q = [3.]*len(times)
    q[20] = 4.5
    assert step_metrics(times, q, 0, 3)['overshoot_percent'] == 50
    assert not step_metrics(times, q, 0, 3)['passed']
    times[90:] = [t+.2 for t in times[90:]]
    assert not step_metrics(times, [3.]*len(times), 0, 3)['passed']


def test_incomplete_or_invalid_windows_rejected():
    with pytest.raises(ValueError):
        step_metrics([0, .1, .1], [0, 1, 2], 0, 3)
