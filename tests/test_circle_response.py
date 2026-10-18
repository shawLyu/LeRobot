"""Known synthetic circle speed and injected vibration, never motor access."""
import json
import math

import numpy as np
import pytest

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.diagnostics.circle_response import response_report
from lerobot_arm_control.robot.so101.joints import JOINTS
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle


def recorded_circle(tmp_path, vibration, *, laps=3, ramp=2):
    source = resource_path('data/trajectories/table-circle-8cm.json')
    plan = json.loads(source.read_text())
    plan['period_seconds'] = 6
    plan['execution_profile'].update(angular_speed_deg_s=60, ramp_seconds=ramp)
    path = tmp_path/'plan.json'
    path.write_text(json.dumps(plan))
    circle = Circle(path)
    calibration_path = resource_path('data/calibration/follower_mujoco.json')
    calibration = AngleCalibration(calibration_path,
                                   json.loads(calibration_path.read_text())['fingerprint'])
    circle.start(circle.points[0], 0, laps=laps)
    frames = []
    for time in np.arange(circle.entry_seconds, circle.entry_seconds+laps*6+ramp, 1/60):
        q = circle.sample(time)
        raw = {}
        for i, joint in enumerate(JOINTS):
            fit = calibration.fit(joint)
            measured = q[i]+(vibration*math.sin(2*math.pi*6*time) if i == 1 else 0)
            raw[joint] = round(measured/fit['radians_per_tick']+fit['zero_raw'])
        frames.append({'circle': circle.status(time), 'raw': raw,
                       'nominal_qpos': dict(zip(JOINTS, q)),
                       'telemetry': {'Status': dict.fromkeys(JOINTS, 0)}})
    trace = tmp_path/f'trace-{vibration}.json'
    trace.write_text(json.dumps({'frames': frames, 'control_config_sha256': 'test',
                                'metadata': {'plan_sha256': file_digest(path)}}))
    return trace, path


def test_circle_response_recovers_known_sixty_degree_speed_and_quantization(tmp_path):
    trace, plan = recorded_circle(tmp_path, 0)
    result = response_report(trace, plan, tmp_path/'response', plots=False)
    assert result['steady_phase_fit_speed_deg_s'] == pytest.approx(60, abs=.03)
    assert result['tracking_max_mm'] < 1
    assert result['tcp_hf_error_rms_mm'] < .3
    assert result['status_flags'] == 0
    assert (tmp_path/'response.json').exists()


def test_high_frequency_score_detects_injected_six_hz_vibration(tmp_path):
    trace, plan = recorded_circle(tmp_path, .004)
    result = response_report(trace, plan, tmp_path/'vibrating', plots=False)
    assert result['per_joint']['shoulder_lift']['hf_error_rms_degrees'] > .14
    assert result['tcp_hf_error_rms_mm'] > .4


def test_one_lap_with_long_ramps_keeps_curves_without_claiming_steady_speed(tmp_path):
    trace, plan = recorded_circle(tmp_path, 0, laps=1, ramp=6)
    result = response_report(trace, plan, tmp_path/'ramps', plots=True)
    assert result['steady_phase_fit_speed_deg_s'] is None
    assert result['tcp_hf_error_rms_mm'] is None
    assert result['tracking_max_mm'] < 1
    assert (tmp_path/'ramps.png').exists() and (tmp_path/'ramps-joints.png').exists()


def test_response_rejects_a_changed_plan_instead_of_measuring_the_wrong_circle(tmp_path):
    trace, plan = recorded_circle(tmp_path, 0)
    changed = json.loads(plan.read_text())
    changed['radius_m'] = .081
    plan.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match='executed plan'):
        response_report(trace, plan, tmp_path/'invalid', plots=False)
