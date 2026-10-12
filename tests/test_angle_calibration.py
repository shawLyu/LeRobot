import pytest

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.transport.realtime import JOINTS, preview_joint_target


def samples(raw=2000, torque=0):
    return [{"seq": i, "time_unix_ns": i * 50_000_000,
             "raw": {joint: raw for joint in JOINTS},
             "telemetry": {"Torque_Enable": {joint: torque for joint in JOINTS},
                           "Status": {joint: 0 for joint in JOINTS}}} for i in range(9)]


@pytest.mark.parametrize("direction", [1, -1])
def test_three_measured_points_verify_direction_and_roundtrip(tmp_path, direction):
    path = tmp_path / "angles.json"
    calibration = AngleCalibration(path, {"port": "verified-usb"})
    for degrees, raw in [(0, 2000), (30, 2000 + direction * 341),
                         (60, 2000 + direction * 683)]:
        calibration.capture("wrist_roll", degrees, samples(raw))
    restored = AngleCalibration(path, {"port": "verified-usb"})
    assert restored.count() == 1
    assert restored.fit("wrist_roll")["radians_per_tick"] * direction > 0
    raw = {joint: 2000 for joint in JOINTS}
    assert restored.angles(raw, [9] * 6)[4] == 0
    assert restored.angles(raw, [9] * 6)[0] == 9
    with pytest.raises(ValueError, match="identity"):
        AngleCalibration(path, {"port": "different-usb"})


def test_moving_or_powered_arm_cannot_be_sampled(tmp_path):
    calibration = AngleCalibration(tmp_path / "angles.json", {})
    with pytest.raises(ValueError, match="torques"):
        calibration.capture("gripper", 0, samples(torque=1))
    moving = samples()
    moving[-1]["raw"]["gripper"] += 10
    with pytest.raises(ValueError, match="moving"):
        calibration.capture("gripper", 0, moving)
    with pytest.raises(ValueError, match="nine"):
        calibration.capture("gripper", 0, samples()[:4])
    assert not calibration.path.exists()


def test_failed_independent_check_does_not_corrupt_saved_points(tmp_path):
    calibration = AngleCalibration(tmp_path / "angles.json", {})
    calibration.capture("gripper", 0, samples())
    calibration.capture("gripper", 30, samples(2341))
    with pytest.raises(ValueError, match="check error"):
        calibration.capture("gripper", 60, samples(2500))
    assert "60" not in calibration.points["gripper"]
    assert calibration.count() == 0


@pytest.mark.parametrize("direction", [1, -1])
def test_native_span_failure_reports_observed_values_and_preserves_capture(tmp_path, direction):
    calibration = AngleCalibration(tmp_path / "angles.json", {})
    calibration.capture("shoulder_pan", 0, samples(1973))
    calibration.capture("shoulder_pan", 30, samples(1973 + direction * 382))
    saved = calibration.path.read_bytes()
    with pytest.raises(ValueError, match="outside native-encoder guard") as error:
        calibration.capture("shoulder_pan", 60, samples(1973 + direction * 764))
    message = str(error.value)
    assert f"{direction * 382:+.1f} ticks" in message
    assert "341.3 +/-10% (307.2..375.5)" in message
    assert "33.57 degrees" in message
    assert f"60-degree check raw={1973 + direction * 764}" in message
    assert "60" not in calibration.points["shoulder_pan"]
    assert calibration.path.read_bytes() == saved


def test_gripper_verification_starts_clear_of_calibrated_endpoints(tmp_path):
    calibration = AngleCalibration(tmp_path / "angles.json", {})
    for degrees, raw in [(0, 1620), (30, 1994), (60, 2361)]:
        calibration.capture("gripper", degrees, samples(raw))
    with pytest.raises(ValueError, match="endpoints"):
        calibration.check_gripper_test({"start_raw": 1478, "target_raw": 1510})
    calibration.check_gripper_test({"start_raw": 1800, "target_raw": 1832})


def test_leader_trigger_uses_measured_0_15_30_degrees_and_restores_its_convention(tmp_path):
    path = tmp_path/'leader.json'
    calibration = AngleCalibration(path, {}, leader_trigger=True)
    assert calibration.reference_angles('shoulder_pan') == (0, 30, 60)
    assert calibration.reference_angles('gripper') == (0, 15, 30)
    for degrees, raw in [(0, 2000), (15, 2171), (30, 2341)]:
        calibration.capture('gripper', degrees, samples(raw))
    restored = AngleCalibration(path, {})
    assert restored.count() == 1
    assert restored.reference_angles('gripper') == (0, 15, 30)
    assert restored.fit('gripper')['radians_per_tick'] == pytest.approx(.001531, abs=1e-6)
    with pytest.raises(ValueError, match='reference angles'):
        restored.capture('gripper', 60, samples(2683))


class ReadOnlyBus:
    values = {"Status": 0, "Present_Position": 2000,
              "Min_Position_Limit": 1000, "Max_Position_Limit": 3000}

    def read(self, register, joint, normalize):
        assert joint == "shoulder_pan"
        assert normalize is False
        return self.values[register]

    def write(self, *args, **kwargs):
        raise AssertionError("preview must never write a motor register")


def test_unified_window_preview_uses_actual_feedback_without_writes():
    result = preview_joint_target(ReadOnlyBus(), "shoulder_pan", 20)
    assert result["start_raw"] == 2000
    assert result["target_raw"] == 2020
    assert result["executed"] is False
    with pytest.raises(ValueError):
        preview_joint_target(ReadOnlyBus(), "shoulder_pan", 40)


def test_identity_mismatch_names_arm_fields_and_joints_without_rewriting(tmp_path):
    import json

    path = tmp_path/'arm1_mujoco.json'
    saved = {'version': 1, 'fingerprint': {'homing_offsets': {'gripper': 1844},
                                         'raw_limits': {'gripper': [1512, 2759]}},
             'points': {}}
    original = json.dumps(saved)
    path.write_text(original)
    changed = {'homing_offsets': {'gripper': -1770}, 'raw_limits': {'gripper': [1353, 2652]}}
    with pytest.raises(ValueError) as caught:
        AngleCalibration(path, changed)
    message = str(caught.value)
    assert all(word in message for word in ['arm1_mujoco.json', 'homing_offsets',
                                            'raw_limits', 'gripper', 'recalibrate'])
    assert path.read_text() == original
