import json

import pytest
from test_manual_control import Bus, Calibration

from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.transport.realtime import JOINTS


def setup(tmp_path):
    bus, now = Bus(), [0.0]
    control = ManualControl(bus, Calibration(), [[-1, 1]] * 6,
                            {joint: [1000, 3000] for joint in JOINTS},
                            clock=lambda: now[0], rest_path=tmp_path / "rest.json")
    return bus, control, now


def tick(bus, control, now, seconds=0.05, follow=True):
    if follow:
        for values in bus.registers.values():
            if values["Torque_Enable"]:
                values["Present_Position"] = values["Goal_Position"]
    now[0] += seconds
    control.heartbeat()
    telemetry = {register: bus.sync_read(register) for register in ("Torque_Enable", "Status")}
    telemetry["Moving"] = {joint: 0 for joint in JOINTS}
    control.step(bus.sync_read("Present_Position"), telemetry)


def teach(bus, control, now):
    for _ in range(50):
        tick(bus, control, now)
    control.parking.save()


def test_reset_requires_a_taught_pose_before_any_torque_write(tmp_path):
    bus, control, _ = setup(tmp_path)
    with pytest.raises(ValueError, match="Teach"):
        control.parking.start()
    assert bus.writes == []


@pytest.mark.parametrize("persistent", [False, True])
def test_reset_settling_rechecks_bulk_status_even_with_all_joints_held(tmp_path, persistent):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    control.enable_all()
    control.parking.release_verified = False
    control.parking.start()
    tick(bus, control, now)
    assert control.parking.phase == "settling"
    assert len(control.stopped) == 6
    bus.registers["gripper"]["Status"] = int(persistent)
    telemetry = {key: bus.sync_read(key) for key in ("Torque_Enable", "Status")}
    telemetry["Status"]["gripper"] = 1
    now[0] += .05
    control.heartbeat()
    control.step(bus.sync_read("Present_Position"), telemetry)
    if persistent:
        assert control.parking.phase == "cancelled: hold"
        assert control.last_fault["telemetry"]["Status"]["gripper"] == 1
    else:
        assert control.parking.phase == "settling"
        assert control.last_fault is None
        assert control.status_warning["confirmed_status"] == 0
    assert all(bus.sync_read("Torque_Enable").values())


def test_arrived_reset_freezes_feedback_and_resumes_only_if_pose_leaves_tolerance(tmp_path):
    from lerobot_arm_control.control.position_feedback import PositionFeedback

    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    control.parking.release_verified = False
    control.enable_all()
    control.position_feedback = PositionFeedback({"version": 1, "joints": {
        j: {"kp": .5, "ki": .8, "max_correction_rad": .12,
            "correction_rate_rad_s": .08} for j in JOINTS}}, control.limits)
    control.position_feedback.integral["elbow_flex"] = .05
    control.parking.start()
    tick(bus, control, now, .05, follow=False)
    assert control.parking.phase == "settling"
    assert control.position_feedback.integral["elbow_flex"] == 0
    assert all(reason == "Reset settling: hold" for reason in control.stopped.values())
    bus.registers["elbow_flex"]["Present_Position"] += 8
    tick(bus, control, now, .05, follow=False)
    assert control.parking.phase == "moving"
    assert not control.stopped
    bus.registers["elbow_flex"]["Present_Position"] = 2200
    for _ in range(50):
        tick(bus, control, now, .05, follow=False)
    assert control.parking.phase == "boundary hold"
    assert all(bus.registers[j]["Goal_Position"] == 2200 for j in JOINTS)
    assert all(bus.registers[j]["Torque_Enable"] == 1 for j in JOINTS)


def test_teaching_requires_unpowered_still_feedback_and_survives_reload(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    saved = json.loads((tmp_path / "rest.json").read_text())
    assert saved["raw"] == {joint: 2200 for joint in JOINTS}
    _, reloaded, _ = setup(tmp_path)
    assert reloaded.parking.pose == saved["raw"]
    bus.registers["gripper"]["Torque_Enable"] = 1
    tick(bus, control, now)
    with pytest.raises(ValueError, match="torques OFF"):
        control.parking.save()


def test_moving_unpowered_pose_cannot_be_saved(tmp_path):
    bus, control, now = setup(tmp_path)
    for _ in range(50):
        tick(bus, control, now)
        bus.registers["gripper"]["Present_Position"] += 1
    with pytest.raises(ValueError, match="arm is moving"):
        control.parking.save()


def test_reset_slews_to_pose_then_releases_and_verifies_no_drift(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    for values in bus.registers.values():
        values["Present_Position"] = 2300
    control.parking.start()
    assert all(values["Goal_Position"] == 2300 for values in bus.registers.values())
    for _ in range(180):
        tick(bus, control, now)
    assert control.parking.phase == "unpowered rest"
    assert not any(bus.sync_read("Torque_Enable").values())
    assert all(abs(raw - 2200) <= 5 for raw in bus.sync_read("Present_Position").values())
    writes = list(bus.writes)
    for _ in range(10):
        tick(bus, control, now)
    assert bus.writes == writes  # No target writes after torque release.


@pytest.mark.parametrize("cause", ["fault", "cancel", "watchdog"])
def test_interruption_keeps_support_instead_of_releasing_at_wrong_pose(tmp_path, cause):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    for values in bus.registers.values():
        values["Present_Position"] = 2300
    control.parking.start()
    if cause == "fault":
        bus.registers["gripper"]["Status"] = 1
        tick(bus, control, now)
    elif cause == "cancel":
        control.stop()
    else:
        now[0] += 0.8
        telemetry = {register: bus.sync_read(register) for register in ("Torque_Enable", "Status")}
        control.step(bus.sync_read("Present_Position"), telemetry)
    assert control.parking.phase == "cancelled: hold"
    assert all(bus.sync_read("Torque_Enable").values())


def test_drift_after_release_is_reported_without_reactivation(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    control.parking.start()
    for _ in range(60):
        tick(bus, control, now)
    assert control.parking.phase == "verifying release"
    bus.registers["elbow_flex"]["Present_Position"] += 9
    tick(bus, control, now)
    assert control.parking.phase == "released pose moved"
    assert not any(bus.sync_read("Torque_Enable").values())


def test_reset_already_at_unpowered_rest_never_enables_motors(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    bus.writes.clear()
    control.parking.start()
    for _ in range(100):
        tick(bus, control, now)
    assert control.parking.phase == "unpowered rest"
    assert all(register == "Torque_Enable" and value == 0 for register, _, value in bus.writes)


def test_all_motor_preflight_finishes_before_first_enable_write(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    bus.writes.clear()
    bus.registers["gripper"]["Present_Voltage"] = 39
    with pytest.raises(ValueError, match="voltage"):
        control.parking.start()
    assert bus.writes == []


def test_model_edge_rest_uses_native_bounds_and_supported_pose(tmp_path):
    bus, control, now = setup(tmp_path)
    joint = "shoulder_lift"
    bus.registers[joint]["Present_Position"] = 1320  # -1.02 rad: beyond nominal model edge.
    original_limits = dict(control.limits)
    teach(bus, control, now)
    saved = json.loads((tmp_path / "rest.json").read_text())
    assert saved["observed_raw"][joint] == 1320
    assert saved["raw"][joint] == control.bounded_raw(joint, 1320)
    assert saved["release_verified"]
    bus.registers[joint]["Present_Position"] = 1500
    control.enable(joint)
    control.target(joint, -1.02)
    control.parking.start()
    for _ in range(200):
        tick(bus, control, now)
    assert control.parking.phase == "unpowered rest"
    assert abs(bus.registers[joint]["Present_Position"]
               - control.bounded_raw(joint, 1320)) <= 5
    assert control.limits == original_limits
    assert not any(bus.sync_read("Torque_Enable").values())
    assert all(value >= 1000 for register, name, value in bus.writes
               if register == "Goal_Position" and name == joint)


@pytest.mark.parametrize("raw", [999, 3001])
def test_rest_must_be_within_exact_native_calibration(tmp_path, raw):
    bus, control, now = setup(tmp_path)
    bus.registers["shoulder_lift"]["Present_Position"] = raw
    for _ in range(50):
        tick(bus, control, now)
    with pytest.raises(ValueError, match="outside calibrated limits"):
        control.parking.save()
    assert bus.writes == []


def test_rest_includes_native_endpoint_without_extra_margin(tmp_path):
    bus, control, now = setup(tmp_path)
    joint = "shoulder_lift"
    # Make the EEPROM limit, rather than the model, the binding rest boundary.
    limits = {name: [1300, 3000] for name in JOINTS}
    control = ManualControl(bus, Calibration(), [[-1, 1]] * 6, limits,
                            clock=lambda: now[0], rest_path=tmp_path / "rest.json")
    bus.registers[joint]["Present_Position"] = 1307
    for _ in range(50):
        tick(bus, control, now)
    control.parking.save()
    assert control.parking.pose[joint] == 1307
    assert bus.writes == []


def test_legacy_supported_pose_is_preserved_with_native_calibration(tmp_path):
    bus, control, now = setup(tmp_path)
    joint = "shoulder_lift"
    bus.registers[joint]["Present_Position"] = 1320
    teach(bus, control, now)
    path = tmp_path / "rest.json"
    saved = json.loads(path.read_text())
    saved["version"] = 1
    saved["raw"] = saved.pop("observed_raw")
    path.write_text(json.dumps(saved))
    _, reloaded, _ = setup(tmp_path)
    migrated = json.loads(path.read_text())
    assert migrated["version"] == 2
    assert migrated["observed_raw"][joint] == 1320
    assert migrated["raw"][joint] == 1320
    assert reloaded.parking.release_verified


def test_boundary_rounding_stays_inside_for_both_calibration_directions(tmp_path):
    _, control, _ = setup(tmp_path)
    for raw in (1000, 3000):
        target = control.bounded_raw("shoulder_lift", raw)
        assert control.limits["shoulder_lift"][0] <= control.angle("shoulder_lift", target)
        assert control.angle("shoulder_lift", target) <= control.limits["shoulder_lift"][1]

    class ReversedCalibration(Calibration):
        def fit(self, joint):
            return {"zero_raw": 2000, "radians_per_tick": -0.0015}

    control.calibration = ReversedCalibration()
    for raw in (1000, 3000):
        target = control.bounded_raw("shoulder_lift", raw)
        assert control.limits["shoulder_lift"][0] <= control.angle("shoulder_lift", target)
        assert control.angle("shoulder_lift", target) <= control.limits["shoulder_lift"][1]


def test_quintic_progress_is_monotonic_with_smooth_endpoints_and_bounded_peak_speed():
    step = 0.0001
    values = [smooth_fraction(i * step) for i in range(10001)]
    assert values[0] == 0 and values[-1] == 1
    speeds = [(b - a) / step for a, b in zip(values, values[1:])]
    assert min(speeds) >= 0
    assert max(speeds) <= 1.875 + 1e-6
    assert speeds[0] < 1e-6 and speeds[-1] < 1e-6
    assert abs((speeds[1] - speeds[0]) / step) < 0.01
    assert abs((speeds[-1] - speeds[-2]) / step) < 0.01


def test_reset_trajectory_runs_on_remote_clock_without_position_commands(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    bus.registers["shoulder_lift"]["Present_Position"] = 2300
    control.parking.start()  # This is the only Reset command.
    start = control.parking.started_at
    duration = control.parking.duration
    now[0] = start + duration * 0.5
    control.heartbeat()
    telemetry = {key: bus.sync_read(key) for key in ("Torque_Enable", "Status")}
    control.step(bus.sync_read("Present_Position"), telemetry)
    assert abs(control.targets["shoulder_lift"] - control.angle("shoulder_lift", 2250)) < 1e-9
    for _ in range(200):
        tick(bus, control, now)
    assert control.parking.phase == "unpowered rest"


def test_reset_lag_does_not_trip_a_tracking_or_elapsed_timeout(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    for values in bus.registers.values():
        values["Present_Position"] = 2400
    control.parking.start()
    tick(bus, control, now, seconds=60, follow=False)
    assert control.parking.active
    assert not control.stopped
    assert control.last_fault is None
    assert all(bus.sync_read("Torque_Enable").values())
    assert {joint: control.raw(joint, control.targets[joint]) for joint in JOINTS} == (
        dict.fromkeys(JOINTS, 2200))
    assert all(1000 <= value < 2200 for value in bus.sync_read("Goal_Position").values())


def test_bounds_change_does_not_promote_unverified_release(tmp_path):
    bus, control, now = setup(tmp_path)
    teach(bus, control, now)
    path = tmp_path / "rest.json"
    saved = json.loads(path.read_text())
    saved["release_verified"] = False
    path.write_text(json.dumps(saved))
    _, reloaded, _ = setup(tmp_path)
    assert not reloaded.parking.release_verified
