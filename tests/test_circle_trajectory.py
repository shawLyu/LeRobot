import json
import math

import mujoco
import numpy as np
import pytest
from fakes import Bus, Calibration

from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.planning.circle_simulation import (
    check_path,
    entry_points,
    validate_hardware_entry,
    verify,
)
from lerobot_arm_control.tasks.circle.circle_trajectory import ROOT, Circle
from lerobot_arm_control.transport.realtime import JOINTS


def test_cartesian_circle_closes_and_remains_within_speed_bound():
    circle = Circle()
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/so101/scene_mujoco.xml"))
    data = mujoco.MjData(model)
    check_path(model, circle.points)
    for q in circle.points:
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        position = data.site("gripperframe").xpos
        offset = position - circle.center
        assert np.linalg.norm(offset[[0, 2]]) == pytest.approx(circle.radius, abs=1e-6)
        assert offset[1] == pytest.approx(0, abs=1e-6)
    circle.start(circle.points[0], 0)
    a = np.asarray(circle.sample(circle.entry_seconds + 4 + circle.period - 0.0001))
    b = np.asarray(circle.sample(circle.entry_seconds + 4 + circle.period + 0.0001))
    assert np.max(np.abs(a - b)) < circle.planned_peak_speed * 0.0002 + 1e-9


def test_two_dynamic_laps_track_circle_without_collisions(tmp_path):
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/so101/scene_mujoco.xml"))
    report = verify(model, ROOT / "data/trajectories/circle.json", tmp_path / "result.json")
    assert report["passed"]


def test_backends_share_timing_but_simulation_instance_cannot_enable_hardware():
    simulation = Circle(simulation=True)
    with pytest.raises(ValueError, match="cannot control hardware"):
        simulation.start_hardware(None, None)
    hardware = Circle()
    assert hardware.period == 16
    assert hardware.entry_speed == 0.8
    assert hardware.ramp_seconds == 1
    assert hardware.status()["profile_sha256"] == simulation.status()["profile_sha256"]
    simulation.start(simulation.points[0], 0)
    hardware.start(hardware.points[0], 0)
    assert simulation.period == 16
    assert simulation.entry_seconds == 0.35
    assert simulation.ramp_seconds == 1
    for now in np.linspace(0, 15, 1501):
        assert simulation.sample(now) == hardware.sample(now)


def test_shared_profile_preserves_closed_smooth_path():
    simulation = Circle(simulation=True)
    hardware = Circle()
    initial = list(hardware.points[0])
    initial[1] -= 1.0
    simulation.start(initial, 0, laps=2)
    hardware.start(initial, 0, laps=2)
    assert hardware.entry_seconds == pytest.approx(1.875 / .8)
    assert hardware.status()["profile_sha256"] == simulation.status()["profile_sha256"]
    for now in np.linspace(0, 16, 961):
        assert simulation.sample(now) == hardware.sample(now)
    end = hardware.entry_seconds + 2 * hardware.period + hardware.ramp_seconds
    assert hardware.sample(end) == pytest.approx(hardware.points[0], abs=1e-10)


@pytest.mark.parametrize("period", [0, -1, math.nan, math.inf, True])
def test_invalid_shared_period_is_rejected(monkeypatch, period):
    from lerobot_arm_control.tasks.circle import circle_trajectory

    profile = dict(circle_trajectory.TRAJECTORY, period_seconds=period)
    monkeypatch.setattr(circle_trajectory, "TRAJECTORY", profile)
    with pytest.raises(ValueError):
        Circle(simulation=True)


@pytest.mark.parametrize("change", ["identity", "speed", "nan", "closure"])
def test_invalid_deployed_plan_is_rejected(tmp_path, change):
    saved = json.loads((ROOT / "data/trajectories/circle.json").read_text())
    if change == "identity":
        saved["model_sha256"] = "wrong"
    elif change == "speed":
        saved["period_seconds"] = 0
    elif change == "nan":
        saved["qpos"][0][0] = math.nan
    else:
        saved["qpos"][-1][0] += 0.01
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(saved))
    with pytest.raises(ValueError):
        Circle(path)


def test_entry_collision_is_rejected_before_any_motor_write():
    bus = Bus()
    control = ManualControl(bus, Calibration(), {joint: [0, 4095] for joint in JOINTS})
    # Folded support pose from the onsite read-only snapshot.
    q = [0.07227, -1.63345, 1.48739, 1.24168, 0.07435, 0.308]
    for joint, angle in zip(JOINTS, q):
        bus.registers[joint]["Present_Position"] = control.raw(joint, angle)
    with pytest.raises(ValueError, match="collision"):
        validate_hardware_entry(Circle(), q)
    assert bus.writes == []


def test_entry_uses_zero_velocity_endpoints():
    points = entry_points([0] * 6, [1] * 6)
    assert points[0] == [0] * 6
    assert points[-1] == [1] * 6
    assert points[1][0] < 0.00001
    assert 1 - points[-2][0] < 0.00001


def test_one_lap_closes_with_zero_terminal_speed_and_acceleration():
    circle = Circle()
    circle.start(circle.points[0], 0, laps=1)
    end = circle.entry_seconds + circle.period + circle.ramp_seconds
    samples = [np.asarray(circle.sample(end - offset)) for offset in (0.01, 0.005, 0)]
    assert np.max(np.abs(samples[-1] - np.asarray(circle.points[0]))) < 1e-10
    assert np.max(np.abs(samples[-1] - samples[-2])) / 0.005 < 1e-5
    assert np.max(np.abs(samples[-1] - 2 * samples[-2] + samples[-3])) / 0.005**2 < 0.01
    assert circle.phase == "completed"


def test_hardware_circle_does_not_abort_or_wait_on_encoder_lag():
    bus, now = Bus(), [0.0]
    control = ManualControl(bus, Calibration(), {joint: [1000, 3000] for joint in JOINTS},
                            clock=lambda: now[0])
    circle = Circle()
    initial = list(circle.points[0])
    initial[2] += 0.2  # Exceeds the former 0.08/0.12 rad abort thresholds.
    for joint, angle in zip(JOINTS, initial):
        bus.registers[joint]["Present_Position"] = control.raw(joint, angle)
    control.enable_all()
    control.trajectory = circle
    circle.control = control
    circle.start(initial, 0, laps=1)
    now[0] = circle.entry_seconds + 4.1
    control.heartbeat()
    telemetry = {key: bus.sync_read(key) for key in ("Status", "Torque_Enable")}
    control.step(bus.sync_read("Present_Position"), telemetry)
    assert circle.phase == "running"
    assert not control.stopped
    assert control.last_fault is None
    assert circle.status(now[0])["tracking_error_stop_rad"] is None
    assert control.targets["elbow_flex"] == circle.sample(now[0])[2]
    # A long scheduler gap must not skip the reference shaper and jump to target.
    assert control.raw("elbow_flex", circle.sample(now[0])[2]) < control.sent["elbow_flex"]
    assert control.sent["elbow_flex"] < bus.registers["elbow_flex"]["Present_Position"]
    assert 1000 <= control.sent["elbow_flex"] <= 3000


@pytest.mark.parametrize("cause", ["fault", "watchdog"])
def test_hardware_circle_cancels_and_holds_on_feedback_or_connection_failure(cause):
    bus, now = Bus(), [0.0]
    control = ManualControl(bus, Calibration(), {joint: [1000, 3000] for joint in JOINTS},
                            clock=lambda: now[0])
    control.enable_all()
    circle = Circle()
    circle.control = control
    control.trajectory = circle
    circle.start([0.3] * 6, 0)
    if cause == "fault":
        bus.registers["gripper"]["Status"] = 1
        now[0] = 0.01
    elif cause == "watchdog":
        now[0] = 0.8
    else:
        # Force a planned-vs-measured error, while sent position itself is still safe.
        circle.points = [[0.3, 0.3, 0.45, 0.3, 0.3, 0.3]] * 721
        circle.entry_seconds = 0
        now[0] = 0.01
    telemetry = {key: bus.sync_read(key) for key in ("Status", "Torque_Enable")}
    control.step(bus.sync_read("Present_Position"), telemetry)
    assert not circle.active
    assert set(control.stopped) == set(JOINTS)
    assert all(bus.sync_read("Torque_Enable").values())
    if cause == "tracking":
        assert "elbow_flex circle tracking error +0.1500 rad" in control.last_fault["reason"]
    writes = list(bus.writes)
    now[0] += 0.01
    control.heartbeat()
    control.step(bus.sync_read("Present_Position"), telemetry)
    assert bus.writes == writes


@pytest.mark.parametrize("laps", [None, 1])
def test_valid_hardware_entry_and_laps_follow_without_local_angle_commands(laps):
    bus, now = Bus(), [0.0]
    control = ManualControl(bus, Calibration(), {joint: [1000, 3000] for joint in JOINTS},
                            clock=lambda: now[0])
    for joint, angle in zip(JOINTS, [0, -0.5, 0.8, -0.3, 0, 0.3]):
        bus.registers[joint]["Present_Position"] = control.raw(joint, angle)
    circle = Circle()
    control.trajectory = circle
    initial = [control.angle(j, bus.registers[j]["Present_Position"]) for j in JOINTS]
    circle.start_hardware(control, validate_hardware_entry(circle, initial), laps=laps)
    duration = circle.entry_seconds + 4 + (2 if laps is None else laps) * circle.period
    peak_average_speed = 0.0
    reference_history = []
    for _ in range(math.ceil(duration / 0.01)):
        for values in bus.registers.values():
            values["Present_Position"] = values["Goal_Position"]
        now[0] += 0.01
        control.heartbeat()
        telemetry = {key: bus.sync_read(key) for key in ("Status", "Torque_Enable")}
        control.step(bus.sync_read("Present_Position"), telemetry)
        if circle.phase != "completed":
            assert not control.stopped
        else:
            assert set(control.stopped) == set(JOINTS)
            assert bus.sync_read("Goal_Position") == bus.sync_read("Present_Position")
            continue
        # This fake bus follows corrected goals instantly. Check the nominal
        # trajectory speed separately from the bounded hardware compensation.
        speed = max(circle.entry_speed, circle.planned_peak_speed)
        for joint in JOINTS:
            bound = control.position_feedback.profile["joints"][joint]["max_correction_rad"]
            assert abs(control.command_angles[joint] - control.targets[joint]) <= bound + .00075
        reference_history.append([control.targets[joint] for joint in JOINTS])
        if len(reference_history) > 10:
            peak_average_speed = max(peak_average_speed, float(np.max(np.abs(
                np.asarray(reference_history[-1]) - np.asarray(reference_history[-11]))) / 0.1))
    assert circle.phase == ("running" if laps is None else "completed")
    assert 0.12 * 4 / circle.period < peak_average_speed <= speed + 0.0015 / 0.1
    control.stop()
    assert not circle.active
    assert all(bus.sync_read("Torque_Enable").values())
    assert control.status()['max_speed_rad_s'] is None
