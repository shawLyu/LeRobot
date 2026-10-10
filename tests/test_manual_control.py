import pytest

from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.transport.realtime import JOINTS


class Calibration:
    def count(self):
        return 6

    def fit(self, joint):
        return {"zero_raw": 2000, "radians_per_tick": 0.0015}


class Bus:
    def __init__(self):
        self.registers = {name: {"Present_Position": 2200, "Goal_Position": 1000,
                                 "Torque_Enable": 0, "Status": 0, "Operating_Mode": 0,
                                 "Present_Voltage": 52, "Min_Voltage_Limit": 40,
                                 "Max_Voltage_Limit": 90, "Acceleration": 0,
                                 "Maximum_Acceleration": 254, "Goal_Time": 0,
                                 "Goal_Velocity": 0, "Maximum_Velocity_Limit": 1000,
                                 "Velocity_Unit_factor": 1, "Lock": 1, "P_Coefficient": 16}
                          for name in JOINTS}
        self.writes = []

    def read(self, register, joint, normalize=False, num_retry=0):
        return self.registers[joint][register]

    def sync_read(self, register, normalize=False):
        return {joint: values[register] for joint, values in self.registers.items()}

    def sync_write(self, register, values, normalize=False):
        for joint, value in values.items():
            self.write(register, joint, value, normalize)

    def write(self, register, joint, value, normalize=False):
        self.writes.append((register, joint, value))
        self.registers[joint][register] = value
        if register == "Goal_Position":
            self.registers[joint]["Torque_Enable"] = 1


@pytest.fixture
def setup():
    bus = Bus()
    now = [0.0]
    control = ManualControl(bus, Calibration(), [[-1, 1]] * 6,
                            {name: [1000, 3000] for name in JOINTS}, clock=lambda: now[0])
    return bus, control, now


def step(bus, control, now, elapsed):
    now[0] += elapsed
    control.step({name: regs["Present_Position"] for name, regs in bus.registers.items()},
                 {key: {name: regs[key] for name, regs in bus.registers.items()}
                  for key in ("Status", "Torque_Enable", "Goal_Position")})


def test_enable_replaces_stale_goal_before_torque_and_only_selected_joint(setup):
    bus, control, _ = setup
    control.enable("gripper")
    assert bus.writes == [("Goal_Position", "gripper", 2200),
                          ("Acceleration", "gripper", 40),
                          ("Goal_Velocity", "gripper", 600), ("Torque_Enable", "gripper", 1)]
    assert control.status()["enabled"] == ["gripper"]
    assert control.status()["error_rad"]["gripper"] == 0


def test_target_requires_enable_and_finite_angles_but_has_no_relative_cap(setup):
    _, control, _ = setup
    with pytest.raises(ValueError, match="enable"):
        control.target("gripper", 0.33)
    control.enable("gripper")
    for target in (float("nan"), True, "0.3"):
        with pytest.raises(ValueError):
            control.target("gripper", target)
    control.target("gripper", 1.2)
    assert control.targets["gripper"] == 1.2
    control.target("gripper", 10)
    assert control.targets["gripper"] == 1.5


def test_settle_clears_only_selected_compensation_without_changing_goals(setup):
    bus, control, _ = setup
    control.enable('wrist_flex')
    control.enable('elbow_flex')
    control.target('wrist_flex', .5)
    control.position_feedback.integral['wrist_flex'] = .04
    control.position_feedback.correction['wrist_flex'] = .03
    control.position_feedback.integral['elbow_flex'] = .02
    before = list(bus.writes)
    control.settle_target('wrist_flex')
    assert control.targets['wrist_flex'] == .5
    assert control.position_feedback.integral['wrist_flex'] == 0
    assert control.position_feedback.correction['wrist_flex'] == .03
    assert control.position_feedback.integral['elbow_flex'] == .02
    assert bus.writes == before
    assert bus.registers['wrist_flex']['Torque_Enable'] == 1
    with pytest.raises(ValueError, match='enabled'):
        control.settle_target('gripper')
    control.stop()
    with pytest.raises(ValueError, match='enabled'):
        control.settle_target('wrist_flex')

def test_hardware_uses_exact_calibrated_raw_endpoints_without_model_margins(setup):
    bus, control, now = setup
    assert control.limits["gripper"] == (-1.5, 1.5)
    control.enable("gripper")
    control.target("gripper", -10)
    step(bus, control, now, 1 / 60)
    assert bus.registers["gripper"]["Goal_Position"] == 1000
    control.target("gripper", 10)
    step(bus, control, now, 1 / 60)
    assert bus.registers["gripper"]["Goal_Position"] == 3000

def test_inherited_torque_can_be_released_without_enabling_again(setup):
    bus, control, now = setup
    bus.registers["shoulder_lift"]["Torque_Enable"] = 1
    step(bus, control, now, 0.05)
    assert control.enabled == {"shoulder_lift"}
    assert control.stopped["shoulder_lift"].startswith("pre-existing")
    control.release("shoulder_lift")
    assert bus.registers["shoulder_lift"]["Torque_Enable"] == 0
    assert bus.registers["gripper"]["Torque_Enable"] == 0


def test_voltage_preflight_rejects_without_motor_writes(setup):
    bus, control, _ = setup
    bus.registers["gripper"]["Present_Voltage"] = 39
    with pytest.raises(ValueError, match="voltage"):
        control.enable("gripper")
    assert bus.writes == []


def test_official_action_is_sent_without_a_software_slew_limiter(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 1.2)
    step(bus, control, now, 1 / 60)
    assert control.sent["gripper"] == 2800
    assert control.command_angles["gripper"] == 1.2
    assert control.status()["max_relative_target"] is None
    assert control.status()["max_speed_rad_s"] is None

def test_persistent_encoder_lag_does_not_stop_official_position_stream(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 1.2)
    for _ in range(60):
        control.heartbeat()
        step(bus, control, now, 1 / 60)
    assert not control.stopped
    assert control.last_fault is None
    assert control.targets["gripper"] == 1.2
    bound = control.position_feedback.profile["joints"]["gripper"]["max_correction_rad"]
    assert 2800 < control.sent["gripper"] <= control.raw("gripper", 1.2 + bound)
    assert control.status()["update_rate_hz"] == pytest.approx(60)


def test_feedback_preserves_reference_and_release_order(setup):
    from lerobot_arm_control.control.position_feedback import PositionFeedback

    bus, control, now = setup
    profile = {"version": 1, "native_stream": {"acceleration": 20, "velocity_ticks_s": 600},
               "joints": {
        j: {"kp": 0.5, "ki": 0.8, "max_correction_rad": 0.12,
            "correction_rate_rad_s": 0.08} for j in JOINTS}}
    control.position_feedback = PositionFeedback(profile, control.limits)
    control.enable("elbow_flex")
    control.target("elbow_flex", 0.4)
    for _ in range(60):
        control.heartbeat()
        step(bus, control, now, 1 / 60)
    assert control.targets["elbow_flex"] == 0.4
    assert control.command_angles["elbow_flex"] > 0.4
    assert not control.stopped
    control.release("elbow_flex")
    assert control.position_feedback.correction["elbow_flex"] == 0
    writes = len(bus.writes)
    step(bus, control, now, 1 / 60)
    assert len(bus.writes) == writes
    assert bus.registers["elbow_flex"]["Torque_Enable"] == 0


def test_stop_clears_feedback_and_holds_measured_pose(setup):
    from lerobot_arm_control.control.position_feedback import PositionFeedback

    bus, control, now = setup
    profile = {"version": 1, "native_stream": {"acceleration": 20, "velocity_ticks_s": 600},
               "joints": {
        j: {"kp": 1.0, "ki": 1.5, "max_correction_rad": 0.12,
            "correction_rate_rad_s": 0.12} for j in JOINTS}}
    control.position_feedback = PositionFeedback(profile, control.limits)
    control.enable("elbow_flex")
    control.target("elbow_flex", 0.4)
    step(bus, control, now, 1 / 60)
    assert control.position_feedback.correction["elbow_flex"] > 0
    control.stop()
    assert control.position_feedback.correction["elbow_flex"] == 0
    assert bus.registers["elbow_flex"]["Goal_Position"] == 2200
    assert bus.registers["elbow_flex"]["Torque_Enable"] == 1

def test_streaming_target_below_speed_limit_is_not_biased_by_tick_budgets(setup):
    bus, control, now = setup
    control.enable("gripper")
    for index in range(100):
        bus.registers["gripper"]["Present_Position"] = control.sent["gripper"]
        target = 0.3 + 0.09 * (index + 1) * 0.01
        control.target("gripper", target)
        step(bus, control, now, 0.01)
        assert abs(control.command_angles["gripper"] - target) <= 0.00075 + 1e-12
        assert abs(control.angle("gripper", control.sent["gripper"]) - target) <= 0.00075 + 1e-12


def test_stationary_running_goal_is_refreshed_after_a_lost_packet(setup):
    bus, control, now = setup
    control.enable("gripper")
    bus.registers["gripper"]["Goal_Position"] = 1000  # Simulated lost/corrupted prior write.
    step(bus, control, now, 0.01)
    assert bus.registers["gripper"]["Goal_Position"] == 2200


def test_official_action_refreshes_delivered_running_goals_each_frame(setup):
    bus, control, now = setup
    control.enable("gripper")
    bus.writes.clear()
    for _ in range(10):
        step(bus, control, now, 1 / 60)
    assert bus.writes == [("Goal_Position", "gripper", 2200)] * 10

def test_native_stream_configuration_respects_eeprom_cap_and_restores_after_hold(setup):
    bus, control, _ = setup
    bus.registers["gripper"]["Maximum_Acceleration"] = 10
    control.enable("gripper")
    assert bus.registers["gripper"]["Acceleration"] == 10
    assert control.status()["native_acceleration"] == {"gripper": 10}
    control.close()
    assert bus.registers["gripper"]["Acceleration"] == 0
    assert bus.registers["gripper"]["Torque_Enable"] == 1
    assert bus.registers["gripper"]["Goal_Position"] == bus.registers["gripper"]["Present_Position"]
    assert not any(register in ("Maximum_Acceleration", "P_Coefficient", "Operating_Mode")
                   for register, _, _ in bus.writes)


def test_native_acceleration_readback_failure_cannot_start_motion(setup):
    bus, control, _ = setup
    original = bus.sync_write

    def dropped_acceleration(register, values, normalize=False):
        if register != "Acceleration":
            original(register, values, normalize)

    bus.sync_write = dropped_acceleration
    with pytest.raises(RuntimeError, match="acceleration could not be confirmed"):
        control.enable_all()
    assert set(control.stopped) == set(JOINTS)
    assert bus.sync_read("Goal_Position") == bus.sync_read("Present_Position")
    assert not any(register == "Torque_Enable" for register, _, _ in bus.writes)


def test_temporary_p_keeps_lock_and_restores_while_holding(setup):
    bus, control, _ = setup
    control.position_feedback.profile["native_stream"]["locked_p_coefficients"] = {
        "shoulder_pan": 32}
    control.enable('shoulder_pan')
    assert bus.registers['shoulder_pan']['P_Coefficient'] == 32
    assert control.status()['native_locked_p_coefficients'] == {'shoulder_pan': 32}
    control.close()
    assert bus.registers['shoulder_pan']['P_Coefficient'] == 16
    assert bus.registers['shoulder_pan']['Lock'] == 1
    assert bus.registers['shoulder_pan']['Torque_Enable'] == 1
    assert not any(register == 'Lock' for register, _, _ in bus.writes)


def test_unlocked_storage_cannot_receive_temporary_p(setup):
    bus, control, _ = setup
    control.position_feedback.profile["native_stream"]["locked_p_coefficients"] = {
        "shoulder_pan": 32}
    bus.registers['shoulder_pan']['Lock'] = 0
    with pytest.raises(RuntimeError, match='storage lock'):
        control.enable('shoulder_pan')
    assert 'shoulder_pan' in control.stopped
    assert not any(register == 'P_Coefficient' for register, _, _ in bus.writes)


def test_dropped_temporary_p_is_held_and_restored_on_close(setup):
    bus, control, _ = setup
    control.position_feedback.profile["native_stream"]["locked_p_coefficients"] = {
        "shoulder_pan": 32}
    original = bus.write

    def drop_trial(register, joint, value, normalize=False):
        if not (register == 'P_Coefficient' and value == 32):
            original(register, joint, value, normalize)

    bus.write = drop_trial
    with pytest.raises(RuntimeError, match='temporary P could not be confirmed'):
        control.enable('shoulder_pan')
    control.close()
    assert bus.registers['shoulder_pan']['P_Coefficient'] == 16
    assert bus.registers['shoulder_pan']['Torque_Enable'] == 1


def test_reenable_keeps_original_native_setting_for_rollback(setup):
    bus, control, _ = setup
    control.enable("gripper")
    control.stop()
    control.enable("gripper")
    control.close()
    assert bus.registers["gripper"]["Acceleration"] == 0


def test_changed_joint_targets_share_one_sync_packet(setup):
    bus, control, now = setup
    control.enable_all()
    control.target("gripper", 0.33)
    control.target("wrist_roll", 0.33)
    packets = []
    original_sync = bus.sync_write

    def record_sync(register, values, normalize=False):
        packets.append((register, dict(values)))
        original_sync(register, values, normalize)

    bus.sync_write = record_sync
    control.heartbeat()
    step(bus, control, now, 0.05)
    assert packets == [("Goal_Position", control.sent)]
    assert set(packets[0][1]) == set(JOINTS)
    assert control.targets["gripper"] == control.targets["wrist_roll"] == .33
    for joint in JOINTS:
        if joint in ("gripper", "wrist_roll"):
            assert control.sent[joint] > 2220
        else:
            assert control.sent[joint] == 2200


def test_watchdog_holds_actual_position_and_latches_until_enable(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 0.4)
    bus.registers["gripper"]["Present_Position"] = 2203
    step(bus, control, now, 0.8)
    assert control.sent["gripper"] == 2203
    assert bus.registers["gripper"]["Torque_Enable"] == 1
    assert "timeout" in control.stopped["gripper"]
    with pytest.raises(ValueError, match="enable"):
        control.target("gripper", 0.35)
    control.enable("gripper")
    control.target("gripper", 0.35)


def test_lost_torque_never_reactivates_joint_via_goal_write(setup):
    bus, control, now = setup
    control.enable("gripper")
    bus.registers["gripper"]["Torque_Enable"] = 0
    bus.writes.clear()
    step(bus, control, now, 0.05)
    assert bus.writes == []
    assert "gripper" not in control.enabled
    assert "lost torque" in control.stopped["gripper"]


@pytest.mark.parametrize("cause", ["fault"])
def test_bad_feedback_latches_hold_instead_of_continuing_target(setup, cause):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 0.4)
    if cause == "fault":
        bus.registers["gripper"]["Status"] = 1
    elif cause == "tracking":
        bus.registers["gripper"]["Present_Position"] = 2300
    else:
        bus.registers["gripper"]["Present_Position"] = 2800
    step(bus, control, now, 0.05)
    writes = list(bus.writes)
    assert "gripper" in control.stopped
    assert control.last_fault is not None
    snapshot = control.last_fault
    step(bus, control, now, 0.05)
    assert bus.writes == writes
    control.release("gripper")
    assert control.last_fault == snapshot


def test_release_torque_off_is_last_and_future_ticks_do_not_reactivate(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.release("gripper")
    assert bus.writes[-1] == ("Torque_Enable", "gripper", 0)
    writes = list(bus.writes)
    step(bus, control, now, 0.05)
    assert bus.writes == writes
    assert bus.registers["gripper"]["Torque_Enable"] == 0


def test_close_holds_supported_joint_without_releasing_torque(setup):
    bus, control, _ = setup
    control.enable("gripper")
    control.target("gripper", 0.4)
    control.close()
    assert bus.registers["gripper"]["Goal_Position"] == 2200
    assert bus.registers["gripper"]["Torque_Enable"] == 1


def test_hold_io_failure_still_latches_all_joints(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.enable("wrist_roll")
    original_read = bus.read

    def failing_read(register, joint, normalize=False):
        if joint == "gripper":
            raise TimeoutError("serial timeout")
        return original_read(register, joint, normalize)

    bus.read = failing_read
    with pytest.raises(RuntimeError, match="could not be confirmed"):
        control.stop()
    assert set(control.stopped) == {"gripper", "wrist_roll"}
    writes = list(bus.writes)
    step(bus, control, now, 0.05)
    assert bus.writes == writes


def test_enable_all_checks_every_motor_before_writing_and_uses_fresh_goals(setup):
    bus, control, _ = setup
    bus.registers["gripper"]["Status"] = 1
    with pytest.raises(ValueError, match="Status=1"):
        control.enable_all()
    assert bus.writes == []
    bus.registers["gripper"]["Status"] = 0
    control.enable_all()
    assert [register for register, _, _ in bus.writes] == (
        ["Goal_Position"] * 6 + ["Acceleration"] * 6
        + ["Goal_Velocity"] * 6 + ['P_Coefficient'] * 5 + ["Torque_Enable"] * 6)
    assert all(bus.sync_read("Torque_Enable").values())
    assert bus.sync_read("Goal_Position") == bus.sync_read("Present_Position")


def test_missing_sync_goal_never_explicitly_enables_motor_with_stale_goal(setup):
    bus, control, _ = setup
    original_sync = bus.sync_write

    def dropped_goal(register, values, normalize=False):
        if register == "Goal_Position":
            values = {joint: raw for joint, raw in values.items() if joint != "gripper"}
        original_sync(register, values, normalize)

    bus.sync_write = dropped_goal
    with pytest.raises(RuntimeError, match="fresh goals"):
        control.enable_all()
    assert bus.registers["gripper"]["Torque_Enable"] == 0
    assert not any(register == "Torque_Enable" for register, _, _ in bus.writes)


def test_model_edge_does_not_narrow_real_calibration_or_stop_control(setup):
    bus, control, now = setup
    joint = "shoulder_lift"
    bus.registers[joint]["Present_Position"] = 1320
    control.enable_all()
    control.target(joint, -1.2)
    step(bus, control, now, 1 / 60)
    assert control.targets[joint] == -1.2
    assert 1000 <= control.sent[joint] <= 1200
    assert not control.stopped
    control.target(joint, 0)
    step(bus, control, now, 1 / 60)
    assert control.sent[joint] == 2000


@pytest.mark.parametrize("bounds", [[1000, 1000], [-1, 3000], [0, 4096], [3000, 1000]])
def test_invalid_native_calibration_is_rejected_before_motor_writes(bounds):
    bus = Bus()
    limits = {joint: [1000, 3000] for joint in JOINTS}
    limits["gripper"] = bounds
    with pytest.raises(ValueError, match="invalid control limits"):
        ManualControl(bus, Calibration(), [[-1, 1]] * 6, limits)
    assert bus.writes == []


def test_transient_status_that_clears_on_independent_read_is_logged_without_hold(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 1.2)
    now[0] = 1 / 60
    telemetry = {key: bus.sync_read(key) for key in ("Status", "Torque_Enable")}
    telemetry["Status"]["gripper"] = 1  # A transient flag, independently now zero.
    control.step(bus.sync_read("Present_Position"), telemetry)
    assert not control.stopped
    assert control.last_fault is None
    assert control.sent["gripper"] == 2800
    assert control.status_warning["reported_status"] == 1
    assert control.status_warning["confirmed_status"] == 0


def test_native_stream_is_ram_only_verified_and_restored_before_release(setup):
    from lerobot_arm_control.control.position_feedback import PositionFeedback

    bus, control, _ = setup
    profile = {"version": 1, "native_stream": {"acceleration": 20, "velocity_ticks_s": 600},
               "joints": {j: {"kp": .8, "ki": 1.2, "max_correction_rad": .12,
                               "correction_rate_rad_s": .12} for j in JOINTS}}
    control.position_feedback = PositionFeedback(profile, control.limits)
    for regs in bus.registers.values():
        regs.update(Goal_Velocity=0, Goal_Time=0, Maximum_Velocity_Limit=68,
                    Velocity_Unit_factor=50, Acceleration=254)
    control.enable_all()
    assert all(r["Goal_Velocity"] == 600 and r["Acceleration"] == 20
               for r in bus.registers.values())
    control.release("gripper")
    assert bus.registers["gripper"]["Goal_Velocity"] == 0
    assert bus.registers["gripper"]["Acceleration"] == 254
    assert bus.writes[-1] == ("Torque_Enable", "gripper", 0)
    control.close()
    assert all(r["Goal_Velocity"] == 0 and r["Acceleration"] == 254
               for r in bus.registers.values())
    assert all(bus.registers[j]["Torque_Enable"] == 1 for j in JOINTS if j != "gripper")
    assert not any(reg in ("P_Coefficient", "I_Coefficient", "D_Coefficient", "Lock")
                   for reg, _, _ in bus.writes)
