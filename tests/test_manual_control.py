import pytest
from fakes import Bus, Calibration

from lerobot_arm_control.control.control_defaults import CONTROL
from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.transport.realtime import JOINTS


@pytest.fixture
def setup():
    bus = Bus()
    now = [0.0]
    control = ManualControl(bus, Calibration(), {name: [1000, 3000] for name in JOINTS},
                            clock=lambda: now[0])
    return bus, control, now


def step(bus, control, now, elapsed):
    now[0] += elapsed
    control.step({name: regs["Present_Position"] for name, regs in bus.registers.items()},
                 {key: {name: regs[key] for name, regs in bus.registers.items()}
                  for key in ("Status", "Torque_Enable", "Goal_Position")})


def test_unlocked_servos_work_without_native_gain_overrides():
    from copy import deepcopy

    bus, now = Bus(), [0.0]
    for registers in bus.registers.values():
        registers.update(Lock=0, Acceleration=40)
    originals = {joint: dict(registers) for joint, registers in bus.registers.items()}
    profile = deepcopy(CONTROL['position_feedback'])
    profile['native_stream'].pop('locked_i_coefficients')
    control = ManualControl(bus, Calibration(), {name: [1000, 3000] for name in JOINTS},
                            clock=lambda: now[0], feedback_profile=profile)
    control.enable_all()
    assert control.enabled == set(JOINTS)
    assert not control.stopped
    assert control.status()['controller_profile'] == CONTROL['position_feedback']['name']
    assert {register for register, _, _ in bus.writes} == {
        'Goal_Position', 'Torque_Enable', 'Acceleration', 'Goal_Velocity'}
    for index, joint in enumerate(JOINTS):
        control.target(joint, .33 + index * .015)
    bus.writes.clear()
    for _ in range(10):
        control.heartbeat()
        step(bus, control, now, 1 / 30)
        assert all(2200 < control.sent[joint] <= 3000 for joint in JOINTS)
    assert all(control.sent[joint] >= 2219 + index * 10
               for index, joint in enumerate(JOINTS))
    assert len(bus.writes) == 60
    control.close()
    assert {register for register, _, _ in bus.writes} == {
        'Goal_Position', 'Acceleration', 'Goal_Velocity'}
    for joint, registers in bus.registers.items():
        assert registers['Torque_Enable'] == 1
        assert registers['Goal_Position'] == registers['Present_Position']
        for register in ('P_Coefficient', 'Acceleration', 'Goal_Velocity', 'Lock'):
            assert registers[register] == originals[joint][register]


def test_enable_replaces_stale_goal_before_torque_and_only_selected_joint(setup):
    from lerobot_arm_control.control.control_defaults import POSITION_FEEDBACK

    bus, control, _ = setup
    control.enable("gripper")
    assert bus.writes == [("Goal_Position", "gripper", 2200),
                          ("Acceleration", "gripper",
                           POSITION_FEEDBACK['native_stream']['acceleration']),
                          ("Goal_Velocity", "gripper", 1000), ("Torque_Enable", "gripper", 1)]
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
    assert control.position_feedback.integral['wrist_flex'] == .04
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
    for _ in range(60):
        control.heartbeat()
        step(bus, control, now, 1 / 60)
        assert 1000 <= control.sent['gripper'] <= 3000
    assert bus.registers["gripper"]["Goal_Position"] == 1000
    control.target("gripper", 10)
    for _ in range(60):
        control.heartbeat()
        step(bus, control, now, 1 / 60)
        assert 1000 <= control.sent['gripper'] <= 3000
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


def test_latest_action_is_preserved_while_shared_controller_smooths_the_motor_command(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 1.2)
    step(bus, control, now, 1 / 60)
    assert control.targets['gripper'] == 1.2
    assert 2200 < control.sent["gripper"] < 2800
    assert .3 < control.command_angles["gripper"] < 1.2
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

def test_streaming_target_keeps_sub_tick_motion_with_bounded_pid_damping(setup):
    bus, control, now = setup
    control.enable("gripper")
    for index in range(100):
        bus.registers["gripper"]["Present_Position"] = control.sent["gripper"]
        target = 0.3 + 0.09 * (index + 1) * 0.01
        control.target("gripper", target)
        step(bus, control, now, 0.01)
        assert control.targets["gripper"] == target
        # Allow the explicitly chosen smoothing delay, but no growing queue of
        # old targets: 0.008 rad at this speed is less than 90 ms of command lag.
        assert abs(control.command_angles["gripper"] - target) <= .008
        assert abs(control.angle("gripper", control.sent["gripper"]) - target) <= .008
    assert control.command_angles["gripper"] > .382


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


def test_stream_readbacks_use_configured_retry_budget_during_enable_and_restore(setup):
    from lerobot_arm_control.control.control_defaults import READ_RETRIES

    bus, control, _ = setup
    original = bus.read
    reads = []
    registers = {'Acceleration', 'Maximum_Acceleration', 'Goal_Time', 'Goal_Velocity',
                 'Maximum_Velocity_Limit', 'Velocity_Unit_factor', 'I_Coefficient'}

    def read(register, joint, normalize=False, num_retry=0):
        if register in registers:
            reads.append((register, num_retry))
            if num_retry < READ_RETRIES:
                raise RuntimeError('There is no status packet!')
        return original(register, joint, normalize, num_retry)

    bus.read = read
    control.enable_all()
    control.close()
    assert reads and all(retries == READ_RETRIES for _, retries in reads)
    assert bus.sync_read('Goal_Position') == bus.sync_read('Present_Position')


def test_scheduled_acceleration_stays_inside_caps_and_restores_original_ram(setup):
    bus, control, _ = setup
    control.position_feedback.profile['native_stream'].update(
        acceleration=32, transition_acceleration=16)
    bus.registers['gripper']['Maximum_Acceleration'] = 10
    control.enable_all()
    for fraction, expected in [(0, 16), (.25, 20), (.5, 24), (1, 32), (.5, 24), (0, 16)]:
        control.schedule_stream_acceleration(fraction)
        assert bus.registers['shoulder_pan']['Acceleration'] == expected
        assert bus.registers['gripper']['Acceleration'] == 10
    bus.writes.clear()
    control.schedule_stream_acceleration(0)
    assert not bus.writes  # No redundant register writes at steady speed or rest.
    control.close()
    assert set(bus.sync_read('Acceleration').values()) == {0}
    assert set(bus.sync_read('Torque_Enable').values()) == {1}
    assert not control.stream_acceleration_bounds
    assert not any(r in ('Maximum_Acceleration', 'P_Coefficient', 'Lock') for r, _, _ in bus.writes)


@pytest.mark.parametrize('value', [0, 33, True, 16.5])
def test_invalid_transition_acceleration_is_rejected_before_io(value):
    from copy import deepcopy

    from lerobot_arm_control.control.control_defaults import POSITION_FEEDBACK

    bus = Bus()
    profile = deepcopy(POSITION_FEEDBACK)
    profile['native_stream'].update(acceleration=32, transition_acceleration=value)
    with pytest.raises(ValueError, match='transition acceleration'):
        ManualControl(bus, Calibration(), {j: [1000, 3000] for j in JOINTS},
                      feedback_profile=profile)
    assert not bus.writes


def test_failed_scheduled_acceleration_readback_holds_instead_of_moving(setup):
    bus, control, _ = setup
    control.position_feedback.profile['native_stream'].update(
        acceleration=32, transition_acceleration=16)
    control.enable_all()
    original = bus.sync_write

    def dropped(register, values, normalize=False):
        if register != 'Acceleration':
            original(register, values, normalize)

    bus.sync_write = dropped
    with pytest.raises(RuntimeError, match='scheduled native acceleration'):
        control.schedule_stream_acceleration(1)
    assert set(control.stopped) == set(JOINTS)
    assert bus.sync_read('Goal_Position') == bus.sync_read('Present_Position')


@pytest.mark.parametrize('coefficient,joint,original,requested', [
    ('P', 'shoulder_pan', 16, 32), ('I', 'shoulder_lift', 4, 0)])
def test_temporary_gain_restores_after_reenable_without_dropping_support(
        setup, coefficient, joint, original, requested):
    bus, control, _ = setup
    register = f'{coefficient}_Coefficient'
    key = coefficient.lower()
    bus.registers[joint][register] = original
    control.position_feedback.profile['native_stream'][f'locked_{key}_coefficients'] = {
        joint: requested}
    control.enable(joint)
    control.stop()
    control.enable(joint)
    assert control.status()[f'native_locked_{key}_coefficients'] == {joint: requested}
    assert bus.registers[joint][register] == requested
    control.close()
    assert bus.registers[joint][register] == original
    assert bus.registers[joint]['Lock'] == 1
    assert bus.registers[joint]['Torque_Enable'] == 1
    assert not any(r == 'Lock' or (r == 'Torque_Enable' and v == 0)
                   for r, _, v in bus.writes)


@pytest.mark.parametrize('coefficient,joint,original,requested', [
    ('P', 'shoulder_pan', 16, 32), ('I', 'shoulder_lift', 4, 0)])
@pytest.mark.parametrize('unlocked', [False, True])
def test_unconfirmed_or_unlocked_temporary_gain_is_not_silently_accepted(
        setup, coefficient, joint, original, requested, unlocked):
    bus, control, _ = setup
    register = f'{coefficient}_Coefficient'
    bus.registers[joint][register] = original
    control.position_feedback.profile['native_stream'][
        f'locked_{coefficient.lower()}_coefficients'] = {joint: requested}
    write = bus.write

    def drop_gain(field, name, value, normalize=False):
        if not (field == register and value == requested):
            write(field, name, value, normalize)

    if unlocked:
        bus.registers[joint]['Lock'] = 0
    else:
        bus.write = drop_gain
    with pytest.raises(RuntimeError, match=f'storage lock|temporary {coefficient}'):
        control.enable(joint)
    assert bus.registers[joint][register] == original
    if unlocked:
        assert not control.enabled
        assert not bus.writes
    else:
        assert joint in control.stopped
        control.close()
        assert bus.registers[joint][register] == original
        assert bus.registers[joint]['Torque_Enable'] == 1


def test_unlocked_native_i_in_any_joint_blocks_enable_all_before_goal_writes(setup):
    bus, control, _ = setup
    bus.registers['elbow_flex']['Lock'] = 0
    with pytest.raises(RuntimeError, match='storage lock'):
        control.enable_all()
    assert not bus.writes
    assert not control.enabled


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
            assert 2200 < control.sent[joint] < 2220
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


def test_bad_feedback_latches_hold_instead_of_continuing_target(setup):
    bus, control, now = setup
    control.enable("gripper")
    control.target("gripper", 0.4)
    bus.registers["gripper"]["Status"] = 1
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
        + ["Goal_Velocity"] * 6 + ["I_Coefficient"] * 2 + ["Torque_Enable"] * 6)
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
    assert 1200 < control.sent[joint] < 1320
    assert not control.stopped
    for _ in range(30):
        control.heartbeat()
        step(bus, control, now, 1/60)
    assert 1000 <= control.sent[joint] <= 1200
    control.target(joint, 0)
    for _ in range(30):
        control.heartbeat()
        step(bus, control, now, 1/60)
    assert control.targets[joint] == 0 and 2000 <= control.sent[joint] <= 3000


@pytest.mark.parametrize("bounds", [[1000, 1000], [-1, 3000], [0, 4096], [3000, 1000]])
def test_invalid_native_calibration_is_rejected_before_motor_writes(bounds):
    bus = Bus()
    limits = {joint: [1000, 3000] for joint in JOINTS}
    limits["gripper"] = bounds
    with pytest.raises(ValueError, match="invalid control limits"):
        ManualControl(bus, Calibration(), limits)
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
    assert 2200 < control.sent["gripper"] < 2800
    assert control.status_warning["reported_status"] == 1
    assert control.status_warning["confirmed_status"] == 0


@pytest.mark.parametrize('endpoint,actual', [(1479, 1510), (2982, 2950)])
def test_measured_jaw_endpoints_bound_pid_and_encoder_commands(setup, endpoint, actual):
    bus, control, now = setup
    control.set_command_limits('gripper', tuple(control.angle('gripper', raw)
                                              for raw in (1479, 2982)))
    assert control.rest_limits['gripper'] == (-1.5, 1.5)
    control.enable('gripper')
    control.target('gripper', control.angle('gripper', endpoint))
    bus.registers['gripper']['Present_Position'] = actual  # Persistent endpoint tracking error.
    for _ in range(300):
        control.heartbeat()
        step(bus, control, now, 1/60)
        assert 1479 <= control.sent['gripper'] <= 2982
    assert control.sent['gripper'] == endpoint
    control.stop()
    assert control.sent['gripper'] == actual


def test_narrowed_command_range_does_not_move_an_outside_held_pose(setup):
    bus, control, now = setup
    control.set_command_limits('gripper', (-.15, .6))
    bus.registers['gripper'].update(Present_Position=1800, Torque_Enable=1)
    step(bus, control, now, 1/60)
    assert bus.registers['gripper']['Goal_Position'] == 1800
    assert control.stopped['gripper'].startswith('pre-existing')
    control.close()
    assert bus.registers['gripper']['Goal_Position'] == 1800


def overload_gripper(bus):
    bus.registers['gripper']['Status'] = 32
    original = bus.read

    def fault_read(register, joint, **kwargs):
        if joint == 'gripper':
            raise RuntimeError('[RxPacketError] Overload error!')
        return original(register, joint, **kwargs)

    bus.read = fault_read


@pytest.mark.parametrize('already_enabled', [False, True])
def test_sdk_overload_latches_motion_and_keeps_monitoring_and_cleanup_alive(setup, already_enabled):
    bus, control, now = setup
    if already_enabled:
        control.enable_all()
    else:
        for registers in bus.registers.values():
            registers['Torque_Enable'] = 1
    overload_gripper(bus)
    bus.writes.clear()
    step(bus, control, now, 1/60)
    assert control.faulted_joints == {'gripper'}
    assert set(control.stopped) == set(JOINTS)
    assert 'Overload error' in control.last_fault['packet_error']
    assert 'Status=32' in control.last_fault['reason']
    assert [(register, raw) for register, joint, raw in bus.writes if joint == 'gripper'] == [
        ('Goal_Position', 2200)]
    assert control.last_fault['overload_hold_raw'] == {'gripper': 2200}
    writes = list(bus.writes)
    for _ in range(3):
        step(bus, control, now, 1/60)
    assert bus.writes == writes
    # A clear status alone cannot resume; explicit enable is required.
    bus.registers['gripper']['Status'] = 0
    step(bus, control, now, 1/60)
    assert bus.writes == writes
    control.close()  # Faulted devices are not written during stream restoration either.
    assert [(register, raw) for register, joint, raw in bus.writes if joint == 'gripper'] == [
        ('Goal_Position', 2200)]


def test_overload_rearm_preflight_refuses_before_writing(setup):
    bus, control, _ = setup
    overload_gripper(bus)
    with pytest.raises(ValueError, match='overload.*Status=32'):
        control.enable_all()
    assert bus.writes == []
    assert control.faulted_joints == {'gripper'}


@pytest.mark.parametrize('status,torque,age', [(32, 0, 0), (32, 1, .1),
                                            (8, 1, 0), (40, 1, 0)])
def test_fault_hold_cannot_enable_torque_or_use_stale_or_sensor_fault_feedback(
        setup, status, torque, age):
    bus, control, now = setup
    control.enable_all()
    control.faulted_joints.add('gripper')
    control.last_fault = {}
    control.last_tick = now[0]-age
    bus.registers['gripper'].update(Status=status, Torque_Enable=torque)
    telemetry = {key: {joint: values[key] for joint, values in bus.registers.items()}
                 for key in ('Status', 'Torque_Enable', 'Goal_Position')}
    bus.writes.clear()
    control.stop('fault', fault_telemetry=telemetry)
    assert not any(joint == 'gripper' for _, joint, _ in bus.writes)
    assert bus.registers['gripper']['Torque_Enable'] == torque


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


@pytest.mark.parametrize('all_joints', [False, True])
def test_startup_readbacks_retry_without_replaying_goal_writes(setup, all_joints):
    from lerobot_arm_control.control.control_defaults import READ_RETRIES

    bus, control, _ = setup
    original = bus.read
    checked = set()
    startup = {'Lock', 'Status', 'Operating_Mode', 'Present_Voltage',
               'Min_Voltage_Limit', 'Max_Voltage_Limit', 'Present_Position',
               'Goal_Position', 'Torque_Enable'}

    def read(register, joint, normalize=False, num_retry=0):
        if register in startup:
            checked.add(register)
            if num_retry < READ_RETRIES:
                raise RuntimeError('[RxPacketError] Input voltage error!')
        return original(register, joint, normalize, num_retry)

    bus.read = read
    control.enable_all() if all_joints else control.enable('shoulder_lift')
    expected = set(JOINTS) if all_joints else {'shoulder_lift'}
    assert control.enabled == expected
    assert checked >= startup - ({'Goal_Position', 'Torque_Enable'} if all_joints else set())
    goals = [(joint, value) for register, joint, value in bus.writes
             if register == 'Goal_Position']
    assert sorted(goals) == sorted((joint, 2200) for joint in expected)


def test_persistent_startup_position_read_failure_blocks_all_goal_writes(setup):
    from lerobot_arm_control.control.control_defaults import READ_RETRIES

    bus, control, _ = setup
    original = bus.read

    def read(register, joint, normalize=False, num_retry=0):
        if register == 'Present_Position' and joint == 'gripper':
            assert num_retry == READ_RETRIES
            raise RuntimeError('[RxPacketError] Input voltage error!')
        return original(register, joint, normalize, num_retry)

    bus.read = read
    with pytest.raises(RuntimeError, match='Input voltage'):
        control.enable_all()
    assert bus.writes == []
    assert not control.enabled
