"""Six-channel contracts and adversarial configuration/motion paths, without hardware."""

import json
from types import SimpleNamespace

import pytest

from lerobot_arm_control.robot.so101.joints import JOINTS
from lerobot_arm_control.workbench.controller import Workbench
from lerobot_arm_control.workbench.registers import BY_KEY, REGISTERS, install_table


class Clock:
    def __init__(self):
        self.now = 10.

    def __call__(self):
        return self.now

    def advance(self, seconds=.01):
        self.now += seconds


class Bus:
    def __init__(self, count=6):
        self.motors = {j: SimpleNamespace(id=i+1, model='sts3215')
                       for i, j in enumerate(JOINTS[:count])}
        self.registers, self.writes = {}, []
        self.model_ctrl_table = {'sts3215': {'Minimum_Startup_Force': (24, 2)}}
        install_table(self)
        for joint, motor in self.motors.items():
            values = {r.key: 0 for r in REGISTERS}
            values.update(ID=motor.id, Model_Number=777, Present_Position=2067,
                Goal_Position=2067, Observed_Goal_Position=2067, P_Coefficient=64,
                I_Coefficient=4, D_Coefficient=128, Minimum_Startup_Force=16,
                Integral_Limit=8, Lock=1, Min_Position_Limit=844, Max_Position_Limit=3440,
                Min_Voltage_Limit=40, Max_Voltage_Limit=90, Max_Temperature_Limit=70,
                Present_Voltage=53, Present_Temperature=25, Angular_Resolution=1,
                Response_Status_Level=1,
                Homing_Offset=-1110, Phase=12, Torque_Limit=1000, Max_Torque_Limit=1000)
            self.registers[joint] = values
        self.follow = True

    def read(self, key, joint, **kwargs):
        return self.registers[joint][key]

    def write(self, key, joint, value, **kwargs):
        self.writes.append((key, joint, value))
        self.registers[joint][key] = value
        if key == 'Goal_Position':
            self.registers[joint]['Torque_Enable'] = 1

    def sync_write(self, key, values, **kwargs):
        for joint, value in values.items():
            self.write(key, joint, value)

    def set_id(self, joint, motor_id):
        self.motors[joint].id = motor_id

    def id_available(self, motor_id):
        return all(m.id != motor_id for m in self.motors.values())

    def set_baudrate(self, value):
        self.baudrate = value

    def feedback(self, **kwargs):
        from lerobot_arm_control.control.servo_feedback import FIELDS

        for values in self.registers.values():
            if self.follow and values['Torque_Enable'] == 1:
                error = values['Goal_Position'] - values['Present_Position']
                values['Present_Position'] += round(.6 * error) if abs(error) > 1 else error
        return {key: {joint: v[key] for joint, v in self.registers.items()} for key in FIELDS}


@pytest.fixture
def rig(tmp_path, monkeypatch):
    clock, bus = Clock(), Bus()
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.time.sleep', lambda _: None)
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.read_control_feedback',
                        lambda bus, **kwargs: bus.feedback(**kwargs))
    discovery = {j: {'id': i+1, 'online': True, 'model': 777} for i, j in enumerate(JOINTS)}
    return Workbench(bus, discovery, log_root=tmp_path, clock=clock), bus, clock


def tick(rig, count, *, heartbeat=True):
    control, _, clock = rig
    for _ in range(count):
        clock.advance()
        if heartbeat:
            control.command('heartbeat', {})
        control.tick()


def test_six_motor_move_seeds_live_pose_respects_speed_and_preserves_native_settings(rig):
    control, bus, _ = rig
    control.enable(list(JOINTS))
    assert [v for k, _, v in bus.writes if k == 'Goal_Position'] == [2067] * 6
    control.move({j: 2107 for j in JOINTS}, 30)
    tick(rig, 80)
    assert all(m['position'] == 2107 for m in control.frame['motors'].values())
    assert control.move_job is None
    assert all(v['P_Coefficient'] == 64 for v in bus.registers.values())
    control.release(list(JOINTS))
    assert all(v['Torque_Enable'] == 0 for v in bus.registers.values())


def test_five_missing_motors_remain_visible_without_being_read_or_written(tmp_path, monkeypatch):
    clock, bus = Clock(), Bus(1)
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.read_control_feedback',
                        lambda bus, **kwargs: bus.feedback())
    discovery = {j: {'id': i+1, 'online': i == 0} for i, j in enumerate(JOINTS)}
    control = Workbench(bus, discovery, log_root=tmp_path, clock=clock)
    state = control.tick()
    assert len(state['discovery']) == 6 and len(state['frame']['motors']) == 1
    assert not bus.writes
    with pytest.raises(ValueError, match='在线'):
        control.enable([JOINTS[1]])


@pytest.mark.parametrize('persist', [False, True])
def test_configuration_readback_and_storage_lock_follow_the_selected_persistence(rig, persist):
    control, bus, _ = rig
    response = control.write_settings(JOINTS[0], {'P_Coefficient': 63, 'Integral_Limit': 7},
                                      {'P_Coefficient': 64, 'Integral_Limit': 8}, persist)
    assert response['after'] == {'P_Coefficient': 63, 'Integral_Limit': 7}
    assert response['completed'] and response['lock_verified']
    assert (('Lock', JOINTS[0], 0) in bus.writes) == persist
    assert bus.registers[JOINTS[0]]['Lock'] == 1
    assert bus.registers[JOINTS[0]]['Minimum_Startup_Force'] == 16
    assert not any(k in ('Torque_Enable', 'Goal_Position') for k, _, _ in bus.writes)


@pytest.mark.parametrize('changes,expected,persist', [
    ({'P_Coefficient': 255}, {'P_Coefficient': 64}, False),
    ({'P_Coefficient': float('nan')}, {'P_Coefficient': 64}, False),
    ({'P_Coefficient': True}, {'P_Coefficient': 64}, False),
    ({'Lock': 0}, {'Lock': 1}, False),
    ({'Present_Position': 100}, {'Present_Position': 2067}, False),
    ({'Return_Delay_Time': 0}, {'Return_Delay_Time': 0}, False),
    ({'P_Coefficient': 63}, {'P_Coefficient': 16}, False),
    ({'P_Coefficient': 63}, {}, False),
    ({'Acceleration': 10}, {'Acceleration': 0}, True),
    ({'Min_Position_Limit': 3500}, {'Min_Position_Limit': 844}, False),
    ({'Min_Voltage_Limit': 95}, {'Min_Voltage_Limit': 40}, False),
])
def test_invalid_stale_readonly_or_mixed_storage_writes_never_touch_the_motor(
        rig, changes, expected, persist):
    control, bus, _ = rig
    with pytest.raises(ValueError):
        control.write_settings(JOINTS[0], changes, expected, persist)
    assert bus.writes == []


def test_parameter_edit_is_rejected_if_any_online_motor_has_torque(rig):
    control, bus, _ = rig
    bus.registers[JOINTS[5]]['Torque_Enable'] = 1
    with pytest.raises(ValueError, match='扭矩'):
        control.write_settings(JOINTS[0], {'P_Coefficient': 63}, {'P_Coefficient': 64})
    assert not bus.writes


def test_partial_eeprom_failure_rolls_back_and_relocks_with_auditable_evidence(rig):
    control, bus, _ = rig
    write = bus.write

    def drop_new_integral(key, joint, value, **kwargs):
        if not (key == 'I_Coefficient' and value == 5):
            write(key, joint, value, **kwargs)

    bus.write = drop_new_integral
    with pytest.raises(RuntimeError, match='读回'):
        control.write_settings(JOINTS[0], {'P_Coefficient': 63, 'I_Coefficient': 5},
                               {'P_Coefficient': 64, 'I_Coefficient': 4}, True)
    assert bus.read('P_Coefficient', JOINTS[0]) == 64
    assert bus.read('I_Coefficient', JOINTS[0]) == 4
    assert bus.read('Lock', JOINTS[0]) == 1
    record = json.loads(next(control.log_root.glob('parameters-*.json')).read_text())
    assert not record['completed'] and record['lock_verified'] and not record['rollback_errors']


def test_lock_failure_cannot_be_reported_as_a_successful_transaction(rig):
    control, bus, _ = rig
    write = bus.write
    bus.write = lambda key, joint, value, **kw: (
        None if key == 'Lock' and value == 1 else write(key, joint, value, **kw))
    with pytest.raises(RuntimeError):
        control.write_settings(JOINTS[0], {'P_Coefficient': 63}, {'P_Coefficient': 64}, True)
    assert control.fault
    record = json.loads(next(control.log_root.glob('parameters-*.json')).read_text())
    assert not record['completed'] and not record['lock_verified']


def test_acknowledgement_loss_is_accepted_only_when_independent_readback_matches(rig):
    control, bus, _ = rig
    write = bus.write

    def no_ack(*args, **kwargs):
        write(*args, **kwargs)
        raise ConnectionError('write acknowledgement suppressed')

    bus.write = no_ack
    response = control.write_settings(JOINTS[0], {'Response_Status_Level': 0},
                                      {'Response_Status_Level': 1})
    assert response['completed'] and response['lock_verified']


def test_reference_edits_invalidate_motion_envelope_until_explicitly_reconfigured(rig):
    control, bus, _ = rig
    control.write_settings(JOINTS[0], {'Homing_Offset': -1109}, {'Homing_Offset': -1110})
    assert control.bounds[JOINTS[0]] is None
    with pytest.raises(ValueError, match='动作范围'):
        control.enable([JOINTS[0]])
    control.configure_bounds(JOINTS[0], [2000, 2200])
    control.enable([JOINTS[0]])
    assert bus.read('Torque_Enable', JOINTS[0]) == 1


def test_direct_goal_parameter_cannot_bypass_enable_or_motion_envelope(rig):
    control, bus, _ = rig
    with pytest.raises(ValueError):
        control.write_settings(JOINTS[0], {'Goal_Position': 2200}, {'Goal_Position': 2067})
    assert not bus.writes
    control.enable([JOINTS[0]])
    count = len(bus.writes)
    with pytest.raises(ValueError):
        control.move({JOINTS[0]: 4095}, 30)
    assert len(bus.writes) == count


def test_voltage_only_warning_does_not_stop_position_control(rig):
    control, bus, _ = rig
    bus.registers[JOINTS[0]]['Status'] = 1
    control.enable([JOINTS[0]])
    control.move({JOINTS[0]: 2107}, 60)
    tick(rig, 40)
    assert not control.fault
    assert control.frame['motors'][JOINTS[0]]['position'] == 2107


def test_alarm_hold_never_sends_a_new_goal_to_the_faulted_motor(rig):
    control, bus, _ = rig
    control.enable(list(JOINTS))
    bus.registers[JOINTS[0]]['Status'] = 32
    bus.writes.clear()
    tick(rig, 1)
    assert control.fault and not control.enabled
    assert not any(key == 'Goal_Position' and joint == JOINTS[0]
                   for key, joint, _ in bus.writes)
    assert any(key == 'Goal_Position' and joint == JOINTS[1]
               for key, joint, _ in bus.writes)


def test_browser_heartbeat_loss_cancels_the_trajectory_and_requires_reenable(rig):
    control, bus, clock = rig
    control.enable([JOINTS[0]])
    control.move({JOINTS[0]: 2107}, 1)
    clock.advance(1.6)
    control.tick()
    assert not control.enabled and not control.move_job
    assert bus.read('Goal_Position', JOINTS[0]) == bus.read('Present_Position', JOINTS[0])
    with pytest.raises(ValueError, match='使能'):
        control.move({JOINTS[0]: 2100}, 1)


def test_loaded_stall_latches_a_fault_instead_of_accumulating_more_position_targets(rig):
    control, bus, _ = rig
    control.enable([JOINTS[0]])
    bus.follow = False
    bus.registers[JOINTS[0]]['Present_Load'] = 900
    control.move({JOINTS[0]: 2150}, 600)
    tick(rig, 50)
    assert '受阻' in control.fault
    assert not control.enabled
    assert bus.read('Goal_Position', JOINTS[0]) == 2067


def test_six_motor_response_runs_sequentially_and_returns_each_motor_to_its_start(rig):
    control, bus, _ = rig
    control.enable(list(JOINTS))
    control.start_step(list(JOINTS), 5, 1, .8)
    tick(rig, 2200)
    result = control.last_result
    assert result['completed'] and control.job is None
    steps = [s for s in result['segments'] if s['kind'] == 'step']
    assert len(steps) == 12
    assert {s['joint'] for s in steps} == set(JOINTS)
    assert len(result['motor_identities']) == 6
    assert result['motor_identities'][JOINTS[0]]['Homing_Offset'] == -1110
    assert all(s['metrics']['steady_peak_error_ticks'] == 0 for s in steps)
    assert all(v['Present_Position'] == 2067 for v in bus.registers.values())
    assert len(list(control.log_root.glob('response-*.json'))) == 1


def test_invalid_step_plan_is_rejected_without_sending_a_preposition(rig):
    control, bus, _ = rig
    control.enable([JOINTS[0]])
    bus.writes.clear()
    with pytest.raises(ValueError):
        control.start_step([JOINTS[0]], 90, 1, 1.2)
    assert not bus.writes and not control.job


def test_transport_changes_require_an_isolated_motor_and_reject_id_collisions(rig):
    control, bus, _ = rig
    with pytest.raises(ValueError, match='一只'):
        control.write_settings(JOINTS[0], {'ID': 7}, {'ID': 1}, True)
    assert not bus.writes


def test_single_motor_id_and_baud_updates_use_new_transport_for_readback(rig):
    control, bus, _ = rig
    control.online = [JOINTS[0]]
    control.discovery = {JOINTS[0]: control.discovery[JOINTS[0]]}
    bus.motors = {JOINTS[0]: bus.motors[JOINTS[0]]}
    response = control.write_settings(JOINTS[0], {'ID': 7}, {'ID': 1}, True)
    assert response['ids'][JOINTS[0]] == 7 and bus.motors[JOINTS[0]].id == 7
    response = control.write_settings(JOINTS[0], {'Baud_Rate': 1}, {'Baud_Rate': 0}, True)
    assert response['baudrate'] == 500000 and bus.baudrate == 500000


def test_parameter_catalog_has_no_overlapping_defined_bytes_or_writable_reserved_fields():
    bytes_used = []
    for register in REGISTERS:
        bytes_used.extend(range(register.address, register.address + register.size))
    assert len(bytes_used) == len(set(bytes_used))
    assert not BY_KEY['Return_Delay_Time'].writable
    assert not BY_KEY['Lock'].writable
    assert not BY_KEY['Maximum_Acceleration'].writable
    assert BY_KEY['Minimum_Startup_Force'].size == BY_KEY['Integral_Limit'].size == 1


def test_unmapped_device_id_collision_is_probed_before_any_eeprom_write(rig):
    control, bus, _ = rig
    control.online = [JOINTS[0]]
    bus.id_available = lambda motor_id: False
    with pytest.raises(ValueError, match='占用'):
        control.write_settings(JOINTS[0], {'ID': 7}, {'ID': 1}, True)
    assert not bus.writes


def test_interrupted_configuration_rolls_back_before_relocking(rig):
    control, bus, _ = rig
    write = bus.write

    def interrupted(key, joint, value, **kwargs):
        if key == 'I_Coefficient' and value == 5:
            raise KeyboardInterrupt('worker interrupted')
        return write(key, joint, value, **kwargs)

    bus.write = interrupted
    with pytest.raises(KeyboardInterrupt):
        control.write_settings(JOINTS[0], {'P_Coefficient': 63, 'I_Coefficient': 5},
                               {'P_Coefficient': 64, 'I_Coefficient': 4}, True)
    assert bus.read('P_Coefficient', JOINTS[0]) == 64
    assert bus.read('Lock', JOINTS[0]) == 1


def test_failed_disk_logging_cannot_prevent_holding_an_active_response(rig, monkeypatch):
    control, bus, _ = rig
    control.enable([JOINTS[0]])
    control.start_step([JOINTS[0]], 5, 1, .8)
    tick(rig, 5)
    def disk_full(*args):
        raise OSError('disk full')
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.atomic_json', disk_full)
    control.hold('test stop')
    assert bus.read('Goal_Position', JOINTS[0]) == bus.read('Present_Position', JOINTS[0])
    assert control.job is None and control.last_result['log_error'] == 'disk full'


def test_velocity_mode_motion_register_cannot_bypass_position_control_gates(rig):
    control, bus, _ = rig
    bus.registers[JOINTS[0]]['Operating_Mode'] = 1
    with pytest.raises(ValueError, match='驱动'):
        control.write_settings(JOINTS[0], {'Goal_Velocity': 1000}, {'Goal_Velocity': 0})
    assert not bus.writes


def test_unexpected_torque_activation_in_parameter_transaction_is_rolled_back_and_released(rig):
    control, bus, _ = rig
    write = bus.write

    def auto_enable(key, joint, value, **kwargs):
        write(key, joint, value, **kwargs)
        if key == 'Torque_Limit':
            bus.registers[joint]['Torque_Enable'] = 1

    bus.write = auto_enable
    with pytest.raises(RuntimeError, match='意外'):
        control.write_settings(JOINTS[0], {'Torque_Limit': 900}, {'Torque_Limit': 1000})
    assert bus.read('Torque_Limit', JOINTS[0]) == 1000
    assert bus.read('Torque_Enable', JOINTS[0]) == 0
    assert bus.read('Lock', JOINTS[0]) == 1


def test_relocking_is_attempted_even_when_final_torque_read_fails(rig):
    control, bus, _ = rig
    read, count = bus.read, 0

    def failed_final_torque_read(key, joint, **kwargs):
        nonlocal count
        if key == 'Torque_Enable':
            count += 1
            if count >= 3 and joint == JOINTS[0]:
                raise ConnectionError('lost torque readback')
        return read(key, joint, **kwargs)

    bus.read = failed_final_torque_read
    # Restrict the preflight loop to one channel to place failure in final cleanup.
    control.online = [JOINTS[0]]
    with pytest.raises(RuntimeError, match='扭矩'):
        control.write_settings(JOINTS[0], {'P_Coefficient': 63}, {'P_Coefficient': 64}, True)
    assert read('Lock', JOINTS[0]) == 1
    assert ('Lock', JOINTS[0], 1) in bus.writes


def test_fault_recovery_can_relock_an_unloaded_device_but_never_resumes_motion(rig):
    control, bus, _ = rig
    control.fault = 'configuration lock check failed'
    bus.registers[JOINTS[0]]['Lock'] = 0
    control.clear_fault()
    assert bus.read('Lock', JOINTS[0]) == 1 and not control.fault
    assert not control.enabled and not any(k == 'Goal_Position' for k, _, _ in bus.writes)


def test_low_level_access_keeps_alarm_packets_readable_and_corrects_adjacent_byte_writes():
    from lerobot_arm_control.workbench.hardware import ServoAccess

    memory = bytearray(100)
    memory[24], memory[25] = 16, 8
    bus = Bus(1)
    bus._read = lambda address, size, *a, **kw: (
        int.from_bytes(memory[address:address+size], 'little'), 0, 32)
    bus._decode_sign = lambda key, values: values
    bus._encode_sign = lambda key, values: values
    bus._is_comm_success = lambda code: code == 0

    def raw_write(address, size, motor_id, value, **kwargs):
        memory[address:address+size] = value.to_bytes(size, 'little')
        return 0, 32

    bus._write = raw_write
    access = ServoAccess(bus)
    assert access.read('Minimum_Startup_Force', JOINTS[0]) == 16
    access.write('Minimum_Startup_Force', JOINTS[0], 0)
    assert memory[24] == 0 and memory[25] == 8


def test_live_torque_after_reconnect_blocks_bounds_edit_without_writes(rig):
    control, bus, _ = rig
    bus.registers[JOINTS[0]]['Torque_Enable'] = 1
    assert not control.enabled
    with pytest.raises(ValueError, match='释放'):
        control.configure_bounds(JOINTS[0], [2000, 2200])
    assert not bus.writes


def test_saved_bounds_survive_reconnect_without_recentering_or_motor_writes(rig, tmp_path):
    control, bus, clock = rig
    control.bounds_path = tmp_path / 'ranges.local.json'
    control.context = {'arm': 'follower', 'port': 'test-bus', 'baudrate': 1000000}
    control.command('bounds', {'joint': JOINTS[0], 'bounds': [2000, 2200], 'persist': True})
    bus.registers[JOINTS[0]]['Present_Position'] = 2300
    other = Workbench(bus, control.discovery, log_root=tmp_path / 'other', clock=clock,
                      context=control.context, bounds_path=control.bounds_path)
    assert other.bounds[JOINTS[0]] == [2000, 2200]
    assert other.bounds_info[JOINTS[0]]['source'] == 'saved'
    with pytest.raises(ValueError, match='不在动作范围'):
        other.enable([JOINTS[0]])
    assert not bus.writes


@pytest.mark.parametrize('change', ['Homing_Offset', 'Min_Position_Limit', 'ID', 'port'])
def test_saved_range_rejects_changed_reference_or_device(rig, tmp_path, change):
    control, bus, clock = rig
    control.bounds_path = tmp_path / 'ranges.local.json'
    control.context = {'arm': 'follower', 'port': 'test-bus'}
    control.configure_bounds(JOINTS[0], [2000, 2200], True)
    if change == 'port':
        control.context = {**control.context, 'port': 'other-bus'}
    elif change == 'ID':
        control.discovery[JOINTS[0]]['id'] = 9
    else:
        bus.registers[JOINTS[0]][change] += 1
    other = Workbench(bus, control.discovery, log_root=tmp_path / 'other', clock=clock,
                      context=control.context, bounds_path=control.bounds_path)
    assert other.bounds[JOINTS[0]] is None
    assert '不匹配' in other.bounds_info[JOINTS[0]]['reason']
    with pytest.raises(ValueError, match='重新设置'):
        other.enable([JOINTS[0]])
    assert not bus.writes


def test_saved_ranges_are_separate_for_each_arm(rig, tmp_path):
    control, bus, clock = rig
    control.bounds_path = tmp_path / 'ranges.local.json'
    control.context = {'arm': 'follower'}
    control.configure_bounds(JOINTS[0], [2000, 2200], True)
    control.context = {'arm': 'leader'}
    control.configure_bounds(JOINTS[0], [1900, 2300], True)
    other = Workbench(bus, control.discovery, log_root=tmp_path / 'other', clock=clock,
                      context={'arm': 'follower'}, bounds_path=control.bounds_path)
    assert other.bounds[JOINTS[0]] == [2000, 2200]
    assert not bus.writes


def test_corrupt_range_file_blocks_enable_instead_of_restoring_default(rig, tmp_path):
    control, bus, clock = rig
    saved = tmp_path / 'ranges.local.json'
    saved.write_text('{broken')
    other = Workbench(bus, control.discovery, log_root=tmp_path / 'other', clock=clock,
                      context={'arm': 'follower'}, bounds_path=saved)
    assert all(v is None for v in other.bounds.values())
    with pytest.raises(ValueError):
        other.enable([JOINTS[0]])
    assert not bus.writes


def test_failed_range_save_does_not_apply_unsaved_bounds(rig, tmp_path, monkeypatch):
    control, bus, _ = rig
    control.bounds_path = tmp_path / 'ranges.local.json'
    control.context = {'arm': 'follower'}
    original = control.bounds[JOINTS[0]][:]
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.atomic_json', fail)
    with pytest.raises(OSError):
        control.configure_bounds(JOINTS[0], [2000, 2200], True)
    assert control.bounds[JOINTS[0]] == original
    assert not bus.writes


def test_outside_eeprom_pose_shows_the_actual_conflict(rig, tmp_path):
    control, bus, clock = rig
    bus.registers[JOINTS[5]].update(Present_Position=590, Min_Position_Limit=1441)
    other = Workbench(bus, control.discovery, log_root=tmp_path, clock=clock)
    assert other.bounds[JOINTS[5]] is None
    assert other.native_limits[JOINTS[5]] == [1441, 3440]
    assert '590' in other.bounds_info[JOINTS[5]]['reason']
    with pytest.raises(ValueError):
        other.configure_bounds(JOINTS[5], [452, 1893])
    assert not bus.writes


def test_position_limit_edit_cannot_strand_the_current_pose_outside_eeprom(rig):
    control, bus, _ = rig
    with pytest.raises(ValueError, match='必须包含当前位置'):
        control.write_settings(JOINTS[0], {'Min_Position_Limit': 2200},
                               {'Min_Position_Limit': 844}, True)
    assert not bus.writes


@pytest.mark.parametrize('duration,fail_twice', [(.01, False), (.01, True), (.051, False)])
def test_worker_retries_only_one_short_failed_read(rig, duration, fail_twice, monkeypatch):
    from lerobot_arm_control.workbench.worker import sample_state

    control, bus, clock = rig
    calls = []
    original_tick = control.tick

    def intermittent():
        calls.append(True)
        if len(calls) == 1 or fail_twice:
            clock.advance(duration)
            raise ConnectionError('bad packet')
        return original_tick()

    monkeypatch.setattr(control, 'tick', intermittent)
    if duration > .05 or fail_twice:
        with pytest.raises(ConnectionError):
            sample_state(control)
    else:
        assert sample_state(control)['frame'] is not None
    assert len(calls) == (1 if duration > .05 else 2)
    assert control.sample_retries == (0 if duration > .05 else 1)
    assert not bus.writes
