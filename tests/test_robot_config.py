"""Saved hardware parameters are checked before any controller can own an arm."""

import copy
import json
from dataclasses import replace
from types import SimpleNamespace

import pytest

from lerobot_arm_control.core.config import load_arms, load_robot_config
from lerobot_arm_control.robot import connection
from lerobot_arm_control.robot import parameters as manager
from lerobot_arm_control.robot.arm import RobotArm


@pytest.fixture
def hardware(monkeypatch, tmp_path):
    arm = load_arms()['follower']

    class Bus:
        def __init__(self):
            self.motors = {j: SimpleNamespace(id=i) for j, i in arm.motors.items()}
            self.values = {j: {**p['persistent'], **p['startup'], **p['identity'],
                               'Lock': 1, 'Torque_Enable': 0, 'Moving': 0, 'Status': 0,
                               'Present_Temperature': 25, 'Present_Voltage': 53,
                               'Present_Position': 2000, 'Goal_Position': 2000}
                           for j, p in arm.parameters.items()}
            for values in self.values.values():
                for register in manager.REGISTERS:
                    values.setdefault(register.key, 0)
            self.writes = []
            self.fail = None
            self.no_ack = False
            self.is_connected = False
            self.port_handler = SimpleNamespace(ser=SimpleNamespace(exclusive=False))

        def read(self, key, joint, **kwargs):
            return self.values[joint][key]

        def write(self, key, joint, value, **kwargs):
            self.writes.append((joint, key, value))
            if self.fail == (joint, key, value):
                raise ConnectionError('injected failure')
            self.values[joint][key] = value
            if self.no_ack:
                raise ConnectionError('ACK suppressed')

        def connect(self, **kwargs):
            self.is_connected = True

        def disconnect(self, **kwargs):
            assert kwargs == {'disable_torque': False}
            self.is_connected = False

        def ping(self, joint, **kwargs):
            return 777

    bus = Bus()
    monkeypatch.setattr(manager, 'activation_marker', lambda a: tmp_path/'activation.json')
    monkeypatch.setattr(manager, 'ServoAccess', lambda b: b)
    monkeypatch.setattr(manager, 'install_table', lambda b: None)
    monkeypatch.setattr(manager.time, 'sleep', lambda _: None)
    monkeypatch.setattr(connection, 'make_bus', lambda a: bus)
    return arm, bus


def test_matching_baseline_leaves_loaded_arm_and_eeprom_untouched(hardware):
    arm, bus = hardware
    for values in bus.values.values():
        values['Torque_Enable'] = 1
    report = manager.ensure_parameters(bus, arm)
    assert len(report['checked']) == 6 and not report['missing'] and not report['restored']
    assert bus.writes == []


def test_restore_differences_only_and_never_writes_motion_or_torque(hardware):
    arm, bus = hardware
    bus.values['elbow_flex']['I_Coefficient'] = 0
    bus.values['gripper']['Torque_Limit'] = 1000
    report = manager.ensure_parameters(bus, arm)
    assert bus.values['elbow_flex']['I_Coefficient'] == 8
    assert bus.values['gripper']['Torque_Limit'] == 500
    assert len(report['restored']) == 2
    assert [w[1] for w in bus.writes] == ['Lock', 'I_Coefficient', 'Torque_Limit', 'Lock']
    assert all(v['Torque_Enable'] == 0 and v['Goal_Position'] == 2000
               for v in bus.values.values())
    assert all(v['Lock'] == 1 for v in bus.values.values())


@pytest.mark.parametrize('key,value', [('Torque_Enable', 1), ('Moving', 1), ('Status', 32),
                                      ('Present_Temperature', 80), ('Present_Voltage', 20)])
def test_unsafe_other_joint_prevents_all_writes(hardware, key, value):
    arm, bus = hardware
    bus.values['gripper']['P_Coefficient'] += 1
    bus.values['shoulder_lift'][key] = value
    with pytest.raises(RuntimeError):
        manager.ensure_parameters(bus, arm)
    assert not bus.writes


def test_failure_rolls_back_and_relocks_all_eeprom(hardware):
    arm, bus = hardware
    bus.values['elbow_flex']['I_Coefficient'] = 0
    bus.values['wrist_flex']['P_Coefficient'] = 50
    bus.fail = ('wrist_flex', 'P_Coefficient', 48)
    with pytest.raises(RuntimeError, match='restore failed'):
        manager.ensure_parameters(bus, arm)
    assert bus.values['elbow_flex']['I_Coefficient'] == 0
    assert bus.values['wrist_flex']['P_Coefficient'] == 50
    assert all(v['Lock'] == 1 for v in bus.values.values())


def test_no_ack_with_successful_readback_is_supported(hardware):
    arm, bus = hardware
    bus.no_ack = True
    bus.values['gripper']['P_Coefficient'] = 16
    assert len(manager.ensure_parameters(bus, arm)['restored']) == 1
    assert bus.values['gripper']['Lock'] == 1


@pytest.mark.parametrize('group,key,value', [('persistent', 'Torque_Enable', 1),
    ('startup', 'Goal_Position', 2000), ('persistent', 'Min_Position_Limit', 4095),
    ('startup', 'Goal_Velocity', 100), ('identity', 'ID', 99),
    ('persistent', 'P_Coefficient', 3.5)])
def test_invalid_config_fails_before_writing(hardware, group, key, value):
    arm, bus = hardware
    profiles = copy.deepcopy(arm.parameters)
    profiles['gripper'][group][key] = value
    with pytest.raises(ValueError):
        manager.ensure_parameters(bus, replace(arm, parameters=profiles))
    assert not bus.writes


@pytest.mark.parametrize('key', ['ID', 'Model_Number', 'Maximum_Acceleration',
                                'Firmware_Major_Version'])
def test_wrong_device_or_factory_data_is_never_overwritten(hardware, key):
    arm, bus = hardware
    bus.values['gripper'][key] += 1
    with pytest.raises(RuntimeError, match='identity/factory mismatch'):
        manager.ensure_parameters(bus, arm)
    assert not bus.writes


def test_partial_discovery_checks_online_axes_and_reports_missing(hardware):
    arm, bus = hardware
    bus.motors = {'gripper': bus.motors['gripper']}
    bus.values['gripper']['P_Coefficient'] = 16
    report = manager.ensure_parameters(bus, arm)
    assert report['checked'] == ['gripper'] and len(report['missing']) == 5
    assert len(report['restored']) == 1
    assert all(j == 'gripper' for j, _, _ in bus.writes)


def test_connection_checks_and_binds_arm_before_exposing_bus(hardware):
    arm, bus = hardware
    bus.values['gripper']['P_Coefficient'] = 16
    robot = RobotArm(arm)
    with robot.connect() as opened:
        assert opened is robot and robot.bus is bus
        assert bus.robot_arm is robot
        assert robot.startup_report['restored'][0]['after'] == 48
    assert robot.bus is None and not bus.is_connected


def test_connection_does_not_expose_unrestored_loaded_arm(hardware):
    arm, bus = hardware
    bus.values['gripper']['P_Coefficient'] = 16
    bus.values['shoulder_lift']['Torque_Enable'] = 1
    with pytest.raises(RuntimeError, match='torque is enabled'):
        with connection.connected(arm):
            pytest.fail('controller received an unverified bus')
    assert not bus.is_connected and not bus.writes


def test_modified_calibration_rejected_before_any_restore(hardware):
    arm, bus = hardware
    resources = copy.deepcopy(arm.resources)
    resources['calibration']['sha256'] = '0'*64
    with pytest.raises(ValueError, match='calibration differs'):
        with RobotArm(replace(arm, resources=resources)).connect():
            pytest.fail('mismatched calibration accepted')
    assert not bus.writes


def test_capture_excludes_live_goals_torque_and_feedback(hardware):
    arm, bus = hardware
    values = manager.capture_parameters(bus, arm)
    assert values == arm.parameters
    assert not bus.writes
    for p in values.values():
        assert not {'Torque_Enable', 'Goal_Position', 'Present_Position'} & {
            k for group in p.values() for k in group}


def test_workspace_json_and_private_port_override(tmp_path, monkeypatch):
    config = load_robot_config()
    directory = tmp_path/'configs'
    directory.mkdir()
    (directory/'robot.json').write_text(json.dumps(config))
    (directory/'robot.local.json').write_text(json.dumps(
        {'arms': {'follower': {'port': '/dev/serial/by-id/private-device'}}}))
    monkeypatch.setenv('LEROBOT_ARM_WORKSPACE', str(tmp_path))
    arm = load_arms()['follower']
    assert arm.port == '/dev/serial/by-id/private-device'
    assert arm.parameters == config['arms']['follower']['parameters']
    (directory/'robot.local.json').write_text(json.dumps(
        {'arms': {'follower': {'parameters': {}}}}))
    with pytest.raises(ValueError, match='ports only'):
        load_arms()


def test_arm_context_closes_controller_before_disconnect_even_on_error(hardware):
    arm, bus = hardware
    robot = RobotArm(arm)
    closed = []
    with pytest.raises(ValueError, match='caller failure'):
        with robot.connect():
            robot.controller = SimpleNamespace(close=lambda: closed.append(bus.is_connected))
            raise ValueError('caller failure')
    assert closed == [True]
    assert robot.controller is None and robot.bus is None and not bus.is_connected


def test_workbench_entry_restores_parameters_before_discovery_is_published(hardware, monkeypatch):
    from lerobot_arm_control.workbench import hardware as entry

    arm, bus = hardware
    bus.packet_handler = SimpleNamespace(ping=lambda port, motor_id: (777, 0, 0))
    bus._is_comm_success = lambda status: status == 0
    bus.set_baudrate = lambda baud: None
    monkeypatch.setattr(entry, 'make_bus', lambda arm: bus)
    monkeypatch.setattr(entry, 'ServoAccess', lambda bus: bus)
    monkeypatch.setattr(entry, 'install_table', lambda bus: None)
    bus.values['gripper']['P_Coefficient'] = 16
    with entry.open_hardware(arm, list(arm.motors.values()), arm.baudrate) as (opened, found):
        assert opened is bus
        assert all(d['online'] for d in found.values())
        assert bus.values['gripper']['P_Coefficient'] == 48
        assert len(bus.robot_arm.startup_report['restored']) == 1
    assert not bus.is_connected


def test_workbench_rejects_reassigned_motor_ids_before_open(hardware):
    from lerobot_arm_control.workbench.hardware import open_hardware

    arm, bus = hardware
    with pytest.raises(ValueError, match='different IDs/baudrate'):
        with open_hardware(arm, [7, 2, 3, 4, 5, 6], arm.baudrate):
            pytest.fail('reassigned devices received the old calibration baseline')
    assert not bus.is_connected and not bus.writes


@pytest.mark.parametrize('old_limits', [(0, 500), (3500, 4000)])
def test_restore_expands_limit_pair_before_contracting_the_other_end(hardware, old_limits):
    arm, bus = hardware
    values = bus.values['shoulder_pan']
    values['Min_Position_Limit'], values['Max_Position_Limit'] = old_limits
    original = bus.write

    def write(key, joint, value, **kwargs):
        current = bus.values[joint]
        if key == 'Min_Position_Limit':
            assert value < current['Max_Position_Limit']
        if key == 'Max_Position_Limit':
            assert value > current['Min_Position_Limit']
        original(key, joint, value, **kwargs)

    bus.write = write
    assert len(manager.ensure_parameters(bus, arm)['restored']) == 2
    saved = arm.parameters['shoulder_pan']['persistent']
    assert values['Min_Position_Limit'] == saved['Min_Position_Limit']
    assert values['Max_Position_Limit'] == saved['Max_Position_Limit']


def test_phase_restore_requires_confirmed_restart_across_later_connections(hardware):
    arm, bus = hardware
    bus.values['gripper']['Phase'] = 0
    with pytest.raises(RuntimeError, match='power-cycle'):
        manager.ensure_parameters(bus, arm)
    assert bus.values['gripper']['Phase'] == 12
    assert bus.values['gripper']['Lock'] == 1
    writes = list(bus.writes)
    with pytest.raises(RuntimeError, match='restart is pending'):
        with connection.connected(arm):
            pytest.fail('a later connection bypassed Phase activation')
    assert bus.writes == writes
    assert manager.confirm_power_cycle(arm)['restored'] == []
    assert not manager.activation_marker(arm).exists()


def test_workbench_keeps_release_available_when_loaded_arm_needs_restore(hardware, tmp_path):
    from lerobot_arm_control.workbench.controller import Workbench

    arm, bus = hardware
    bus.values['shoulder_lift']['Torque_Enable'] = 1
    bus.values['gripper']['P_Coefficient'] = 16
    RobotArm(arm).attach(bus, allow_unverified=True)
    discovery = {j: {'online': True, 'id': i} for j, i in arm.motors.items()}
    bench = Workbench(bus, discovery, log_root=tmp_path)
    assert 'torque is enabled' in bench.fault
    assert bench.parameters('gripper')['values']['P_Coefficient'] == 16
    with pytest.raises(ValueError, match='故障'):
        bench.enable(['gripper'])
    assert not bus.writes
    bench.release(list(arm.motors))  # An explicit operator action, never automatic unloading.
    bench.clear_fault()
    assert bus.values['gripper']['P_Coefficient'] == 48
    assert bench.fault is None and not bench.state()['parameter_check'].get('error')
    assert bench.bounds['shoulder_pan'] is not None
    assert all(key != 'Goal_Position' for _, key, _ in bus.writes)
