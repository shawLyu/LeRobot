"""Receive locally planned results and execute validated segments with a fake motor bus."""

import copy
import hashlib
from types import SimpleNamespace

import pytest
from test_push_t_remote import motor as motor  # noqa: F401
from test_push_t_remote import observed as observed  # noqa: F401

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.tasks.push_t.push_t_receiver import PushReceiver


@pytest.fixture
def receiver(motor, tmp_path):
    instance, bus, clock, state, segments = motor
    state['captured_unix_ns'] = 1_000_000_000
    model_hash = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    robot = SimpleNamespace(motor=instance, metadata={'fingerprint': {'mjcf_sha256': model_hash}})
    endpoint = PushReceiver(robot, tmp_path/'received', clock=lambda: clock[0])
    endpoint.observe(state)
    command = {'version': 1, 'id': 1, 'op': 'push_execute',
               'control_config_sha256': CONTROL_CONFIG_SHA256, 'model_sha256': model_hash,
               'reference': copy.deepcopy(state), 'segments': segments}
    return endpoint, command, motor


def test_received_local_plan_runs_on_bus_owner_and_ends_holding(receiver):
    endpoint, command, (motor, bus, clock, state, _) = receiver
    status = endpoint.handle(command)
    assert status['planning_location'] == 'local'
    assert status['execution_location'] == 'remote'
    assert status['job_id'] == 1
    assert motor.status()['phase'] == 'pending'
    assert bus.writes == []
    for _ in range(160):
        clock[0] += 1/60
        for values in bus.registers.values():
            if values['Torque_Enable']:
                values['Present_Position'] = values['Goal_Position']
        endpoint.observe(state)
        endpoint.handle({**command, 'op': 'ping'})
        motor.tick(bus.sync_read('Present_Position'),
                   {key: bus.sync_read(key) for key in ('Status', 'Torque_Enable')})
        if motor.status()['phase'] == 'completed':
            break
    assert motor.status()['phase'] == 'completed'
    assert bus.sync_read('Goal_Position') == bus.sync_read('Present_Position')
    assert all(bus.sync_read('Torque_Enable').values())


@pytest.mark.parametrize('change', ['config', 'model', 'scene', 'stale', 'nan', 'speed',
                                    'duration', 'limits', 'continuity', 'recovery'])
def test_bad_local_plans_are_rejected_before_any_motor_write(receiver, change):
    endpoint, command, (motor, bus, _, state, _) = receiver
    if change == 'config':
        command['control_config_sha256'] = 'old'
    elif change == 'model':
        command['model_sha256'] = 'old'
    elif change == 'scene':
        command['reference']['objects']['wood_t']['xy'][0] += .01
    elif change == 'stale':
        command['reference']['captured_unix_ns'] -= 800_000_000
    elif change == 'nan':
        command['segments'][0]['end'][0] = float('nan')
    elif change == 'speed':
        command['segments'][0]['speed_rad_s'] = 1.0
    elif change == 'duration':
        command['segments'][0]['seconds'] = .06
    elif change == 'limits':
        command['segments'][0]['end'][0] = 10
    elif change == 'continuity':
        command['segments'].append(copy.deepcopy(command['segments'][0]))
    elif change == 'recovery':
        command['recovery'] = 'yes'
    with pytest.raises(ValueError):
        endpoint.handle(command)
    assert motor.status()['phase'] == 'idle'
    assert bus.writes == []
    assert endpoint.state['objects'] == state['objects']


def test_local_heartbeat_timeout_stops_pending_execution(receiver):
    endpoint, command, (motor, bus, clock, _, _) = receiver
    endpoint.handle(command)
    clock[0] = .8
    endpoint.check_heartbeat()
    motor.tick(bus.sync_read('Present_Position'),
               {key: bus.sync_read(key) for key in ('Status', 'Torque_Enable')})
    assert motor.status()['phase'] == 'stopped'
    assert 'heartbeat timeout' in motor.status()['reason']
    assert bus.writes == []


def test_preview_result_is_saved_without_enabling_hardware(receiver):
    endpoint, command, (motor, bus, _, _, _) = receiver
    endpoint.robot.motor = None
    endpoint.handle({**command, 'op': 'push_result',
                     'result': {'passed': True, 'phase': 'preview'}})
    assert endpoint.last_result['passed']
    assert (endpoint.output/'local-results.jsonl').exists()
    with pytest.raises(ValueError, match='disabled'):
        endpoint.handle(command)
    assert motor.status()['phase'] == 'idle'
    assert bus.writes == []
