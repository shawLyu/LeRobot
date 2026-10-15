"""Native tuning must stay volatile, bounded, measured and recoverable."""

import copy
import math

import pytest
from fakes import Bus

from lerobot_arm_control.diagnostics.native_pid import NativeBench, metrics, validate_plan


def zero_record():
    fingerprint = {'registers': {'Operating_Mode': 0, 'Angular_Resolution': 1,
                   'Min_Position_Limit': 800, 'Max_Position_Limit': 3400}}
    return {'version': 1, 'identity': fingerprint, 'ticks_per_turn': 4096,
            'direction': 1, 'zero_raw': 2100, 'allowed_degrees': [-90, 90]}


def test_large_angle_steps_use_half_amplitude_endpoints_inside_authorized_travel():
    zero = zero_record()
    bounds, targets = validate_plan(zero, zero['identity'], [5, 30, 90])
    assert targets[-1] == (90, (1588, 2612))
    assert bounds[0] < targets[-1][1][0] < targets[-1][1][1] < bounds[1]


@pytest.mark.parametrize('issue', ['identity', 'limit', 'mode', 'scale', 'nan', 'amplitude'])
def test_invalid_origin_or_plan_is_rejected_before_a_bus_is_written(issue):
    zero = zero_record()
    actual = copy.deepcopy(zero['identity'])
    amplitudes = [90]
    if issue == 'identity':
        actual['port'] = 'changed'
    elif issue == 'limit':
        zero['identity']['registers']['Min_Position_Limit'] = 2000
        actual = copy.deepcopy(zero['identity'])
    elif issue == 'mode':
        zero['identity']['registers']['Operating_Mode'] = 1
        actual = copy.deepcopy(zero['identity'])
    elif issue == 'scale':
        zero['ticks_per_turn'] = 2048
    else:
        amplitudes = [math.nan if issue == 'nan' else 180]
    with pytest.raises(ValueError):
        validate_plan(zero, actual, amplitudes)


def test_one_tick_peak_error_cannot_pass_by_averaging_positive_and_negative_errors():
    frames = [{'t': i / 200, 'position': 2100 if i < 10 else 2200 + (1 if i % 2 else -1)}
              for i in range(241)]
    result = metrics({'frames': frames, 'start': 2100, 'target': 2200,
                      'command_ack_seconds': .002})
    assert result['steady_peak_error_ticks'] == 1
    assert not result['encoder_precision_passed']
    assert not result['passed']


def test_fast_exact_encoder_response_and_reversed_direction_keep_command_time():
    for direction in (-1, 1):
        frames = [{'t': i / 200,
                   'position': 2100 + direction * round(100 * (1 - math.exp(-i / 200 / .01)))}
                  for i in range(241)]
        result = metrics({'frames': frames, 'start': 2100, 'target': 2100 + direction * 100,
                          'command_ack_seconds': .002})
        assert .02 < result['time_to_90_s'] < .03
        assert result['steady_peak_error_ticks'] == 0
        assert result['passed']
        assert result['command_ack_seconds'] == .002


def bench_bus():
    bus = Bus()
    for values in bus.registers.values():
        values.update(I_Coefficient=0, D_Coefficient=32, CW_Dead_Zone=1, CCW_Dead_Zone=1,
                      Integral_Limit=0, Minimum_Startup_Force=16)
    return bus


def test_locked_native_changes_restore_original_gains_without_any_eeprom_unlock():
    bus = bench_bus()
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    bench.configure((32, 1, 48), 0)
    assert bus.read('P_Coefficient', 'shoulder_pan') == 32
    assert bus.read('CW_Dead_Zone', 'shoulder_pan') == 0
    assert bench.close() == []
    assert bus.read('P_Coefficient', 'shoulder_pan') == 16
    assert bus.read('CW_Dead_Zone', 'shoulder_pan') == 1
    assert not any(register == 'Lock' for register, _, _ in bus.writes)


def test_alarm_cleanup_disables_torque_without_reissuing_a_goal_that_clears_protection():
    bus = bench_bus()
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    bench.activated = True
    bus.registers['shoulder_pan'].update(Status=32, Torque_Enable=1)
    assert bench.close() == []
    assert bus.read('Torque_Enable', 'shoulder_pan') == 0
    assert not any(register == 'Goal_Position' for register, _, _ in bus.writes)


def test_invalid_coefficients_and_unlocked_storage_never_write():
    bus = bench_bus()
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    with pytest.raises(ValueError):
        bench.configure((255, 0, 32), 0)
    assert bus.writes == []
    bus.registers['shoulder_pan']['Lock'] = 0
    with pytest.raises(ValueError, match='locked'):
        NativeBench(bus, 'shoulder_pan', (1000, 3000))
    assert bus.writes == []


def test_restoration_after_normal_motion_disables_torque_last_among_goal_commands():
    bus = bench_bus()
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    bench.activated = True
    bus.registers['shoulder_pan']['Torque_Enable'] = 1
    bench.configure((32, 1, 48), 0)
    assert bench.close() == []
    movement = [(key, value) for key, _, value in bus.writes
                if key in ('Goal_Position', 'Torque_Enable')]
    assert movement == [('Goal_Position', 2200), ('Torque_Enable', 0)]
    assert bus.read('Torque_Enable', 'shoulder_pan') == 0


def test_excessive_overshoot_aborts_the_step_and_keeps_the_partial_trace(monkeypatch):
    bus = bench_bus()
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    observations = iter([{'position': 2200}, {'position': 2340, 'sampled_monotonic': 10.}])
    monkeypatch.setattr(bench, 'sample', lambda: next(observations))
    with pytest.raises(RuntimeError, match='overshoot'):
        bench.step(2300, 1., 200)
    assert bench.last_segment['frames'][-1]['position'] == 2340
    assert bus.writes == [('Goal_Position', 'shoulder_pan', 2300)]


def test_zero_capture_only_reads_and_never_overwrites_an_existing_origin(tmp_path, monkeypatch):
    import json
    from contextlib import contextmanager
    from types import SimpleNamespace

    from lerobot_arm_control.diagnostics import native_pid

    bus = bench_bus()

    @contextmanager
    def connection(arm):
        yield bus

    monkeypatch.setattr(native_pid, 'connected', connection)
    monkeypatch.setattr(native_pid, 'identity', lambda *args: zero_record()['identity'])
    monkeypatch.setattr(native_pid.time, 'sleep', lambda _: None)
    args = SimpleNamespace(arm='follower', joint='shoulder_pan', allowed_degrees=[-90, 90],
                           confirm_zero=True, output=tmp_path / 'zero.json')
    native_pid.capture_zero(args)
    assert json.loads(args.output.read_text())['zero_raw'] == 2200
    assert bus.writes == []
    with pytest.raises(ValueError, match='new output'):
        native_pid.capture_zero(args)
    assert bus.writes == []


def test_capture_command_requires_explicit_execution_before_opening_a_bus(monkeypatch):
    from lerobot_arm_control.diagnostics import native_pid

    monkeypatch.setattr('sys.argv', ['native-pid', 'capture', '--zero', 'missing.json',
                                    '--p', '16', '--i', '0', '--d', '32'])
    monkeypatch.setattr(native_pid, 'connected', lambda *args:
                        pytest.fail('must reject before opening a motor'))
    with pytest.raises(SystemExit) as result:
        native_pid.main()
    assert result.value.code == 2


def test_integral_limit_alias_does_not_mutate_the_shared_driver_control_table():
    from types import SimpleNamespace

    from lerobot_arm_control.diagnostics.native_pid import add_native_registers

    original = {'sts3215': {'P_Coefficient': (21, 1)}}
    bus = SimpleNamespace(model_ctrl_table=original)
    add_native_registers(bus)
    assert bus.model_ctrl_table['sts3215']['Integral_Limit'] == (25, 1)
    assert 'Integral_Limit' not in original['sts3215']


def test_partial_gain_write_failure_still_restores_the_original_configuration():
    bus = bench_bus()
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    original_write = bus.write

    def dropped_integral(register, joint, value, **kwargs):
        if register != 'I_Coefficient':
            original_write(register, joint, value, **kwargs)

    bus.write = dropped_integral
    with pytest.raises(RuntimeError, match='readback'):
        bench.configure((64, 4, 128), 0)
    assert bench.close() == []
    assert bus.read('P_Coefficient', 'shoulder_pan') == 16
    assert bus.read('Torque_Enable', 'shoulder_pan') == 0


@pytest.mark.parametrize('issue', ['alarm', 'position', 'torque', 'temperature'])
def test_bad_live_feedback_aborts_without_a_goal_write(monkeypatch, issue):
    from lerobot_arm_control.diagnostics import native_pid

    bus = bench_bus()
    bus.registers['shoulder_pan']['Present_Temperature'] = 60 if issue == 'temperature' else 25
    bench = NativeBench(bus, 'shoulder_pan', (1000, 3000))
    bench.activated = issue == 'torque'
    fields = {'Present_Position': 3100 if issue == 'position' else 2200,
              'Goal_Position': 2200, 'Present_Velocity': 0, 'Present_Load': 0,
              'Present_Current': 0, 'Present_Voltage': 53,
              'Status': 32 if issue == 'alarm' else 0, 'Torque_Enable': 0}
    monkeypatch.setattr(native_pid, 'read_control_feedback', lambda *args, **kwargs:
                        {key: {'shoulder_pan': value} for key, value in fields.items()})
    with pytest.raises(RuntimeError):
        bench.sample()
    assert bus.writes == []


def test_zero_repeatability_requires_each_settled_sample_to_match_the_saved_origin():
    from lerobot_arm_control.diagnostics.native_pid import zero_metrics

    frames = [{'t': i / 200, 'position': 2067 + (i % 2)} for i in range(201)]
    result = zero_metrics({'frames': frames}, 2067)
    assert result['peak_error_ticks'] == 1
    assert 0 < result['exact_fraction'] < 1


def test_failed_final_readback_exits_unsuccessfully_and_keeps_restoration_evidence(
        tmp_path, monkeypatch):
    import json
    from contextlib import contextmanager
    from types import SimpleNamespace

    from lerobot_arm_control.diagnostics import native_pid

    bus = bench_bus()
    zero = zero_record()
    zero['identity']['joint'] = 'shoulder_pan'
    zero_file = tmp_path / 'zero.json'
    zero_file.write_text(json.dumps(zero))
    identities = iter([zero['identity'], None])

    @contextmanager
    def connection(arm):
        yield bus

    def read_identity(*args):
        value = next(identities)
        if value is None:
            raise ConnectionError('final identity read failed')
        return value

    def measured_step(self, target, *args):
        return {'start': target - 100, 'target': target, 'command_ack_seconds': .002,
                'frames': [{'t': i / 200, 'position': target if i else target - 100}
                           for i in range(241)]}

    monkeypatch.setattr(native_pid, 'connected', connection)
    monkeypatch.setattr(native_pid, 'add_native_registers', lambda bus: None)
    monkeypatch.setattr(native_pid, 'identity', read_identity)
    monkeypatch.setattr(NativeBench, 'sample', lambda self: {'position': 2200})
    monkeypatch.setattr(NativeBench, 'step', measured_step)
    args = SimpleNamespace(zero=zero_file, arm='follower', output=tmp_path / 'trial',
        p=64, i=4, d=128, deadzone=0, acceleration=None, startup_force=None,
        integral_limit=8, sample_hz=200, response_ms=50, amplitudes=[5],
        repeats=1, dwell=1.2)
    with pytest.raises(SystemExit) as result:
        native_pid.capture(args)
    report = json.loads((args.output / 'result.json').read_text())
    assert result.value.code == 1
    assert report['completed'] and not report['performance_passed']
    assert report['error'] == 'final identity read failed'
    assert report['restoration_errors'] == []
    assert bus.read('Torque_Enable', 'shoulder_pan') == 0
    assert bus.read('P_Coefficient', 'shoulder_pan') == 16
