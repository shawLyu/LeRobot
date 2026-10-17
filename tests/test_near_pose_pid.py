"""Near-pose tuning keeps loaded joints energized and guards device references."""

import pytest
from test_servo_workbench import Bus, Clock

from lerobot_arm_control.diagnostics.near_pose_pid import PoseSweep, profile, summarize
from lerobot_arm_control.workbench.controller import Workbench


@pytest.fixture
def sweep(tmp_path, monkeypatch):
    bus, clock = Bus(), Clock()
    for v in bus.registers.values():
        v['Torque_Enable'] = 1
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.time.sleep', lambda _: None)
    monkeypatch.setattr('lerobot_arm_control.workbench.controller.read_control_feedback',
                        lambda bus, **kwargs: bus.feedback(**kwargs))
    discovery = {j: {'id': m.id, 'online': True} for j, m in bus.motors.items()}
    control = Workbench(bus, discovery, log_root=tmp_path, clock=clock)
    return PoseSweep(control), bus


def test_locked_tuning_never_releases_supporting_joints_or_changes_goal(sweep):
    tuner, bus = sweep
    joint = 'shoulder_pan'
    requested = profile(tuner.original[joint], 32, 2, 96)
    tuner.configure(joint, requested)
    assert all(k not in ('Torque_Enable', 'Goal_Position', 'Lock') for k, _, _ in bus.writes)
    assert all(j == joint for _, j, _ in bus.writes)
    assert all(v['Torque_Enable'] == 1 for v in bus.registers.values())
    assert {k: bus.read(k, joint) for k in requested} == requested


@pytest.mark.parametrize('key,value', [('Lock', 0), ('Homing_Offset', 100),
                                      ('Status', 32), ('Present_Temperature', 51)])
def test_changed_reference_or_fault_blocks_coefficient_writes(sweep, key, value):
    tuner, bus = sweep
    joint = 'shoulder_pan'
    bus.registers[joint][key] = value
    with pytest.raises(RuntimeError):
        tuner.configure(joint, tuner.original[joint])
    assert not any(k in tuner.original[joint] for k, _, _ in bus.writes)
    if key == 'Status':
        assert not any(k == 'Goal_Position' and j == joint for k, j, _ in bus.writes)


def test_only_tuning_fields_can_be_changed(sweep):
    tuner, bus = sweep
    with pytest.raises(ValueError):
        tuner.configure('shoulder_pan', {'Homing_Offset': 0})
    assert bus.writes == []


def test_invalid_values_are_rejected_before_any_writes(sweep):
    tuner, bus = sweep
    joint = 'shoulder_pan'
    with pytest.raises(ValueError):
        tuner.configure(joint, profile(tuner.original[joint], 0, 0, 32))
    assert bus.writes == []


def test_pose_near_limit_is_rejected_without_clipping_test_or_moving(sweep):
    tuner, bus = sweep
    bus.registers['shoulder_pan']['Present_Position'] = 845
    with pytest.raises(ValueError, match='clearance'):
        PoseSweep(tuner.control)
    assert bus.writes == []


def test_unpowered_joint_never_enabled_implicitly(sweep):
    tuner, bus = sweep
    bus.registers['shoulder_pan']['Torque_Enable'] = 0
    with pytest.raises(ValueError, match='already hold'):
        PoseSweep(tuner.control)
    assert bus.writes == []


def test_metrics_do_not_hide_zero_mean_oscillation_or_missing_response():
    def segment(jitter, t90):
        return {'kind': 'step', 'frames': [{'t': n / 100, 'position': 100 + jitter * (-1)**n}
                                          for n in range(121)],
                'metrics': {'steady_peak_error_ticks': jitter,
                            'steady_rms_error_ticks': jitter, 'overshoot_ticks': jitter,
                            'time_to_90_s': t90, 'max_sample_gap_s': .01}}
    steady = summarize([segment(0, .08)])
    jitter = summarize([segment(5, .03)])
    missing = summarize([segment(0, None)])
    assert steady['acceptable']
    assert not jitter['acceptable'] and jitter['jitter_peak_to_peak_ticks'] == 10
    assert jitter['score'] > steady['score']
    assert not missing['acceptable'] and missing['score'] > steady['score']


def test_overshoot_rejection_restores_gains_and_preserves_partial_trace(sweep, monkeypatch):
    from lerobot_arm_control.diagnostics.near_pose_pid import CandidateRejected

    tuner, bus = sweep
    joint = 'shoulder_pan'
    partial = {'frames': [{'t': .03, 'position': 2090}]}

    def rejected(*args, **kwargs):
        tuner.last_segment = partial
        raise CandidateRejected('overshoot above 8 ticks')

    monkeypatch.setattr(tuner, 'segment', rejected)
    result = tuner.trial(joint, profile(tuner.original[joint], 32, 2, 96))
    assert not result['completed'] and result['interrupted_segment'] == partial
    assert {k: bus.read(k, joint) for k in tuner.original[joint]} == tuner.original[joint]
    assert not any(k in ('Torque_Enable', 'Lock') for k, _, _ in bus.writes)


def test_fault_in_another_joint_stops_tuning_selected_joint(sweep):
    tuner, bus = sweep
    bus.registers['elbow_flex']['Status'] = 32
    with pytest.raises(RuntimeError):
        tuner.configure('shoulder_pan', profile(tuner.original['shoulder_pan'], 32, 2, 96))
    assert not any(k in tuner.original['shoulder_pan'] for k, _, _ in bus.writes)
    assert not any(k == 'Goal_Position' and j == 'elbow_flex' for k, j, _ in bus.writes)


def test_voltage_warning_remains_nonfatal(sweep):
    tuner, bus = sweep
    bus.registers['shoulder_pan']['Status'] = 1
    tuner.configure('shoulder_pan', profile(tuner.original['shoulder_pan'], 32, 2, 96))
    assert not tuner.control.fault


def test_missing_crossing_remains_null(sweep):
    frames = [{'t': i / 100, 'position': 0} for i in range(121)]
    result = summarize([{'kind': 'step', 'frames': frames, 'metrics': {
        'steady_peak_error_ticks': 20, 'steady_rms_error_ticks': 20,
        'overshoot_ticks': 0, 'time_to_90_s': None, 'max_sample_gap_s': .01}}])
    assert result['t90_ms'] is None and not result['acceptable']


def test_refinement_anchor_does_not_recenter_or_expand_envelope(sweep):
    tuner, bus = sweep
    anchor = {'centers': tuner.centers.copy(), 'identities': tuner.identities.copy()}
    bus.registers['shoulder_pan']['Present_Position'] += 10
    refined = PoseSweep(tuner.control, anchor=anchor)
    assert refined.centers == anchor['centers']
    assert refined.control.bounds['shoulder_pan'] == [2010, 2124]
    bus.registers['shoulder_pan']['Present_Position'] += 60
    with pytest.raises(ValueError, match='clearance'):
        PoseSweep(tuner.control, anchor=anchor)
    assert bus.writes == []


def test_single_corrupt_sample_is_retried_once_without_goal_writes(sweep, monkeypatch):
    tuner, bus = sweep
    original_tick = tuner.control.tick
    calls = []

    def flaky():
        calls.append(1)
        if len(calls) == 1:
            raise ConnectionError('corrupt packet')
        return original_tick()

    monkeypatch.setattr(tuner.control, 'tick', flaky)
    tuner.sample()
    assert len(calls) == 2 and tuner.sample_retries == 1
    assert bus.writes == []


def test_repeated_corrupt_samples_fail_without_unbounded_retries(sweep, monkeypatch):
    tuner, bus = sweep
    calls = []

    def broken():
        calls.append(1)
        raise ConnectionError('corrupt packet')

    monkeypatch.setattr(tuner.control, 'tick', broken)
    with pytest.raises(ConnectionError):
        tuner.sample()
    assert len(calls) == 2 and bus.writes == []


def test_finalists_are_distinct_and_use_worst_observed_repeat():
    from lerobot_arm_control.diagnostics.near_pose_pid import select_finalists

    def trial(p, score, completed=True):
        return {'settings': {'P_Coefficient': p}, 'completed': completed,
                'summary': {'score': score}}

    trials = [trial(32, 1), trial(32, 50), trial(40, 3), trial(48, 4),
              trial(64, 0), trial(64, 0, False)]
    selected = select_finalists(trials)
    assert [t['settings']['P_Coefficient'] for t in selected] == [40, 48, 32]
    assert selected[-1]['summary']['score'] == 50
