import numpy as np
import pytest

from lerobot_arm_control.diagnostics.follow_latency import analyze, estimate_lag


@pytest.mark.parametrize('delay', [.04, .08, .25])
def test_estimate_recovers_known_delay_despite_static_error_and_gain(delay):
    t = np.arange(0, 5, 1/60)
    def signal(x):
        return .2*np.sin(x*5)+.04*np.sin(x*9)
    result = estimate_lag(t, signal(t), t+.004, 1.1*signal(t+.004-delay)+.03)
    assert result['valid']
    assert result['lag_ms'] == pytest.approx(delay*1000, abs=3)


def test_static_and_straight_ramp_cannot_claim_zero_latency():
    t = np.arange(0, 5, 1/60)
    assert estimate_lag(t, t*0, t, t*0) is None
    assert estimate_lag(t, t*.1, t, (t-.2)*.1) is None


def synthetic_trace(delay):
    t = np.arange(0, 10, 1/60)
    trace = {'control_config_sha256': 'test', 'metadata': {'follow_config': {
        'leader_trigger_degrees': [0, 118], 'follower_gripper_degrees': [0, 90]}},
        'frames': []}
    for time in t:
        leader = [.3+.15*np.sin(time*5)]*6
        actual = [.3+.15*np.sin((time-delay)*5)]*6
        actual[5] *= 90/118
        reference = leader[:5]+[leader[5]*90/118]
        trace['frames'].append({'leader_time': time, 'follower_time': time,
                                'leader': leader, 'actual': actual, 'reference': reference,
                                'stage': 'following', 'sensor_to_command_ms': 5})
    return trace


def test_acceptance_includes_sampling_uncertainty_and_all_six_joints():
    assert analyze(synthetic_trace(.04))['passed']
    assert not analyze(synthetic_trace(.09))['passed']  # 90 + 16.7 ms exceeds 100.
    trace = synthetic_trace(.04)
    for frame in trace['frames']:
        frame['leader'][5] = .3
    assert not analyze(trace)['passed']
    for frame in trace['frames']:
        frame['stage'] = 'held'
    assert not analyze(trace)['passed']


def test_guided_capture_requires_the_labelled_joint_and_explicit_subset():
    trace = synthetic_trace(.04)
    for frame in trace['frames']:
        frame['measurement_joint'] = 'gripper'
    report = analyze(trace, required_joints=['gripper'])
    assert report['passed']
    assert report['required_joints'] == ['gripper']
    assert report['joints']['shoulder_pan']['valid_windows'] == 0
    assert not analyze(trace)['passed']


def test_interrupted_guided_capture_cannot_pass_on_partial_good_windows():
    trace = synthetic_trace(.04)
    trace['metadata']['measurement_completed'] = False
    report = analyze(trace)
    assert all(joint['passed'] for joint in report['joints'].values())
    assert report['capture_completed'] is False
    assert not report['passed']
    trace['metadata']['measurement_completed'] = True
    assert analyze(trace)['passed']


def test_cli_reads_archived_trace_and_exits_nonzero_for_failed_acceptance(tmp_path, monkeypatch):
    import gzip
    import json

    from lerobot_arm_control.diagnostics.follow_latency import main

    source, report = tmp_path/'trace.json.gz', tmp_path/'report.json'
    with gzip.open(source, 'wt') as stream:
        json.dump(synthetic_trace(.25), stream)
    monkeypatch.setattr('sys.argv', ['follow-latency', str(source), '--output', str(report)])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
    assert not json.loads(report.read_text())['passed']
