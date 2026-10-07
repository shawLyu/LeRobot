import pytest

from arm_control.joint_precision import precision_metrics


def test_settled_metrics_exclude_motion_and_detect_bad_visit():
    samples = [dict(visit=0, reference_raw=1600, raw=1550, settled_window=False)]
    samples += [dict(visit=visit, reference_raw=1600, raw=raw, settled_window=True)
                for visit, raw in [(1, 1599), (1, 1601), (2, 1598), (2, 1598)]]
    metrics = precision_metrics(samples)
    assert metrics['settled_samples'] == 4
    assert metrics['max_abs_error_ticks'] == 2
    assert metrics['repeat_visit_median_range_ticks']['1600'] == 2
    assert metrics['target_met_in_this_trial']
    samples.append(dict(visit=3, reference_raw=1600, raw=1590, settled_window=True))
    assert not precision_metrics(samples)['target_met_in_this_trial']


def test_no_settled_window_cannot_claim_precision():
    with pytest.raises(ValueError, match='No settled'):
        precision_metrics([])
