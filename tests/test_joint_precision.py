import pytest

from lerobot_arm_control.diagnostics.joint_precision import marker_metrics, precision_metrics


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


def test_image_repeatability_excludes_motion_and_duplicate_camera_frames():
    rows = [dict(visit=visit, reference_raw=1600, image_center_px=center,
                 settled_window=settled, captured_unix_ns=time)
            for visit, center, settled, time in [
                (None, [50, 50], False, 1), (0, [1, 2], True, 2),
                (0, [99, 99], True, 2), (0, [1, 2], True, 3),
                (1, [3, 2], True, 4), (1, [3, 2], True, 5),
                (1, [100, 100], False, 6)]]
    result = marker_metrics(rows, [0, 1])
    assert result['all_visits_observed']
    assert [visit['samples'] for visit in result['visits']] == [2, 2]
    assert result['max_pairwise_visit_median_distance_px']['1600'] == 2
    assert not result['precision_validated']
    assert not marker_metrics(rows, [0, 1, 2])['all_visits_observed']


def test_one_observed_visit_cannot_claim_zero_repeatability_error():
    rows = [dict(visit=0, reference_raw=1600, image_center_px=[1, 2],
                 settled_window=True, captured_unix_ns=time) for time in [1, 2]]
    result = marker_metrics(rows, [0, 1])
    assert not result['all_visits_observed']
    assert result['max_pairwise_visit_median_distance_px']['1600'] is None
