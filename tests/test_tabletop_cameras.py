"""Second views can rescue occlusion, never hide stale frames or camera disagreement."""

import copy
import json
from unittest.mock import Mock

import pytest

from lerobot_arm_control.runtime.tabletop_cameras import (
    CameraWorker,
    fuse_objects,
    refresh_observation,
)


def state(age=50, xy=None, yaw=.2):
    return {'marker_visible': True, 'units': 'm', 'calibrated': True,
            'captured_unix_ns': 1_000_000_000, 'capture_age_at_send_ms': age,
            'objects': {'wood_t': {'observed': True, 'pose_valid': True,
                'tracking_status': 'observed', 'xy': xy or [.1, .2], 'yaw': yaw}}}


def test_occluded_primary_uses_fresh_view_without_modifying_observer():
    primary, side = state(), state()
    primary['objects']['wood_t'].update(observed=False, pose_valid=False,
                                        tracking_status='held')
    before = copy.deepcopy(side)
    fuse_objects(primary, {'side': side})
    assert primary['objects']['wood_t']['observation_source'] == 'side'
    assert primary['objects']['wood_t']['observation_age_s'] == .05
    assert primary['objects']['wood_t']['observed']
    assert side == before


@pytest.mark.parametrize('change', ['stale', 'not_calibrated', 'no_marker', 'held', 'invalid'])
def test_untrusted_view_cannot_rescue_missing_object(change):
    primary, side = state(), state()
    primary['objects'] = {}
    if change == 'stale':
        side['capture_age_at_send_ms'] = 501
    elif change == 'not_calibrated':
        side['calibrated'] = False
    elif change == 'no_marker':
        side['marker_visible'] = False
    elif change == 'held':
        side['objects']['wood_t']['tracking_status'] = 'held'
    else:
        side['objects']['wood_t']['pose_valid'] = False
    fuse_objects(primary, {'side': side})
    assert 'wood_t' not in primary['objects']


@pytest.mark.parametrize('xy,yaw', [([.11, .2], .2), ([.1, .2], .5)])
def test_synchronized_views_that_disagree_invalidate_observation(xy, yaw):
    primary = state()
    fuse_objects(primary, {'side': state(xy=xy, yaw=yaw)})
    obj = primary['objects']['wood_t']
    assert not obj['pose_valid'] and not obj['observed']
    assert obj['tracking_status'] == 'camera_disagreement'


def test_provisional_disagreement_keeps_trusted_primary_and_records_rejection():
    primary, side = state(), state(xy=[.12, .2])
    side['intrinsics_quality'] = 'provisional'
    fuse_objects(primary, {'side': side})
    assert primary['objects']['wood_t']['pose_valid']
    assert primary['objects']['wood_t']['xy'] == [.1, .2]
    assert primary['camera_disagreements']['wood_t']['rejected_source'] == 'side'


def test_recent_primary_measurement_retains_capture_time_and_expires():
    primary, side = state(age=100), state(xy=[.12, .2])
    obj = primary['objects']['wood_t']
    obj.update(observed=False, tracking_status='held',
               observation_captured_unix_ns=primary['captured_unix_ns']-100_000_000)
    side['intrinsics_quality'] = 'provisional'
    fresh = fuse_objects(copy.deepcopy(primary), {'side': side})
    kept = fresh['objects']['wood_t']
    assert kept['tracking_status'] == 'recent' and not kept['observed']
    assert kept['observation_age_s'] == .2
    assert kept['observation_captured_unix_ns'] == obj['observation_captured_unix_ns']
    primary['capture_age_at_send_ms'] = 401
    expired = fuse_objects(primary, {'side': side})
    assert expired['objects']['wood_t']['observation_source'] == 'side'
    assert expired['objects']['wood_t']['xy'] == [.12, .2]


def test_time_skew_is_not_mistaken_for_simultaneous_disagreement():
    primary, side = state(), state(xy=[.2, .2])
    side['captured_unix_ns'] += 200_000_000
    fuse_objects(primary, {'side': side})
    assert primary['objects']['wood_t']['pose_valid']
    assert primary['observation_sources']['wood_t'] == 'primary'


def test_secondary_update_keeps_primary_and_motor_ages_and_original_evidence():
    primary, side = state(age=100), state(age=80)
    primary['objects']['wood_t'].update(observed=False, tracking_status='held')
    primary['robot'] = {'sample_age_at_send_ms': 25, 'registration': {'status': 'fixed'}}
    before = copy.deepcopy(primary)
    updated = refresh_observation(primary, 200, {'side': side})
    assert updated['capture_age_at_send_ms'] == 300
    assert updated['robot']['sample_age_at_send_ms'] == 225
    assert updated['objects']['wood_t']['observation_age_s'] == .08
    assert updated['objects']['wood_t']['observation_source'] == 'side'
    assert primary == before
    expired = refresh_observation(primary, 501, {'side': side})
    assert expired['capture_age_at_send_ms'] == 601
    assert not expired['marker_visible'] or expired['capture_age_at_send_ms'] > 500


def test_provisional_intrinsics_require_explicit_use_and_remain_labeled(tmp_path, monkeypatch):
    record = tmp_path/'intrinsics.json'
    record.write_text(json.dumps({'activated': True, 'quality': 'provisional',
        'camera_device': '/dev/video2', 'validation': {'passed': False}}))
    config = {'camera_calibration': {'file': str(record)}, 'robot': {'enabled': False},
              'image_size': [1280, 720], 'capture_size': [1280, 720]}
    module = 'lerobot_arm_control.runtime.tabletop_cameras.'
    monkeypatch.setattr(module+'read_config', lambda path: config)
    monkeypatch.setattr(module+'Perception', Mock())
    monkeypatch.setattr(module+'threading.Thread', Mock())
    camera = Mock()
    camera.isOpened.return_value = True
    monkeypatch.setattr(module+'cv2.VideoCapture', Mock(return_value=camera))
    entry = {'name': 'side', 'device': '/dev/video2', 'tabletop_config': 'side.json'}
    with pytest.raises(ValueError, match='explicit provisional'):
        CameraWorker(entry)
    observer = CameraWorker({**entry, 'allow_provisional': True})
    try:
        assert observer.snapshot({})['intrinsics_quality'] == 'provisional'
        assert json.loads(record.read_text())['validation']['passed'] is False
    finally:
        observer.close()
    config['robot']['enabled'] = True
    with pytest.raises(ValueError, match='cannot own robot'):
        CameraWorker({**entry, 'allow_provisional': True})
