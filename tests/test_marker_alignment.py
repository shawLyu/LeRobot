"""Independent held-out errors and observability gate local camera calibration."""

import json
import math
import shutil
import sys

import numpy as np
import pytest

pytest.importorskip('mujoco')

from lerobot_arm_control.calibration.marker_alignment import (  # noqa: E402
    ROOT,
    MarkerKinematics,
    digest,
    fit,
    read_observations,
)
from lerobot_arm_control.robot.so101.model_metadata import MODEL  # noqa: E402
from lerobot_arm_control.transport.realtime import JOINTS  # noqa: E402


@pytest.fixture(autouse=True)
def isolated_calibration(tmp_path, monkeypatch):
    """Synthetic bases keep fitting tests independent of physical registration status."""
    for relative in ('configs/tabletop.json', 'data/calibration/follower_mujoco.json'):
        destination = tmp_path/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/relative, destination)
    (tmp_path/'data/calibration/follower_table_base.json').write_text(json.dumps({
        'pose': {'position': [.1, .2, 0.], 'quaternion': [1., 0., 0., 0.]}}))
    monkeypatch.setattr('lerobot_arm_control.calibration.marker_alignment.ROOT', tmp_path)
    monkeypatch.setattr(sys.modules[__name__], 'ROOT', tmp_path)


def observations():
    model = MarkerKinematics()
    rng = np.random.default_rng(183)
    true = np.radians([1., -2., 1.5, -.7, .5])
    poses = []
    for i in range(24):
        values = [2143+rng.integers(-250, 250), rng.integers(1100, 1600),
                  rng.integers(2200, 2900), rng.integers(2100, 2700),
                  rng.integers(1700, 2200), 1800]
        raw = dict(zip(JOINTS, map(int, values), strict=True))
        poses.append({'name': str(i), 'raw': raw, 'target_raw': raw.copy(),
                      'measured_position_m': model(raw, true).tolist()})
    return poses, true


def test_varied_poses_recover_offsets_and_unseen_positions():
    poses, true = observations()
    result = fit(poses, [3, 9, 15, 21])
    assert result['validation']['passed'], result['validation']
    np.testing.assert_allclose(list(result['offsets_rad'].values())[:5], true, atol=1e-6)
    assert result['validation']['held_out_max_m'] < 1e-7
    assert not result['activated']


def test_held_out_measurements_do_not_modify_training_fit():
    poses, _ = observations()
    poses[3]['measured_position_m'][2] += .002
    result = fit(poses, [3, 9, 15, 21])
    assert not result['validation']['passed']
    assert result['validation']['training_rmse_m'] < 1e-7
    assert result['validation']['held_out_max_m'] > .0019


def test_repeated_pose_cannot_validate_five_joint_alignment():
    poses, _ = observations()
    repeated = [poses[0]]*16
    result = fit(repeated, [3, 7, 11])
    assert not result['validation']['passed']
    assert result['validation']['jacobian_condition'] > 100
    assert all(abs(v) <= math.radians(5) for v in result['offsets_rad'].values())


def test_failed_or_stale_capture_cannot_be_rebound_to_current_geometry(tmp_path):
    path = tmp_path/'result.json'
    path.write_text(json.dumps({'completed': False}))
    with pytest.raises(ValueError, match='Incomplete'):
        read_observations(tmp_path)
    recorded = {'angles': digest(ROOT/'data/calibration/follower_mujoco.json'),
                'base': digest(ROOT/'data/calibration/follower_table_base.json'),
                'model': digest(MODEL)}
    path.write_text(json.dumps({'completed': True, 'input_sha256': recorded,
                               'cleanup_error': 'hold failed'}))
    with pytest.raises(ValueError, match='Incomplete'):
        read_observations(tmp_path)
    recorded['base'] = 'old-base'
    path.write_text(json.dumps({'completed': True, 'input_sha256': recorded}))
    with pytest.raises(ValueError, match='provenance'):
        read_observations(tmp_path)


def test_relative_capture_cannot_be_used_for_absolute_calibration(tmp_path):
    (tmp_path/'result.json').write_text(json.dumps({
        'completed': True, 'validation_scope': 'relative motion only; absolute base unvalidated'}))
    with pytest.raises(ValueError, match='Relative-only'):
        read_observations(tmp_path)
