"""Independent synthetic poses check table registration and held-out layout changes."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from lerobot_arm_control.calibration.table_reference_calibration import fit_capture
from lerobot_arm_control.perception.tabletop_perception import read_config

ROOT = Path(__file__).resolve().parents[1]


def capture(tmp_path, *, moved_validation=False):
    config = read_config(ROOT/'configs/tabletop.json')
    config['dictionary'] = 'DICT_APRILTAG_36h11'
    config['marker_id'] = 3
    config['additional_markers'] = [{'dictionary': 'DICT_APRILTAG_16h5', 'id': 0}]
    metadata = {'config': config, 'robot': None, 'device': 'synthetic-camera',
                'camera_controls': {'verified': True, 'autofocus': False, 'focus': 68}}
    (tmp_path/'calibration.json').write_text(json.dumps(metadata))
    half = config['marker_size_m']/2
    primary = np.array([[-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0]], float)
    center = np.array([-.34, .02, 0.])
    yaw = 1.5
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    auxiliary = primary@rotation.T+center
    rows = []
    for index in range(30):
        aux = auxiliary.copy()
        if moved_validation and index % 5 == 4:
            aux[:, 0] += .03
        markers = []
        for dictionary, marker_id, points in [('DICT_APRILTAG_36h11', 3, primary),
                                              ('DICT_APRILTAG_16h5', 0, aux)]:
            pixels = cv2.projectPoints(points, np.array([2.8, .1, .2]),
                                       np.array([.15+index*.00001, 0., .65]),
                                       np.asarray(config['camera_matrix']),
                                       np.asarray(config['dist_coeffs']))[0].reshape(4, 2)
            markers.append({'dictionary': dictionary, 'id': marker_id,
                            'corners': pixels.tolist()})
        rows.append({'sequence': index+1, 'captured_unix_ns': index+1,
                     'detected_markers': markers})
    (tmp_path/'states.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    return config, center, yaw


def test_two_marker_fit_recovers_known_layout_on_independent_held_out_frames(tmp_path):
    config, center, yaw = capture(tmp_path)
    result = fit_capture(tmp_path, config)
    np.testing.assert_allclose(result['auxiliary']['center_m'], center, atol=1e-8)
    assert result['auxiliary']['yaw_rad'] == pytest.approx(yaw, abs=1e-8)
    assert result['quality']['training_frames'] == 24
    assert result['quality']['held_out_frames'] == 6
    assert max(result['quality']['held_out_native_rms_px']) < 1e-5
    assert result['physical_precision_validated'] is False


def test_auxiliary_movement_on_held_out_frames_cannot_be_accepted(tmp_path):
    config, _, _ = capture(tmp_path, moved_validation=True)
    with pytest.raises(ValueError, match='Held-out table reprojection'):
        fit_capture(tmp_path, config)


def test_changed_configuration_cannot_relabel_old_observations(tmp_path):
    config, _, _ = capture(tmp_path)
    config['marker_id'] = 1
    with pytest.raises(ValueError, match='active vision configuration'):
        fit_capture(tmp_path, config)


def test_small_primary_corner_noise_does_not_become_false_auxiliary_motion(tmp_path):
    config, center, yaw = capture(tmp_path)
    path = tmp_path/'states.jsonl'
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rng = np.random.default_rng(17)
    for row in rows:
        corners = np.asarray(row['detected_markers'][0]['corners'])
        row['detected_markers'][0]['corners'] = (corners+rng.normal(0, .08, (4, 2))).tolist()
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result = fit_capture(tmp_path, config)
    np.testing.assert_allclose(result['auxiliary']['center_m'], center, atol=.001)
    assert result['auxiliary']['yaw_rad'] == pytest.approx(yaw, abs=.01)
    assert result['quality']['passed'] is True
    assert max(result['quality']['held_out_native_rms_px']) < 2


def test_moving_camera_cannot_pass_stationary_table_calibration(tmp_path):
    config, _, _ = capture(tmp_path)
    path = tmp_path/'states.jsonl'
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    for index, row in enumerate(rows):
        for marker in row['detected_markers']:
            marker['corners'] = (np.asarray(marker['corners'])+[index*.2, 0]).tolist()
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError, match='Camera moved'):
        fit_capture(tmp_path, config)


def test_duplicate_capture_timestamps_are_rejected(tmp_path):
    config, _, _ = capture(tmp_path)
    path = tmp_path/'states.jsonl'
    rows = path.read_text().splitlines()
    path.write_text(rows[0]+'\n'+rows[0]+'\n')
    with pytest.raises(ValueError, match='distinct, ordered'):
        fit_capture(tmp_path, config)
