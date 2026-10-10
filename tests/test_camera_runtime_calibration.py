"""The selected camera record must govern projection and fixed-base validity."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from lerobot_arm_control.perception.tabletop_perception import read_config

ROOT = Path(__file__).resolve().parents[1]


def test_native_and_processing_views_have_identical_distorted_camera_rays():
    config = read_config(ROOT/'configs/tabletop.json')
    source = json.loads((ROOT/config['camera_calibration']['file']).read_text())
    native = np.array([[100., 100.], [1700., 800.], [900., 600.], [400., 950.]])
    low = cv2.transform(native.reshape(-1, 1, 2), cv2.invertAffineTransform(
        np.asarray(config['low_to_capture_affine']))).reshape(-1, 1, 2)
    distortion = np.asarray(source['dist_coeffs'])
    expected = cv2.undistortPoints(native.reshape(-1, 1, 2),
                                  np.asarray(source['camera_matrix']), distortion)
    actual = cv2.undistortPoints(low, np.asarray(config['camera_matrix']), distortion)
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)


@pytest.mark.parametrize('change', ['source', 'matrix', 'distortion', 'size'])
def test_configuration_rejects_mixed_or_modified_camera_calibration(tmp_path, change):
    config = read_config(ROOT/'configs/tabletop.json')
    source = tmp_path/'intrinsics.json'
    source.write_bytes((ROOT/config['camera_calibration']['file']).read_bytes())
    config['camera_calibration']['file'] = str(source)
    if change == 'source':
        source.write_bytes(source.read_bytes()+b'\n')
    elif change == 'matrix':
        config['camera_matrix'][0][0] += 1
    elif change == 'distortion':
        config['dist_coeffs'][0] += .001
    else:
        config['capture_size'][0] += 1
    path = tmp_path/'tabletop.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='Camera calibration|Runtime camera'):
        read_config(path)
