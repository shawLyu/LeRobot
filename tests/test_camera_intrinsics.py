"""A camera candidate must use varied board views and separate validation images."""

import json

import cv2
import numpy as np
import pytest

from lerobot_arm_control.calibration.camera_intrinsics import fit_capture


def test_unverified_focus_cannot_validate_intrinsics(tmp_path):
    (tmp_path/'calibration.json').write_text(json.dumps({'camera_controls': None}))
    with pytest.raises(ValueError, match='fixed camera focus'):
        fit_capture(tmp_path, 30)


def test_known_camera_is_recovered_from_independent_perspective_views(tmp_path):
    (tmp_path/'native').mkdir()
    (tmp_path/'calibration.json').write_text(json.dumps({
        'device': 'synthetic', 'camera_controls': {'autofocus': False, 'focus': 68,
                                                'verified': True}}))
    board = cv2.aruco.CharucoBoard((5, 7), .03, .022,
                                  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
    pattern = board.generateImage((1500, 2100), marginSize=0)
    source = np.array([[0, 0], [1499, 0], [1499, 2099], [0, 2099]], np.float32)
    objects = np.array([[0, 0, 0], [.15, 0, 0], [.15, .21, 0], [0, .21, 0]], np.float32)
    matrix = np.array([[1200., 0, 960], [0, 1190., 540], [0, 0, 1]])
    frames = []
    for i in range(27):
        rotation = np.array([(-.4, 0, .4)[i % 3], (-.35, .1, .35)[i//3 % 3],
                             (-.15, .1, .2)[i//9 % 3]])
        center = np.array([(600, 960, 1320)[i % 3], (320, 540, 760)[i//3 % 3]])
        translation = np.array([(center[0]-960)*.5/1200,
                                (center[1]-540)*.5/1190, .5])
        translation -= cv2.Rodrigues(rotation)[0]@np.array([.075, .105, 0])
        corners = cv2.projectPoints(objects, rotation, translation, matrix, np.zeros(5))[0]
        transform = cv2.getPerspectiveTransform(source, corners.reshape(4, 2))
        image = cv2.warpPerspective(pattern, transform, (1920, 1080), borderValue=255)
        name = f'native/{i:03d}.png'
        cv2.imwrite(str(tmp_path/name), image)
        frames.append({'sequence': i, 'native_frame': name})
    (tmp_path/'states.jsonl').write_text('\n'.join(json.dumps(s) for s in frames))
    result = fit_capture(tmp_path, 30)
    assert result['validation']['passed'], result['validation']
    assert len(result['held_out_sequences']) >= 4
    np.testing.assert_allclose(np.diag(result['camera_matrix'])[:2], [1200, 1190], rtol=.01)
    assert not result['activated']
