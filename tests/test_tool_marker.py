import cv2
import numpy as np
import pytest

from arm_control.tool_marker import ToolMarker


def setup_marker():
    matrix = np.array([[530., 0, 320], [0, 530., 240], [0, 0, 1]])
    tracker = ToolMarker({'dictionary': 'DICT_4X4_50', 'id': 8, 'marker_size_m': .042},
                         matrix, np.zeros(5))
    rotation = cv2.Rodrigues(np.array([2.7, .15, .1]))[0]
    table = {'marker_visible': True, 'camera_position': [.1, -.2, .7],
             'camera_rotation': rotation.tolist(), 'captured_unix_ns': 42}
    return tracker, table


def test_marker_position_is_measured_without_any_robot_angles():
    tracker, table = setup_marker()
    translation = np.array([.08, .1, .62])
    rvec = np.array([2.8, .12, -.1])
    corners = cv2.projectPoints(tracker.points, rvec, translation, tracker.matrix,
                               tracker.distortion)[0]
    result = tracker.estimate(corners, table)
    assert result['position_valid'] and result['orientation_valid']
    expected = np.array(table['camera_position'])+np.array(table['camera_rotation']).T@translation
    np.testing.assert_allclose(result['position'], expected, atol=1e-8)
    assert result['reprojection_error_px'] < 1e-7
    assert result['captured_unix_ns'] == 42


def test_tool_marker_rejects_lost_table_and_bad_geometry():
    tracker, table = setup_marker()
    corners = cv2.projectPoints(tracker.points, np.array([2.8, .12, -.1]),
                               np.array([.08, .1, .62]), tracker.matrix,
                               tracker.distortion)[0]
    assert not tracker.estimate(corners, {**table, 'marker_visible': False})['position_valid']
    corners[0, 0] += [12, 15]
    assert tracker.estimate(corners, table)['status'] == 'geometry_rejected'


def test_tool_marker_must_have_measured_size_and_distinct_identity():
    with pytest.raises(ValueError, match='measured'):
        ToolMarker({'marker_size_m': None}, np.eye(3), np.zeros(5))
    from arm_control.tabletop_perception import Perception

    config = {'camera_matrix': np.eye(3).tolist(), 'dist_coeffs': [0]*5,
              'marker_size_m': .15, 'wood_height_m': 0, 'dictionary': 'DICT_4X4_50',
              'marker_id': 7, 'robot': {'enabled': False, 'tool_marker': {
                  'enabled': True, 'dictionary': 'DICT_4X4_50', 'id': 7,
                  'marker_size_m': .042}}}
    with pytest.raises(ValueError, match='stationary'):
        Perception(config)


def test_duplicate_tool_marker_cannot_produce_a_measurement():
    tracker, table = setup_marker()

    class DuplicateDetector:
        def detectMarkers(self, image):
            return [np.zeros((1, 4, 2))]*2, np.array([[8], [8]]), []

    tracker.detector = DuplicateDetector()
    result = tracker.process(np.zeros((480, 640, 3), np.uint8), table)
    assert result['status'] == 'duplicate_marker'
    assert not result['position_valid'] and not result['orientation_valid']
