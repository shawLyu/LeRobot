import cv2
import numpy as np
import pytest

from lerobot_arm_control.perception.tool_marker import ToolMarker


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
    from lerobot_arm_control.perception.tabletop_perception import Perception

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


def test_marker_pixel_scale_is_not_reported_as_validated_precision():
    tracker, table = setup_marker()
    expected_corners = np.array([[[10., 20.], [42., 20.], [42., 44.], [10., 44.]]])

    class Detector:
        def detectMarkers(self, image):
            return [expected_corners], np.array([[8]]), []

    tracker.detector = Detector()
    result = tracker.process(np.zeros((480, 640, 3), np.uint8),
                             {**table, 'marker_visible': False})
    assert result['source_short_edge_px'] == 24
    assert result['source_pixels_per_cell'] == 4
    assert result['source_image_size'] == [640, 480]
    assert not result['precision_validated']
    assert not result['position_valid']


@pytest.mark.parametrize('constant', [float('nan'), -1, 21, 'small'])
def test_invalid_marker_threshold_constant_is_rejected(constant):
    with pytest.raises(ValueError, match='threshold'):
        ToolMarker({'dictionary': 'DICT_4X4_50', 'id': 8, 'marker_size_m': .020,
                    'adaptive_thresh_constant': constant}, np.eye(3), np.zeros(5))


@pytest.mark.parametrize('validated', [False, True])
def test_contact_reference_uses_marker_axes_and_preserves_mount_validation(validated):
    tracker, table = setup_marker()
    config = {**tracker.config, 'tool_reference': {
        'name': 'fixed_jaw_contact_center', 'offset_marker_m': [-.087, .000218, -.0262],
        'transform_validated': validated}}
    tracker = ToolMarker(config, tracker.matrix, tracker.distortion)
    rvec, translation = np.array([2.8, .12, -.1]), np.array([.08, .1, .62])
    corners = cv2.projectPoints(tracker.points, rvec, translation, tracker.matrix,
                               tracker.distortion)[0]
    result = tracker.estimate(corners, table)
    reference = result['tool_reference']
    expected = (np.array(table['camera_position'])+np.array(table['camera_rotation']).T
                @(translation+cv2.Rodrigues(rvec)[0]@config['tool_reference']['offset_marker_m']))
    np.testing.assert_allclose(reference['position' if validated else 'nominal_position'],
                               expected, atol=1e-8)
    assert reference['position_valid'] is validated
    assert not reference['precision_validated']
    if not validated:
        assert 'position' not in reference


def test_ambiguous_marker_orientation_withholds_contact_position():
    tracker, table = setup_marker()
    config = {**tracker.config, 'marker_size_m': .020, 'tool_reference': {
        'name': 'fixed_jaw_contact_center', 'offset_marker_m': [-.087, .000218, -.0262],
        'transform_validated': True}}
    tracker = ToolMarker(config, tracker.matrix, tracker.distortion)
    corners = cv2.projectPoints(tracker.points, np.array([2.9, .12, -.1]),
                               np.array([.08, .1, 2.]), tracker.matrix, tracker.distortion)[0]
    result = tracker.estimate(corners, table)
    assert not result['orientation_valid']
    reference = result['tool_reference']
    assert reference['status'] == 'pose_ambiguous'
    assert not reference['position_valid']
    assert 'position' not in reference and 'nominal_position' not in reference


@pytest.mark.parametrize('offset', [[float('nan'), 0, 0], [0, 0], [.4, 0, 0]])
def test_invalid_contact_reference_offset_is_rejected(offset):
    tracker, _ = setup_marker()
    with pytest.raises(ValueError, match='three-dimensional'):
        ToolMarker({**tracker.config, 'tool_reference': {'offset_marker_m': offset}},
                   tracker.matrix, tracker.distortion)


def test_text_false_cannot_validate_a_tool_mount():
    tracker, _ = setup_marker()
    with pytest.raises(ValueError, match='boolean'):
        ToolMarker({**tracker.config, 'tool_reference': {
            'offset_marker_m': [0, 0, 0], 'transform_validated': 'false'}},
                   tracker.matrix, tracker.distortion)
