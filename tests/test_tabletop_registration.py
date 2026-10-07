"""Two noncoplanar base tags must preserve table coordinates and calibration provenance."""

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from arm_control.tabletop_registration import BaseRegistration  # noqa: E402


class Detector:
    def __init__(self, corners, ids):
        self.corners, self.ids = corners, ids

    def detectMarkers(self, frame):
        return self.corners, self.ids, []


def fixture():
    matrix = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1.0]])
    distortion = np.zeros(5)
    markers = [
        {
            "id": 4,
            "mount": "vertical",
            "dictionary": "DICT_ARUCO_MIP_36H12",
            "corners_base_m": [
                [-0.02, -0.01, 0.04],
                [-0.02, 0.01, 0.04],
                [-0.02, 0.01, 0.02],
                [-0.02, -0.01, 0.02],
            ],
        },
        {
            "id": 5,
            "mount": "horizontal",
            "dictionary": "DICT_ARUCO_MIP_36H12",
            "corners_base_m": [
                [0.01, 0.01, 0.01],
                [0.03, 0.01, 0.01],
                [0.03, -0.01, 0.01],
                [0.01, -0.01, 0.01],
            ],
        },
    ]
    registration = BaseRegistration({"marker_size_m": 0.02, "markers": markers}, matrix, distortion)
    table_rotation = np.diag([1.0, -1.0, -1.0])
    center = np.array([0.0, 0.0, 1.0])
    yaw = 0.3
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]]
    )
    position = np.array([0.2, 0.05, 0.0024])
    camera_rotation = table_rotation @ rotation
    translation = table_rotation @ (position - center)
    corners = [
        cv2.projectPoints(
            np.array(m["corners_base_m"]),
            cv2.Rodrigues(camera_rotation)[0],
            translation,
            matrix,
            distortion,
        )[0]
        .reshape(1, 4, 2)
        .astype(np.float32)
        for m in markers
    ]
    registration.detectors = {"DICT_ARUCO_MIP_36H12": Detector(corners, np.array([[4], [5]]))}
    table = {
        "marker_visible": True,
        "camera_rotation": table_rotation.tolist(),
        "camera_position": center.tolist(),
    }
    return registration, table, position


def test_noncoplanar_tags_recover_base_in_anchor_coordinates():
    registration, table, position = fixture()
    result = registration.process(np.zeros((480, 640, 3), np.uint8), table)
    assert result["status"] == "tracked"
    assert result["calibrated"] is True
    np.testing.assert_allclose(result["position"], position, atol=1e-6)
    np.testing.assert_allclose(
        result["quaternion"], [math.cos(0.15), 0, 0, math.sin(0.15)], atol=1e-5
    )


def test_table_loss_and_expired_marker_cache_are_not_fresh(monkeypatch):
    registration, table, _ = fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    monkeypatch.setattr("arm_control.tabletop_registration.time.monotonic", lambda: 10.0)
    assert registration.process(frame, table)["status"] == "tracked"
    registration.detectors = {}
    monkeypatch.setattr("arm_control.tabletop_registration.time.monotonic", lambda: 11.0)
    assert registration.process(frame, table)["status"] == "held"
    monkeypatch.setattr("arm_control.tabletop_registration.time.monotonic", lambda: 13.0)
    result = registration.process(frame, table)
    assert result["status"] == "markers_lost"
    assert "position" not in result
    table["marker_visible"] = False
    assert registration.process(frame, table)["status"] == "table_reference_lost"


def test_missing_mount_transform_never_becomes_calibrated():
    registration, table, _ = fixture()
    for marker in registration.config["markers"]:
        marker["corners_base_m"] = None
    result = registration.process(np.zeros((480, 640, 3), np.uint8), table)
    assert len(result["markers"]) == 2
    assert result["status"] == "mount_calibration_missing"
    assert result["calibrated"] is False
    assert "position" not in result


def test_measured_horizontal_mount_recovers_base_without_depth_or_tilt_ambiguity():
    registration, table, position = fixture()
    registration.config['markers'] = [registration.config['markers'][1]]
    registration.config.update(base_constraint='flat_on_table', base_table_z_m=.0024)
    result = registration.process(np.zeros((480, 640, 3), np.uint8), table)
    assert result['calibrated'] and result['constraint'] == 'base_flat_on_table'
    np.testing.assert_allclose(result['position'], position, atol=1e-6)
    np.testing.assert_allclose(result['quaternion'], [math.cos(.15), 0, 0, math.sin(.15)],
                               atol=1e-5)
    # Image corner noise cannot turn a resting base into a tilted or floating base.
    detector = registration.detectors['DICT_ARUCO_MIP_36H12']
    detector.corners[1] += np.array([[[.2, -.1], [-.1, .2], [.1, .2], [-.2, -.1]]],
                                  dtype=np.float32)
    noisy = registration.process(np.zeros((480, 640, 3), np.uint8), table)
    assert noisy['calibrated']
    assert noisy['position'][2] == pytest.approx(.0024, abs=1e-10)
    assert noisy['quaternion'][1:3] == pytest.approx([0, 0], abs=1e-10)


def test_vertical_marker_alone_recovers_flat_base_at_known_corner_heights():
    registration, table, position = fixture()
    registration.config['markers'] = [registration.config['markers'][0]]
    registration.config.update(base_constraint='flat_on_table', base_table_z_m=.0024)
    table_rotation = cv2.Rodrigues(np.array([2.6, .2, .1]))[0]
    table['camera_rotation'] = table_rotation.tolist()
    yaw = .3
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                         [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    obj = np.array(registration.config['markers'][0]['corners_base_m'])
    corners = cv2.projectPoints(obj, cv2.Rodrigues(table_rotation@rotation)[0],
                                table_rotation@(position-np.array(table['camera_position'])),
                                registration.matrix, registration.distortion)[0]
    registration.detectors = {'DICT_ARUCO_MIP_36H12': Detector(
        [corners.reshape(1, 4, 2).astype(np.float32)], np.array([[4]]))}
    result = registration.process(np.zeros((480, 640, 3), np.uint8), table)
    assert result['status'] == 'tracked' and result['calibrated']
    assert [m['id'] for m in result['markers']] == [4]
    np.testing.assert_allclose(result['position'], position, atol=1e-6)
    np.testing.assert_allclose(result['quaternion'], [math.cos(.15), 0, 0, math.sin(.15)],
                               atol=1e-5)


def test_small_marker_fallback_requires_two_decodes_and_never_calibrates_unknown_mount():
    registration, table, _ = fixture()
    marker = registration.config['markers'][0]
    marker['corners_base_m'] = None
    registration.config['markers'] = [marker]
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_MIP_36H12)
    image = np.full((200, 300), 255, np.uint8)
    image[60:96, 120:156] = cv2.aruco.generateImageMarker(dictionary, 4, 36)
    actual = cv2.aruco.ArucoDetector(dictionary)

    class MissOriginal:
        def detectMarkers(self, frame):
            if frame.shape == image.shape:
                return [], None, []
            return actual.detectMarkers(frame)

    registration.detectors = {'DICT_ARUCO_MIP_36H12': MissOriginal()}
    result = registration.process(image, table)
    assert [m['id'] for m in result['markers']] == [4]
    assert result['calibrated'] is False
    assert result['status'] == 'mount_calibration_missing'
    assert ('DICT_ARUCO_MIP_36H12', 4) in registration.marker_rois
    image[60:96, 120:156] = cv2.aruco.generateImageMarker(dictionary, 6, 36)
    assert registration.process(image, table)['markers'] == []


def test_selected_id4_does_not_use_id5_mount_as_a_hidden_dependency():
    registration, table, _ = fixture()
    registration.config['localization_marker_ids'] = [4]
    registration.config['markers'][0]['corners_base_m'] = None
    result = registration.process(np.zeros((480, 640, 3), np.uint8), table)
    assert [m['id'] for m in result['markers']] == [4]
    assert result['status'] == 'mount_calibration_missing'
    assert result['calibrated'] is False


def test_flat_pose_filter_resets_on_measured_base_motion():
    registration, table, position = fixture()
    registration.config['markers'] = [registration.config['markers'][1]]
    registration.config.update(base_constraint='flat_on_table', base_table_z_m=.0024)
    frame = np.zeros((480, 640, 3), np.uint8)
    assert registration.process(frame, table)['pose_filter_samples'] == 1
    assert registration.process(frame, table)['pose_filter_samples'] == 2
    detector = registration.detectors['DICT_ARUCO_MIP_36H12']
    detector.corners[1] += np.array([[[6., 0]]], dtype=np.float32)
    moved = registration.process(frame, table)
    assert moved['pose_filter_samples'] == 1
    np.testing.assert_allclose(moved['position'], position+[.009876, 0, 0], atol=2e-5)


def fixed_fixture(tmp_path):
    registration, table, position = fixture()
    registration.config.update(base_constraint='flat_on_table', base_table_z_m=.0024,
                               fixed_base={'file': str(tmp_path/'base.json'), 'sample_count': 20,
                                           'reference': {'id': 1}, 'model_sha256': 'test-model'})
    instance = BaseRegistration(registration.config, registration.matrix, registration.distortion)
    instance.detectors = registration.detectors
    table.update(anchor_visible=True, captured_unix_ns=1)
    return instance, table, position


def test_fixed_base_survives_occlusion_restart_and_clock_age(tmp_path, monkeypatch):
    registration, table, position = fixed_fixture(tmp_path)
    frame = np.zeros((480, 640, 3), np.uint8)
    for index in range(20):
        monkeypatch.setattr('arm_control.tabletop_registration.time.monotonic',
                            lambda i=index: 10+i*.11)
        table['captured_unix_ns'] = index+1
        result = registration.process(frame, table)
    assert result['status'] == 'fixed' and result['fixed_to_table']
    np.testing.assert_allclose(result['position'], position, atol=1e-6)

    class NeverDetect:
        def detectMarkers(self, image):
            pytest.fail('fixed base must not run marker detection')

    registration.detectors = {'DICT_ARUCO_MIP_36H12': NeverDetect()}
    table['marker_visible'] = False
    assert registration.process(frame, table)['position'] == result['position']
    restored = BaseRegistration(registration.config, registration.matrix, registration.distortion)
    restored.detectors = registration.detectors
    assert restored.process(frame, table)['quaternion'] == result['quaternion']
    restored.config['markers'][0]['corners_base_m'][0][0] += .005
    changed = BaseRegistration(restored.config, restored.matrix, restored.distortion)
    assert changed.fixed_pose is None


def test_fixed_calibration_needs_distinct_frames_and_primary_anchor(tmp_path, monkeypatch):
    registration, table, _ = fixed_fixture(tmp_path)
    frame = np.zeros((480, 640, 3), np.uint8)
    for index in range(25):
        monkeypatch.setattr('arm_control.tabletop_registration.time.monotonic',
                            lambda i=index: 10+i*.11)
        registration.process(frame, table)
    assert registration.fixed_pose is None and len(registration.fixed_samples) == 1
    table.update(anchor_visible=False, captured_unix_ns=2)
    registration.process(frame, table)
    assert registration.fixed_samples == []
    assert not registration.fixed_path.exists()


def test_connected_tag_border_is_decoded_from_real_lines_without_filling_code_pixels():
    registration, _, _ = fixture()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_MIP_36H12)
    image = np.full((240, 300), 255, np.uint8)
    image[80:160, 100:180] = cv2.aruco.generateImageMarker(dictionary, 6, 80)
    image[70:120, 65:101] = 0
    marker = {'dictionary': 'DICT_ARUCO_MIP_36H12', 'id': 6,
              'border_seed_native': [[100, 80], [179, 80], [179, 159], [100, 159]]}
    corner = registration.detect_border(marker, image)
    assert corner is not None
    assert cv2.contourArea(corner) > 6000
    marker['id'] = 5
    assert registration.detect_border(marker, image) is None


def test_two_arm_registration_keeps_separate_frames_and_distances(tmp_path, monkeypatch):
    from arm_control.tabletop_registration import base_distances, base_registration_configs

    matrix = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    distortion = np.zeros(5)
    table_rotation = cv2.Rodrigues(np.array([2.6, .2, .1]))[0]
    camera = np.array([0., 0., 1.])
    table = {'marker_visible': True, 'anchor_visible': True,
             'camera_rotation': table_rotation.tolist(), 'camera_position': camera.tolist()}
    entries, image_corners, positions = [], [], {}
    for name, marker_id, xy, offsets in (
        ('follower', 7, [-.2, 0], [(1, -1), (-1, -1), (-1, 1), (1, 1)]),
        ('arm1', 6, [.1, .02], [(-1, 1), (1, 1), (1, -1), (-1, -1)]),
    ):
        points = [[-.002865+x*.021, y*.021, .0666] for x, y in offsets]
        position = np.array([*xy, .0024])
        positions[name] = position
        projected = cv2.projectPoints(np.array(points), cv2.Rodrigues(table_rotation)[0],
                                     table_rotation@(position-camera), matrix, distortion)[0]
        image_corners.append(projected.reshape(1, 4, 2).astype(np.float32))
        entries.append({'enabled': True, 'arm': name, 'marker_size_m': .042,
                        'markers': [{'dictionary': 'DICT_4X4_50', 'id': marker_id,
                                     'mount': 'base_parallel_to_table', 'corners_base_m': points}],
                        'base_constraint': 'flat_on_table', 'base_table_z_m': .0024,
                        'fixed_base': {'file': str(tmp_path/(name+'.json')), 'sample_count': 20,
                                       'reference': {'id': 1}, 'model_sha256': 'same-model'}})
    config = {'robot': entries[0], 'additional_robots': entries[1:]}
    group = {entry['arm']: BaseRegistration(entry, matrix, distortion)
             for entry in base_registration_configs(config)}
    for registration in group.values():
        registration.detectors = {'DICT_4X4_50': Detector(image_corners, np.array([[7], [6]]))}
    frame = np.zeros((480, 640, 3), np.uint8)
    for index in range(20):
        monkeypatch.setattr('arm_control.tabletop_registration.time.monotonic',
                            lambda i=index: 10+i*.11)
        table['captured_unix_ns'] = index+1
        poses = {name: registration.process(frame, table) for name, registration in group.items()}
    for name, pose in poses.items():
        assert pose['status'] == 'fixed'
        np.testing.assert_allclose(pose['position'], positions[name], atol=1e-6)
        assert (tmp_path/(name+'.json')).exists()
    measured = base_distances(poses)[0]
    assert measured['arms'] == ['follower', 'arm1'] and measured['both_fixed']
    assert measured['planar_distance_m'] == pytest.approx(math.hypot(.3, .02), abs=1e-6)
    poses['arm1']['status'] = 'held'
    assert base_distances(poses) == []


@pytest.mark.parametrize('duplicate', ['arm', 'file', 'marker'])
def test_two_arms_reject_shared_identity_or_calibration_file(duplicate):
    import copy

    from arm_control.tabletop_registration import base_registration_configs

    first = {'enabled': True, 'arm': 'follower', 'markers': [
        {'dictionary': 'DICT_4X4_50', 'id': 7}], 'fixed_base': {'file': 'follower.json'}}
    second = copy.deepcopy(first)
    second.update(arm='arm1', fixed_base={'file': 'arm1.json'},
                  markers=[{'dictionary': 'DICT_4X4_50', 'id': 6}])
    if duplicate == 'arm':
        second['arm'] = first['arm']
    elif duplicate == 'file':
        second['fixed_base'] = first['fixed_base']
    else:
        second['markers'] = first['markers']
    with pytest.raises(ValueError, match='distinct'):
        base_registration_configs({'robot': first, 'additional_robots': [second]})


def test_observed_border_overrides_oversized_dark_background_detection(monkeypatch):
    registration, _, _ = fixture()
    marker = registration.config['markers'][0]
    marker.update(prefer_observed_border=True, border_seed_native=[[0, 0]]*4)
    true_border = np.array([[[20., 20.], [60., 20.], [60., 60.], [20., 60.]]], np.float32)
    monkeypatch.setattr(registration, 'detect_border', lambda *args: true_border)
    detector = registration.detectors[marker['dictionary']]
    found = registration.detect(detector, marker['dictionary'], np.zeros((100, 100), np.uint8))
    np.testing.assert_array_equal(found[marker['id']], true_border)
    monkeypatch.setattr(registration, 'detect_border', lambda *args: None)
    # A failed exact border/code check cannot revive the oversized detection.
    found = registration.detect(detector, marker['dictionary'], np.zeros((100, 100), np.uint8))
    assert marker['id'] not in found
