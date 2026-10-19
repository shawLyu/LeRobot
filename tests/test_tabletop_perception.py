"""Geometry and loss/recovery tests; run on the remote OpenCV environment."""

import math
import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2", reason="Perception tests run on the remote OpenCV host")

from lerobot_arm_control.perception.tabletop_perception import (  # noqa: E402
    Perception,
    fit_t,
    pixels_on_plane,
    read_config,
)


def perception_config():
    config = read_config("configs/tabletop.json")
    config.update(dictionary="DICT_APRILTAG_36h10", marker_id=1,
                  additional_markers=[{"dictionary": "DICT_APRILTAG_16h5", "id": 0}],
                  image_size=[640, 480],
                  camera_matrix=[[537.6, 0., 306.7], [0., 537.4, 261.9], [0., 0., 1.]])
    return config


def test_static_map_restores_occluded_print_without_resetting_age(tmp_path):
    config = perception_config()
    config["robot"]["enabled"] = False
    perception = Perception(config)
    path = tmp_path / "static_map.json"
    pose = {"xy": [.1, .2], "yaw": .3, "iou": .9, "observed": True}
    perception.save_static_map(path, {"objects": {"black_t": pose},
                                     "captured_unix_ns": time.time_ns()-int(60e9)})
    restored = Perception(config)
    restored.restore_static_map(path)
    assert restored.trackers["wood_t"].last is None
    assert restored.trackers["black_t"].last["xy"] == [.1, .2]
    assert time.monotonic()-restored.trackers["black_t"].last_observed >= 60
    config["marker_id"] = 2
    other_reference = Perception(config)
    other_reference.restore_static_map(path)
    assert other_reference.trackers["black_t"].last is None



def test_saved_auxiliary_layout_is_rejected_after_camera_calibration_changes(tmp_path):
    config = perception_config()
    perception = Perception(config)
    perception.secondary_layouts = {'DICT_APRILTAG_16h5:0': (
        perception.tag_points+[.22, .08, 0.]).tolist()}
    path = tmp_path/'static_map.json'
    perception.save_static_map(path, {'objects': {}, 'captured_unix_ns': time.time_ns()})
    restored = Perception(config)
    restored.restore_static_map(path)
    assert restored.secondary_layouts
    config['camera_calibration']['sha256'] = 'different-native-calibration'
    changed = Perception(config)
    changed.restore_static_map(path)
    assert not changed.secondary_layouts


def test_pixel_ray_hits_known_plane_and_height():
    matrix = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
    rotation = np.diag([1., -1, -1])
    translation = np.array([0., 0, 1])
    points = pixels_on_plane([[320, 240], [370, 290]], matrix, np.zeros(5),
                             rotation, translation, .2)
    np.testing.assert_allclose(points, [[0, 0, .2], [.08, -.08, .2]], atol=1e-8)


def test_t_orientation_and_concave_dimensions():
    polygon = np.array([[-.15, .2], [.15, .2], [.15, .13], [.035, .13],
                        [.035, -.2], [-.035, -.2], [-.035, .13], [-.15, .13]])
    angle = .7
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    fitted = fit_t(polygon @ rotation.T + [1, 2])
    assert fitted["iou"] > .94
    assert fitted["yaw"] == pytest.approx(angle, abs=.025)
    np.testing.assert_allclose(fitted["xy"], [1, 2], atol=.006)
    assert fitted["width"] == pytest.approx(.3, abs=.008)
    assert fitted["length"] == pytest.approx(.4, abs=.008)
    assert fitted["stem_width"] < fitted["width"] / 2


def test_rectangle_is_not_a_t():
    fitted = fit_t(np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]))
    assert fitted is None or fitted["iou"] < .78


def tag_image(config, *, anchor=True, secondary=True):
    frame = np.full((480, 640, 3), 245, np.uint8)
    scale = config["marker_size_m"]
    half = scale / 2
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]])
    rvec, tvec = np.array([2.8, .15, .2]), np.array([-.13, .05, .7])
    for enabled, dictionary, tag_id, offset in (
        (anchor, "DICT_APRILTAG_36h10", 1, [0, 0, 0]),
        (secondary, "DICT_APRILTAG_16h5", 0, [.22, .08, 0]),
    ):
        if not enabled:
            continue
        tag = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)), tag_id, 256)
        projected = cv2.projectPoints(points + offset, rvec, tvec,
                                     np.array(config["camera_matrix"]),
                                     np.array(config["dist_coeffs"]))[0].reshape(-1, 2)
        transform = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [255, 0], [255, 255], [0, 255]]), projected.astype(np.float32))
        warped = cv2.warpPerspective(tag, transform, (640, 480), borderValue=245)
        frame = np.minimum(frame, warped[:, :, None])
    return frame, rvec, tvec


def test_second_dictionary_learns_layout_then_recovers_same_world():
    config = perception_config()
    config["marker_size_m"] = .1
    # Use a pinhole render; distortion is tested independently by the ray test.
    config["dist_coeffs"] = [0., 0., 0., 0., 0.]
    perception = Perception(config)
    frame, rvec, tvec = tag_image(config)
    first = perception.process(frame, 1, 0)
    assert first["marker_visible"] and first["anchor_visible"]
    assert len(first["detected_markers"]) == 2
    assert len(first["secondary_layouts"]) == 1
    frame, _, _ = tag_image(config, anchor=False)
    second = perception.process(frame, 2, 0)
    assert second["marker_visible"] and not second["anchor_visible"]
    expected = -cv2.Rodrigues(rvec)[0].T @ tvec
    np.testing.assert_allclose(second["camera_position"], expected, atol=.02)
    # The relevant error is tabletop XY, rather than camera orientation/depth ambiguity.
    pixel = cv2.projectPoints(np.array([[.1, 0., 0.]]), rvec, tvec,
                              np.array(config["camera_matrix"]), np.zeros(5))[0]
    for state in (first, second):
        rotation = np.array(state["camera_rotation"])
        translation = -rotation @ np.array(state["camera_position"])
        measured = pixels_on_plane(pixel, perception.matrix, perception.distortion,
                                   rotation, translation)
        np.testing.assert_allclose(measured, [[.1, 0., 0.]], atol=.003)
    lost = perception.process(np.full_like(frame, 245), 3, 0)
    assert not lost["marker_visible"] and not lost["objects"]


def test_unregistered_secondary_does_not_silently_switch_origin():
    config = perception_config()
    config["marker_size_m"] = .1
    config["dist_coeffs"] = [0., 0., 0., 0., 0.]
    frame, _, _ = tag_image(config, anchor=False)
    state = Perception(config).process(frame, 1, 0)
    assert len(state["detected_markers"]) == 1
    assert not state["marker_visible"]


def test_marker_only_retains_table_pose_and_tool_measurement_without_base_or_objects(monkeypatch):
    from types import SimpleNamespace

    config = perception_config()
    config['dist_coeffs'] = [0., 0., 0., 0., 0.]
    frame, _, _ = tag_image(config)
    full = Perception(config).process(frame, 1, 0)
    perception = Perception(config)

    def forbidden(*args, **kwargs):
        raise AssertionError('Marker-only acquisition must not fit objects or register bases')

    calls = []

    def tool(image, table, native, mapping):
        calls.append(table['camera_position'])
        return {'status': 'observed', 'position_valid': True, 'id': 8}

    monkeypatch.setattr('lerobot_arm_control.perception.tabletop_perception.fit_t', forbidden)
    perception.base_registrations = {'follower': SimpleNamespace(process=forbidden)}
    perception.tool_marker = SimpleNamespace(process=tool)
    measured = perception.process(frame, 2, 0, marker_only=True)
    np.testing.assert_allclose(measured['camera_position'], full['camera_position'])
    np.testing.assert_allclose(measured['camera_rotation'], full['camera_rotation'])
    assert calls == [measured['camera_position']]
    assert measured['tool_marker']['id'] == 8
    assert measured['objects'] == {}
    assert 'base_registrations' not in measured
    assert 'robot_registration' not in measured
    assert 'no base registration' in measured['observation_scope']


def test_saved_secondary_layout_localizes_after_restart_with_anchor_hidden(tmp_path):
    config = perception_config()
    config['robot']['enabled'] = False
    config['marker_size_m'] = .1
    config['dist_coeffs'] = [0.]*5
    perception = Perception(config)
    frame, _, _ = tag_image(config)
    first = perception.process(frame, 1, time.time_ns())
    first['objects']['black_t'] = {'xy': [.1, .2], 'yaw': .3, 'iou': .9, 'observed': True}
    path = tmp_path/'static_map.json'
    perception.save_static_map(path, first)
    restored = Perception(config)
    restored.restore_static_map(path)
    frame, _, _ = tag_image(config, anchor=False)
    current = restored.process(frame, 2, time.time_ns())
    assert current['marker_visible'] and not current['anchor_visible']
    assert current['pose_source'] == 'secondary'
    assert restored.secondary_layouts == perception.secondary_layouts


def test_wrong_camera_resolution_is_rejected():
    perception = Perception(perception_config())
    with pytest.raises(ValueError, match="resolution"):
        perception.process(np.zeros((240, 320, 3), np.uint8), 1, 0)


def test_bad_secondary_layout_falls_back_to_clear_anchor():
    config = perception_config()
    config["marker_size_m"] = .1
    config["dist_coeffs"] = [0.] * 5
    perception = Perception(config)
    frame, _, _ = tag_image(config)
    first = perception.process(frame, 1, 0)
    assert first["marker_visible"]
    key = next(iter(perception.secondary_layouts))
    layout = np.array(perception.secondary_layouts[key])
    layout[:, 0] += .1
    perception.secondary_layouts[key] = layout.tolist()
    state = perception.process(frame, 2, 0)
    assert state["marker_visible"]
    assert state["pose_source"] == "anchor_fallback"
    assert state["reprojection_error_px"] < config["max_reprojection_error_px"]
    np.testing.assert_allclose(state["camera_position"], first["camera_position"], atol=1e-6)


def test_native_marker_outside_low_crop_retains_calibrated_frame():
    config = perception_config()
    config['robot']['enabled'] = False
    config['additional_robots'] = []
    config['additional_markers'] = []
    config['dist_coeffs'] = [0.]*5
    native = np.full((960, 1280, 3), 245, np.uint8)
    tag = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h10), 1, 160)
    native[300:460, 100:260] = tag[:, :, None]
    mapping = np.array([[2., 0., 320.], [0., 2., 0.]])
    low = cv2.warpAffine(native, cv2.invertAffineTransform(mapping), (640, 480))
    perception = Perception(config)
    assert not perception.process(low, 1, 0)['marker_visible']
    measured = perception.process(low, 2, 0, native, mapping)
    assert measured['marker_visible'] and measured['anchor_visible']
    assert np.max(np.array(measured['tag_corners'])[:, 0]) < 0
    np.testing.assert_allclose(measured['tag_corners'],
                               [[-110.22, 149.78], [-30.28, 149.78],
                                [-30.28, 229.72], [-110.22, 229.72]], atol=.5)
    with pytest.raises(ValueError, match='pixel mapping'):
        perception.process(low, 3, 0, native)


def test_registered_secondary_can_register_new_tag_and_save_without_t(tmp_path):
    config = perception_config()
    config['robot']['enabled'] = False
    config['additional_robots'] = []
    config['marker_size_m'] = .1
    config['dist_coeffs'] = [0.]*5
    perception = Perception(config)
    frame, _, _ = tag_image(config)
    first = perception.process(frame, 1, 0)
    config['additional_markers'].append({'dictionary': 'DICT_APRILTAG_36h10', 'id': 1})
    observer = Perception(config)
    observer.secondary_layouts = first['secondary_layouts'].copy()
    observer.config = {**config, 'marker_id': 2}
    measured = observer.process(frame, 2, 0)
    assert measured['marker_visible'] and not measured['anchor_visible']
    assert measured['pose_source'] == 'secondary'
    assert 'DICT_APRILTAG_36h10:1' in measured['secondary_layouts']
    np.testing.assert_allclose(measured['secondary_layouts']['DICT_APRILTAG_36h10:1'],
                               observer.tag_points, atol=.003)
    measured['objects'] = {}
    path = tmp_path/'static_map.json'
    observer.save_static_map(path, measured)
    restored = Perception(observer.config)
    restored.restore_static_map(path)
    assert restored.secondary_layouts == observer.secondary_layouts
    assert restored.trackers['black_t'].last is None


def test_two_base_poses_are_published_without_replacing_follower_registration(monkeypatch):
    config = perception_config()
    perception = Perception(config)
    first = {'status': 'fixed', 'calibrated': True, 'markers': [], 'position': [0., 0., .0024]}
    second = {**first, 'position': [.3, .04, .0024]}
    monkeypatch.setattr(perception.base_registrations['follower'], 'process',
                        lambda *args: first)
    from lerobot_arm_control.perception.tabletop_registration import BaseRegistration

    extra = BaseRegistration(config['robot'], perception.matrix, perception.distortion)
    monkeypatch.setattr(extra, 'process', lambda *args: second)
    perception.base_registrations['arm1'] = extra
    state = perception.process(np.full((480, 640, 3), 255, np.uint8), 1, 1)
    assert state['robot_registration'] is first
    assert state['base_registrations'] == {'follower': first, 'arm1': second}
    assert state['base_distances'][0]['planar_distance_m'] == pytest.approx(math.hypot(.3, .04))


def test_fast_marker_frame_keeps_cached_object_age_and_still_updates_bases(monkeypatch):
    config = perception_config()
    config['robot']['enabled'] = False
    config['additional_robots'] = []
    perception = Perception(config)
    for tracker in perception.trackers.values():
        tracker.last = {'xy': [.1, .2], 'yaw': .3, 'iou': .9}
        tracker.last_observed = time.monotonic()-3

    class Registration:
        calls = 0

        def process(self, frame, state, native, mapping):
            self.calls += 1
            return {'status': 'fixed', 'markers': [], 'position': [0., 0., 0.],
                    'quaternion': [1., 0., 0., 0.]}

    registration = Registration()
    perception.base_registrations = {'follower': registration}

    def no_shape_fit(*args):
        pytest.fail('fast marker frames must not run silhouette searches')

    monkeypatch.setattr('lerobot_arm_control.perception.tabletop_perception.fit_t', no_shape_fit)
    state = perception.process(tag_image(config)[0], 1, time.time_ns(), observe_objects=False)
    assert state['marker_visible'] and registration.calls == 1
    assert 'follower' in state['base_registrations']
    assert state['objects']['wood_t']['tracking_status'] == 'lost'
    assert state['objects']['black_t']['tracking_status'] == 'static_cached'
    assert all(not obj['observed'] and obj['observation_age_s'] >= 3
               for obj in state['objects'].values())
