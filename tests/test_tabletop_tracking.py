"""Known-shape tests with missing boundaries and moving partial observations."""

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2", reason="Remote OpenCV perception tests")

from lerobot_arm_control.perception.tabletop_tracking import (  # noqa: E402
    TTracker,
    match_partial,
    t_outline,
)

DIMENSIONS = {"width": .048, "length": .06, "cap_depth": .012, "stem_width": .012,
              "height": .006, "height_measured": True}


def render_mask(xy=(0., 0.), yaw=.7, occlusion=None):
    resolution, origin = .00075, np.array([-.08, -.08])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    polygon = t_outline(DIMENSIONS) @ rotation.T + xy
    mask = np.zeros((214, 214), np.uint8)
    cv2.fillPoly(mask, [np.round((polygon-origin)/resolution).astype(np.int32)], 255)
    xx, yy = np.meshgrid(np.arange(214)*resolution+origin[0],
                         np.arange(214)*resolution+origin[1])
    local = (np.stack((xx, yy), axis=2)-xy) @ rotation
    if occlusion == "partial":
        mask[(local[:, :, 0] > 0) & (local[:, :, 1] > 0)] = 0
    elif occlusion == "stem_only":
        mask[local[:, :, 1] > .015] = 0
    return mask, origin, resolution


def test_partially_hidden_moving_t_recovers_full_fixed_shape():
    previous = {"xy": [0., 0.], "yaw": .7}
    xy, yaw = [.006, -.004], .78
    mask, origin, resolution = render_mask(xy, yaw, "partial")
    match = match_partial(mask, origin, resolution, DIMENSIONS, previous)
    assert match is not None
    np.testing.assert_allclose(match["xy"], xy, atol=.003)
    assert match["yaw"] == pytest.approx(yaw, abs=.06)
    assert len(match["complete_outline_xy"]) == 8
    assert match["visible_fraction"] < .9
    assert match["edge_support"] > .42


def test_filled_occluder_is_not_a_t_observation():
    mask = np.full((214, 214), 255, np.uint8)
    assert match_partial(mask, [-.08, -.08], .00075, DIMENSIONS,
                         {"xy": [0., 0.], "yaw": .7}) is None


def test_only_stem_does_not_claim_an_unambiguous_pose():
    mask, origin, resolution = render_mask(occlusion="stem_only")
    assert match_partial(mask, origin, resolution, DIMENSIONS,
                         {"xy": [0., 0.], "yaw": .7}) is None


def update_without_visible_shape(tracker, now):
    matrix = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
    return tracker.update([], np.zeros((480, 640), np.uint8), matrix, np.zeros(5),
                          np.diag([1., -1, -1]), np.array([0., 0., 1.]), now)


def test_mobile_object_hold_expires_and_does_not_change_dimensions():
    tracker = TTracker(DIMENSIONS, hold_seconds=2.)
    tracker.last = {"xy": [.1, .2], "yaw": .3, "iou": .9}
    tracker.last_observed = 10.
    held = update_without_visible_shape(tracker, 11.)
    assert held["tracking_status"] == "held" and not held["observed"]
    assert held["width"] == .048 and held["length"] == .06 and held["height"] == .006
    lost = update_without_visible_shape(tracker, 12.1)
    assert lost["tracking_status"] == "lost" and not lost["pose_valid"]
    np.testing.assert_allclose(lost["xy"], [.1, .2])


def test_static_print_keeps_full_cached_outline_with_age():
    tracker = TTracker({**DIMENSIONS, "height": 0}, static=True)
    tracker.last = {"xy": [.1, .2], "yaw": .3, "iou": .9}
    tracker.last_observed = 10.
    state = update_without_visible_shape(tracker, 20.)
    assert state["tracking_status"] == "static_cached" and not state["observed"]
    assert state["observation_age_s"] == 10.
    assert len(state["complete_outline_xy"]) == 8


def test_partial_static_template_cannot_move_fixed_target_or_refresh_capture_age(monkeypatch):
    def unused_search(*args):
        pytest.fail('occluded static targets must not consume partial-search latency')

    monkeypatch.setattr('lerobot_arm_control.perception.tabletop_tracking.match_partial',
                        unused_search)
    xy, yaw = [.006, -.004], .78
    mask, origin, resolution = render_mask(xy, yaw, 'partial')
    tracker = TTracker({**DIMENSIONS, 'height': 0}, static=True)
    tracker.last = {'xy': [0., 0.], 'yaw': .7, 'iou': .9}
    tracker.last_observed = 1.
    matrix = np.array([[1/resolution, 0, -origin[0]/resolution],
                        [0, 1/resolution, -origin[1]/resolution], [0, 0, 1.]])
    state = tracker.update([], mask, matrix, np.zeros(5), np.eye(3),
                            np.array([0., 0., 1.]), 2.)
    assert state['tracking_status'] == 'static_cached' and not state['observed']
    assert state['xy'] == [0., 0.] and state['yaw'] == .7
    assert state['observation_age_s'] == 1.
    assert 'partial_candidate' not in state


def test_missing_first_observation_does_not_invent_a_pose():
    assert update_without_visible_shape(TTracker(DIMENSIONS), 10.) is None


def test_static_target_rejects_single_frame_jitter_but_tracks_sustained_displacement():
    tracker = TTracker(DIMENSIONS, static=True)
    for now, x in [(1., 0.), (1.1, .0002), (1.2, .004)]:
        tracker._record({'xy': [x, 0.], 'yaw': 2*math.pi-.01}, now)
    assert tracker.last['xy'][0] == pytest.approx(.0002)
    assert tracker.last['measured_xy'][0] == .004
    tracker._record({'xy': [.004, 0.], 'yaw': .01}, 1.3)
    assert tracker.last['xy'][0] == .004
    assert abs(math.atan2(math.sin(tracker.last['yaw']), math.cos(tracker.last['yaw']))) < .02
    cached = tracker.cached(2.)
    assert not cached['observed'] and cached['observation_age_s'] == pytest.approx(.7)
    tracker._record({'xy': [.03, 0.], 'yaw': .01}, 2.1)
    assert tracker.last['xy'][0] == .03


def test_moving_wood_is_never_temporally_filtered():
    tracker = TTracker(DIMENSIONS)
    for now, x in [(1., 0.), (1.1, .0002), (1.2, .004)]:
        tracker._record({'xy': [x, 0.], 'yaw': .3}, now)
    assert tracker.last['xy'][0] == .004
    assert 'filter_window_s' not in tracker.last
