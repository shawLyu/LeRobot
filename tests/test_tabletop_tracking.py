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


def test_missing_first_observation_does_not_invent_a_pose():
    assert update_without_visible_shape(TTracker(DIMENSIONS), 10.) is None
