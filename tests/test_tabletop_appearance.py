"""Remote texture tests reject foreground ghosts and preserve object-local material sampling."""

import base64
import json

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from arm_control.tabletop_appearance import ObservedAppearance  # noqa: E402
from arm_control.tabletop_tracking import world_outline  # noqa: E402


def observation(yaw=0.0):
    config = json.load(open("configs/tabletop.json"))
    config.update(
        camera_matrix=[[600.0, 0, 320.0], [0, 600.0, 240.0], [0, 0, 1]],
        dist_coeffs=[0.0] * 5,
        robot={"enabled": False},
        appearance={"enabled": True, "texture_size": 256},
    )
    model = {"width": 0.048, "length": 0.060, "cap_depth": 0.012, "stem_width": 0.012}
    state = {
        "marker_visible": True,
        "camera_rotation": np.diag([1.0, -1.0, -1.0]).tolist(),
        "camera_position": [0, 0, 1.0],
        "models": {},
        "objects": {},
        "sequence": 1,
        "captured_unix_ns": 123,
        "secondary_layouts": {
            "DICT_APRILTAG_16h5:0": [
                [-0.25, 0.3, 0],
                [-0.10, 0.3, 0],
                [-0.10, 0.15, 0],
                [-0.25, 0.15, 0],
            ]
        },
    }
    extractor = ObservedAppearance(config)
    frame = np.full((480, 640, 3), 210, np.uint8)
    for name, xy, height, color in [
        ("black_t", [0.10, 0.10], 0.0, (65, 65, 65)),
        ("wood_t", [-0.06, 0.12], 0.006, (90, 195, 220)),
    ]:
        state["models"][name] = {**model, "height": height}
        outline = world_outline(model, xy, yaw)
        state["objects"][name] = {"xy": xy, "yaw": yaw, "complete_outline_xy": outline.tolist()}
        pixels = extractor.project(np.c_[outline, np.full(8, height)], state, False)
        cv2.fillPoly(frame, [np.int32(pixels)], color)
    return config, state, frame


def test_object_color_and_foreground_removed_from_table_texture():
    config, state, frame = observation()
    result = ObservedAppearance(config).extract(frame, state)
    assert result is not None
    metadata = result["metadata"]
    assert metadata["perception_location"] == "remote"
    assert metadata["lighting"]["physically_calibrated"] is False
    color = metadata["materials_rgb"]["wood_t_cap"]
    assert color[0] > color[1] > color[2]
    table = cv2.imdecode(
        np.frombuffer(base64.b64decode(result["pngs"]["tabletop.png"]), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    # Uniform synthetic background must remain uniform after the black/yellow T is removed.
    assert table.std() < 3


def test_rotating_object_keeps_its_material_color():
    colors = []
    for yaw in (0.0, 0.7):
        config, state, frame = observation(yaw)
        result = ObservedAppearance(config).extract(frame, state)
        assert result is not None
        colors.append(result["metadata"]["materials_rgb"]["wood_t_cap"])
    np.testing.assert_allclose(colors[0], colors[1], atol=0.015)


def test_no_plane_means_no_observed_texture():
    config, state, frame = observation()
    state["marker_visible"] = False
    assert ObservedAppearance(config).extract(frame, state) is None
