"""The nominal installed mount stays consistent with the immutable CAD references."""

import json

import numpy as np
import pytest

pytest.importorskip('mujoco')

from lerobot_arm_control.calibration.tool_mount import cad_reference  # noqa: E402
from lerobot_arm_control.robot.so101.model_metadata import MODEL  # noqa: E402


def test_specified_edge_distance_uses_the_rounded_shoulder_not_flat_patch():
    reference = cad_reference()
    surface = reference['outer_flat_surface']
    assert surface['plane_x_m'] == pytest.approx(-.0352)
    assert surface['upper_rounded_edge_z_m'] == pytest.approx(-.005)
    assert surface['patch_bounds_m'][1][2] == pytest.approx(-.007976)
    mount = reference['marker_pose_in_gripper']
    np.testing.assert_allclose(mount['position_m'], [-.0352, -.000218, -.016])
    assert not mount['transform_validated']
    rotation = np.array(mount['rotation'])
    np.testing.assert_allclose(rotation.T@rotation, np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1)
    contact = (np.array(mount['position_m'])
               +rotation@reference['contact_offset_marker_m'])
    np.testing.assert_allclose(contact, [-.009, 0, -.103], atol=1e-9)


def test_shared_configuration_matches_cad_and_keeps_mount_unvalidated():
    config = json.loads((MODEL.parents[2]/'configs/tabletop.json').read_text())
    marker = config['robot']['tool_marker']
    reference = cad_reference()
    assert marker['cad_reference'] == reference
    assert marker['tool_reference']['offset_marker_m'] == reference['contact_offset_marker_m']
    assert not marker['tool_reference']['transform_validated']
    assert not marker['tool_transform_measured']
