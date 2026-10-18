"""Remote plane validation uses no simulator or optional scientific dependency."""

import json
import math

import pytest

from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
from lerobot_arm_control.tasks.circle.geometry import circle_frame


def test_inclined_basis_preserves_radius_and_normal_distance():
    angle = math.radians(30)
    u, v, normal = circle_frame('inclined',
                                [[math.cos(angle), 0, -math.sin(angle)], [0, 1, 0]])
    point = [0.08*u[i]+0.003*normal[i] for i in range(3)]
    assert sum(a*b for a, b in zip(point, normal)) == pytest.approx(.003)
    assert sum(a*b for a, b in zip(point, u)) == pytest.approx(.08)
    assert sum(a*b for a, b in zip(point, v)) == 0


@pytest.mark.parametrize('basis', [None, [], [[1, 0, 0]],
                                  [[1, 0, 0], [1, 0, 0]],
                                  [[2, 0, 0], [0, 1, 0]],
                                  [[1, 0, 0], [0, math.nan, 0]],
                                  [[True, 0, 0], [0, 1, 0]]])
def test_malformed_inclined_plane_is_rejected(basis):
    with pytest.raises(ValueError, match='basis'):
        circle_frame('inclined', basis)


def test_axis_aligned_plane_cannot_hide_an_inclined_basis():
    with pytest.raises(ValueError, match='must not override'):
        circle_frame('XY', [[1, 0, 0], [0, 1, 0]])


@pytest.mark.parametrize('angle', [15, 30])
def test_reviewed_inclined_circle_retains_table_reference_and_motion_budgets(angle):
    circle = Circle(resource_path(f'data/trajectories/table-circle120-tilt{angle}.json'))
    assert circle.table_bound
    assert circle.normal[2] == pytest.approx(math.cos(math.radians(angle)))
    assert circle.period == 3 and circle.ramp_seconds == 8
    assert circle.feedback_profile['name'] == 'smooth-circle120-v1'
    assert circle.check_motion_budget(circle.points[0])
    circle.validate_laps(4)


@pytest.mark.parametrize('change', ['table_z_m', 'joint_calibration_sha256', 'held_joint'])
def test_inclined_circle_cannot_omit_table_identity_or_move_held_joint(tmp_path, change):
    saved = json.loads(resource_path('data/trajectories/table-circle120-tilt15.json').read_text())
    if change == 'held_joint':
        saved['qpos'][1][4] += .01
    else:
        saved.pop(change)
    path = tmp_path/'invalid.json'
    path.write_text(json.dumps(saved))
    with pytest.raises(ValueError, match='table-referenced'):
        Circle(path)
