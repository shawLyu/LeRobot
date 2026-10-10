"""Local contact physics and adaptive target planning; no hardware imports or writes."""

import json
import shutil
from copy import deepcopy

import numpy as np
import pytest

from lerobot_arm_control.planning.pick_t import PickPlanner
from lerobot_arm_control.transport.pick_t_protocol import STAGES
from lerobot_arm_control.visualization.tabletop_scene import ROOT


@pytest.fixture
def pick(tmp_path):
    source = ROOT/'data/push_t/measured'
    snapshot = tmp_path/'snapshot'
    snapshot.mkdir()
    for name in ('calibration.json', 'joint_calibration.json', 'marker.png'):
        shutil.copyfile(source/name, snapshot/name)
    shutil.copytree(source/'appearance', snapshot/'appearance')
    state = json.loads((source/'state.json').read_text())
    base = state['robot']['registration']['position']
    state['objects']['wood_t'].update(xy=[base[0]-.02, base[1]+.21], yaw=0.)
    state['objects']['black_t'].update(xy=[base[0]+.03, base[1]+.23], yaw=.2)
    (snapshot/'state.json').write_text(json.dumps(state))
    return PickPlanner(snapshot, tmp_path/'simulation')


@pytest.mark.parametrize('offset,yaw', [((.05, .02), .2), ((.02, -.01), -.3)])
def test_original_jaws_pick_carry_rotate_and_release_wood_without_a_weld(pick, offset, yaw):
    source = pick.state['objects']['wood_t']['xy']
    pick.state['objects']['black_t'].update(xy=np.add(source, offset).tolist(), yaw=yaw)
    plan = pick.plan()
    assert [s['stage'] for s in plan['stages']] == list(STAGES)
    assert all(s['collision_samples'] >= 21 for s in plan['stages'])
    assert pick.model.neq == 0
    assert pick.model.joint('wood_free').type[0] == 0
    result = pick.simulate(plan)
    assert result['passed'], result
    assert result['final_center_error_m'] < .005
    assert all(s['wood_center_height_m'] > .02 for s in result['phases']
               if s['stage'] in ('lift', 'transfer'))


def test_unreachable_target_and_invalid_actual_pose_are_rejected(pick):
    pick.state['objects']['black_t']['xy'] = [2., 2.]
    with pytest.raises(ValueError, match='IK'):
        pick.plan()
    pick.initial[4] = -3.14
    with pytest.raises(ValueError, match='initial pose'):
        pick.plan()


def test_additional_arm_is_an_obstacle_and_only_its_preview_joints_are_constrained(pick):
    snapshot = pick.output.parent/'snapshot'
    hello = json.loads((snapshot/'calibration.json').read_text())
    extra = deepcopy(hello['config']['robot'])
    extra.update(arm='arm1', enabled=True, simulation={'preview_qpos': [.1]*6})
    hello['config']['additional_robots'] = [extra]
    (snapshot/'calibration.json').write_text(json.dumps(hello))
    state = json.loads((snapshot/'state.json').read_text())
    state['base_registrations'] = {'arm1': {
        'position': [.5, .5, .0024], 'quaternion': [1, 0, 0, 0],
        'calibrated': True, 'status': 'fixed'}}
    (snapshot/'state.json').write_text(json.dumps(state))
    planner = PickPlanner(snapshot, pick.output.parent/'two-arm')
    assert planner.model.neq == 6
    assert planner.model.body('arm1__robot_root').id > 0
    assert all(planner.model.joint(int(j)).name.startswith('arm1__')
               for j in planner.model.eq_obj1id)
    assert planner.model.joint('wood_free').type[0] == 0
    plan = planner.plan()
    assert planner.simulate(plan)['passed']
