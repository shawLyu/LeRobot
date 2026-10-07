"""Physical offline Push-T checks; never import or connect hardware drivers."""

import ast
import math
import shutil

import mujoco
import numpy as np
import pytest

from arm_control.push_t import (
    CONTACT_HEIGHT,
    ROOT,
    PushPlanner,
    PushSimulation,
    build_scene,
    errors,
    pose,
)


@pytest.fixture(scope='module')
def sim(tmp_path_factory):
    return PushSimulation(output=tmp_path_factory.mktemp('push-t'))


def test_offline_snapshot_runs_without_generated_scenes_photos_or_duplicate_meshes(tmp_path):
    source = tmp_path/'snapshot'
    shutil.copytree(ROOT/'data/push_t/measured', source,
                    ignore=shutil.ignore_patterns('robot_assets', 'scene.xml',
                                                  'so101_source.xml', '*.jpg'))
    output = tmp_path/'simulation'
    _, _, path = build_scene(source, output)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nq == 13 and model.nu == 6 and model.nmesh == 18
    canonical = ROOT/'assets/so101/assets'
    for mesh in canonical.glob('*.stl'):
        assert (output/'robot_assets'/mesh.name).read_bytes() == mesh.read_bytes()


def test_scene_preserves_measured_poses_and_has_real_contact_physics(sim):
    sim.reset(sim.initial)
    np.testing.assert_allclose(pose(sim.data), sim.initial, atol=1e-6)
    np.testing.assert_allclose(sim.data.body('black_t').xpos[:2], sim.target[:2])
    np.testing.assert_allclose(sim.model.body('robot_root').pos,
                               sim.state['robot']['registration']['position'])
    assert sim.model.nu == 6 and sim.model.nq == 13
    assert sim.model.body('wood_t').mass[0] > 0
    np.testing.assert_allclose(sim.model.opt.gravity, [0, 0, -9.81])
    assert any(sim.model.geom_bodyid[c.geom2] == sim.model.body('wood_t').id
               or sim.model.geom_bodyid[c.geom1] == sim.model.body('wood_t').id
               for c in sim.data.contact)
    for name in ('wood_t_cap', 'wood_t_stem'):
        assert sim.model.geom(name).contype[0] == 1
    assert sim.model.geom('black_t_cap').contype[0] == 0
    names = {sim.model.geom(i).name for i in range(sim.model.ngeom)}
    assert not {'push_tip_contact', 'push_tool_shaft'} & names
    assert sim.roll_model.nq == sim.model.nq
    assert sim.roll_model.ngeom == sim.model.ngeom
    assert sim.roll_model.nmocap == 0
    assert 'fixed_jaw_box3' in names and 'moving_jaw_box2' in names


def test_ik_rejects_unreachable_and_respects_limits(sim):
    seed = sim.data.qpos[sim.qadr].copy()
    assert sim.arm.solve([2, 2, .01], seed, attempts=3) is None
    result = sim.arm.solve([*sim.target[:2], .035], seed, attempts=8)
    assert result is not None
    assert np.all(result >= sim.arm.limits[:, 0])
    assert np.all(result <= sim.arm.limits[:, 1])
    sim.arm.data.qpos[sim.qadr] = result
    mujoco.mj_forward(sim.model, sim.arm.data)
    assert np.linalg.norm(sim.arm.data.site('push_contact').xpos-[*sim.target[:2], .035]) < .001


def test_no_network_camera_or_motor_driver_imports():
    module = ROOT/'src/arm_control/push_t.py'
    syntax = ast.parse(module.read_text())
    imports = [n.module or '' for n in ast.walk(syntax) if isinstance(n, ast.ImportFrom)]
    imports += [a.name for n in ast.walk(syntax) if isinstance(n, ast.Import) for a in n.names]
    assert not any(name in ('subprocess', 'socket', 'serial', 'cv2', 'lerobot') for name in imports)
    assert not any('driver' in name or 'realtime' in name or 'tabletop_client' in name
                   for name in imports)


def test_original_gripper_prediction_matches_complete_executed_motion(sim):
    sim.reset(sim.initial)
    initial = sim.data.qpos.copy()
    initial_time = sim.data.time
    action = sim.plan()
    assert action is not None
    sim.predict(action, pose(sim.data), friction=.4)
    predicted = sim.roll_data.qpos.copy()
    np.testing.assert_array_equal(sim.data.qpos, initial)
    assert sim.data.time == initial_time
    sim.execute(action)
    np.testing.assert_allclose(sim.data.qpos, predicted, atol=1e-9)
    assert sum(sim.contact_counts.values()) > 0
    assert abs(sim.data.site('push_contact').xpos[2]-CONTACT_HEIGHT) < .001
    assert not sim.trace[-1]['action']['transition'].startswith('retract')


def test_prefetched_plan_matches_execution_and_rejects_changed_state(sim):
    sim.reset(sim.initial)
    planner = PushPlanner(sim)
    planner.capture(sim)
    planned = planner.plan()
    assert planned.action is not None and planned.matches(sim.model, sim.data)
    assert not planned.prefetched
    original = sim.data.qpos.copy()
    planner.capture(sim)
    future = planner.plan(after=planned.action)
    assert future.prefetched and future.action is not None
    np.testing.assert_array_equal(sim.data.qpos, original)
    assert sim.contact_counts == {}
    sim.execute(planned.action)
    assert future.matches(sim.model, sim.data)
    sim.data.qvel[0] += .01
    assert not future.matches(sim.model, sim.data)
    sim.data.qvel[0] -= .01
    sim.reset(sim.initial)
    assert not future.matches(sim.model, sim.data)


def test_measured_start_converges_through_contact_with_full_arm(sim):
    sim.reset(sim.initial)
    before = pose(sim.data).copy()
    result = sim.solve(65)
    assert result['success'], result
    distance, yaw = errors(pose(sim.data), sim.target)
    assert distance < .0015 and yaw < math.radians(3)
    assert np.linalg.norm(before[:2]-pose(sim.data)[:2]) > .10
    assert sim.pushes > 0 and sim.data.time > 1
    assert result['pusher'] == 'original_so101_closed_gripper'
    assert sum(result['jaw_contacts'].values()) > 100
    assert any(name.startswith(('fixed_jaw', 'moving_jaw', 'gripper_collision'))
               for name in result['jaw_contacts'])
    assert np.all(sim.data.qpos[sim.qadr] >= sim.arm.limits[:, 0]-.002)
    assert np.all(sim.data.qpos[sim.qadr] <= sim.arm.limits[:, 1]+.002)
    assert not result['chunk_end_retraction']
    assert result['contact_continuations'] > 0
    assert result['lift_transfers'] < sim.pushes
    assert max(abs(t['gripper_height_m']-CONTACT_HEIGHT) for t in result['trace']) < .001


def test_contact_continuation_contains_only_low_pushing_waypoints(sim):
    sim.reset(sim.initial)
    for _ in range(10):
        action = sim.plan()
        assert action is not None
        if action.transition == 'contact continuation':
            assert action.approach_count == 0
            assert len(action.joints) == 10
            for q in action.joints:
                sim.arm.data.qpos[sim.qadr] = q
                mujoco.mj_forward(sim.model, sim.arm.data)
                assert abs(sim.arm.data.site('push_contact').xpos[2]-CONTACT_HEIGHT) < .001
            break
        sim.execute(action)
    else:
        pytest.fail('Measured trajectory never retained contact across a chunk boundary')


def test_wood_table_contact_does_not_block_robot_departure(sim):
    sim.reset(sim.initial)
    arm = sim.arm
    mujoco.mj_copyData(arm.data, sim.model, sim.data)
    yaw, roll = sim.initial[2]/2, .05/2
    arm.data.qpos[sim.wood_qadr+3:sim.wood_qadr+7] = [
        math.cos(yaw)*math.cos(roll), math.cos(yaw)*math.sin(roll),
        math.sin(yaw)*math.sin(roll), math.sin(yaw)*math.cos(roll)]
    q = arm.solve([*sim.target[:2], .045], sim.data.qpos[sim.qadr], attempts=8)
    assert q is not None
    assert any(c.dist < -.0005 and any(sim.model.geom_bodyid[g] == 0
                                      for g in (c.geom1, c.geom2))
               for c in arm.data.contact)
    assert arm.path_clear(q, q, avoid_wood=True)
