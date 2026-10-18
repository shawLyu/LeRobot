"""Verify articulated CAD stays intact when attached to the desktop coordinate frame."""

import json

import mujoco
import numpy as np
import pytest

from lerobot_arm_control.visualization.tabletop_scene import ROOT, scene_xml


def load_scene(tmp_path):
    config = json.loads((ROOT / "configs/tabletop.json").read_text())
    state = {
        "objects": {},
        "models": {},
        "robot": {
            "status": "live",
            "qpos": [-0.4, -1.45, 1.5, 0.88, 0.08, 0.31],
            "registration": {"position": [0.2, -0.3, 0.0024], "quaternion": [1, 0, 0, 0]},
        },
    }
    (tmp_path / "marker.png").write_bytes((ROOT / "assets/tabletop/marker.png").read_bytes())
    path = tmp_path / "scene.xml"
    path.write_text(scene_xml(state, config))
    return mujoco.MjModel.from_xml_path(str(path))


def test_six_joints_preserved_and_no_actuators(tmp_path):
    model = load_scene(tmp_path)
    assert model.nq == 6
    assert model.nu == 0
    assert model.nkey == 1
    np.testing.assert_allclose(model.key_qpos[0], [-0.4, -1.45, 1.5, 0.88, 0.08, 0.31])
    assert model.nmesh == 18
    assert tuple(model.joint(i).name for i in range(model.njnt)) == (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )


def test_base_mocap_moves_complete_articulated_chain(tmp_path):
    model = load_scene(tmp_path)
    data = mujoco.MjData(model)
    data.qpos[:] = [-0.4, -1.45, 1.5, 0.88, 0.08, 0.31]
    mujoco.mj_forward(model, data)
    before = data.site("gripperframe").xpos.copy()
    mocap = model.body_mocapid[model.body("robot_root").id]
    data.mocap_pos[mocap] += [0.07, 0.02, 0.01]
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.site("gripperframe").xpos - before, [0.07, 0.02, 0.01])
    data.qpos[model.joint("elbow_flex").qposadr[0]] += 0.1
    mujoco.mj_forward(model, data)
    assert np.linalg.norm(data.site("gripperframe").xpos - before - [0.07, 0.02, 0.01]) > 0.01


def test_tool_marker_is_fixed_to_the_finger_and_has_a_20_mm_black_square(tmp_path):
    model = load_scene(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    finger = data.body('gripper')
    center = data.site('tool_marker_8_center').xpos.copy()
    local = (center-finger.xpos)@finger.xmat.reshape(3, 3)
    np.testing.assert_allclose(local, [-.0352, -.000218, -.016], atol=1e-9)
    body = model.body('tool_marker_8').id
    assert model.body_parentid[body] == model.body('gripper').id
    data.qpos[model.joint('gripper').qposadr[0]] += .2
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.site('tool_marker_8_center').xpos, center)
    data.qpos[model.joint('wrist_roll').qposadr[0]] += .2
    mujoco.mj_forward(model, data)
    assert np.linalg.norm(data.site('tool_marker_8_center').xpos-center) > .005
    first, last = model.geom('tool_marker_8_0_0'), model.geom('tool_marker_8_5_5')
    assert last.pos[0]-first.pos[0]+first.size[0]+last.size[0] == pytest.approx(.020)
    assert first.pos[1]-last.pos[1]+first.size[1]+last.size[1] == pytest.approx(.020)
    visuals = model.geom_bodyid == body
    assert np.all(model.geom_contype[visuals] == 0)
    assert np.all(model.geom_conaffinity[visuals] == 0)
    assert model.nu == 0


def test_original_calibrated_model_not_modified():
    import hashlib

    saved = json.loads((ROOT / "data/calibration/follower_mujoco.json").read_text())
    actual = hashlib.sha256((ROOT / "assets/so101/so101.xml").read_bytes()).hexdigest()
    assert saved["fingerprint"]["mjcf_sha256"] == actual


def test_leader_trigger_mesh_uses_the_upstream_fixed_rpy_frame():
    import math
    import xml.etree.ElementTree as ET

    leader = ET.parse(ROOT/'assets/so101/so101_leader.xml')
    source = ET.parse(ROOT/'assets/so101/so101_leader_new_calib.urdf')
    geom = leader.find(".//geom[@mesh='Trigger_SO101'][@class='visual']")
    origin = source.find("link[@name='trigger_link']/visual/origin")
    roll, pitch, yaw = map(float, origin.get('rpy').split())
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    expected = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
                np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
                np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    actual = np.empty(9)
    mujoco.mju_quat2Mat(actual, np.fromstring(geom.get('quat'), sep=' '))
    np.testing.assert_allclose(actual.reshape(3, 3), expected, atol=1e-10)


def test_leader_trigger_joint_and_actuator_cover_the_configured_full_travel():
    import math

    from lerobot_arm_control.control.follow import load_follow_config

    model = mujoco.MjModel.from_xml_path(str(ROOT/'assets/so101/scene_leader.xml'))
    expected = list(map(math.radians, load_follow_config()['leader_trigger_degrees']))
    np.testing.assert_allclose(model.joint('gripper').range, expected, atol=1e-10)
    np.testing.assert_allclose(model.actuator('gripper').ctrlrange, expected, atol=1e-10)


def test_two_arms_share_meshes_but_have_independent_joints_bases_and_materials(tmp_path):
    from lerobot_arm_control.visualization.tabletop_scene import scene_arms

    config = json.loads((ROOT / 'configs/tabletop.json').read_text())
    registrations = {
        'follower': {'position': [-.2, 0., .0024], 'quaternion': [1., 0., 0., 0.]},
        'arm1': {'position': [.1, .02, .0024], 'quaternion': [1., 0., 0., 0.]},
    }
    state = {'objects': {}, 'models': {}, 'base_registrations': registrations,
             'robot': {'status': 'live', 'qpos': [0, 1.2, -1.5, .6, 0, .2],
                       'registration': registrations['follower']}}
    (tmp_path/'marker.png').write_bytes((ROOT/'assets/tabletop/marker.png').read_bytes())
    path = tmp_path/'scene.xml'
    path.write_text(scene_xml(state, config))
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nq == 12 and model.nu == 0 and model.nmesh == 21
    assert model.joint('arm1__gripper').range[1] == pytest.approx(np.deg2rad(
        json.loads((ROOT/'configs/robot.json').read_text())['follow']['leader_trigger_degrees'][1]))
    head = model.body('arm1__gripper').id
    meshes = {model.mesh(int(index)).name for index in model.geom_dataid[
        (model.geom_bodyid == head) & (model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)]}
    assert {'Handle_SO101', 'Wrist_Roll_SO101'} <= meshes
    assert 'wrist_roll_follower_so101_v1' not in meshes
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.qpos[:6], state['robot']['qpos'])
    np.testing.assert_allclose(data.qpos[6:], config['additional_robots'][0]['simulation']
                              ['preview_qpos'])
    for arm, body in [('follower', 'robot_root'), ('arm1', 'arm1__robot_root')]:
        np.testing.assert_allclose(data.body(body).xpos, registrations[arm]['position'])
    white = data.site('arm1__gripperframe').xpos.copy()
    data.qpos[model.joint('shoulder_pan').qposadr[0]] += .1
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.site('arm1__gripperframe').xpos, white)
    white_material = model.mat('arm1__base_so101_v2_material').id
    np.testing.assert_allclose(model.mat_rgba[white_material, :3], [.88, .89, .87])
    assert scene_arms(state, config)[1]['joint_pose_source'] == 'unmeasured_preview'
    for name, marker in [('follower', 7), ('arm1', 6)]:
        prefix = '' if name == 'follower' else name+'__'
        points = [data.geom(f'{prefix}base_marker_{marker}_{row}_{col}').xpos
                  for row in range(6) for col in range(6)]
        # The follower height was remeasured; the other mount retains its old measurement.
        expected_height = .0701 if name == 'follower' else .0691
        assert np.mean(points, axis=0)[2] == pytest.approx(expected_height)


def test_unlocalized_second_arm_is_not_drawn_at_an_invented_base(tmp_path):
    model = load_scene(tmp_path)
    assert model.nq == 6
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm1__robot_root') == -1


@pytest.fixture
def two_arm_scene(tmp_path):
    from lerobot_arm_control.robot.so101.joints import JOINTS

    config = json.loads((ROOT/'configs/tabletop.json').read_text())
    bases = {name: {'status': 'fixed', 'position': position, 'quaternion': [1., 0., 0., 0.]}
             for name, position in [('follower', [0., .1, .0024]), ('arm1', [1.2, .2, .0024])]}
    state = {'sequence': 12, 'captured_unix_ns': 123, 'units': 'm', 'marker_visible': True,
             'objects': {}, 'models': {}, 'wood_height_measured': True,
             'capture_age_at_send_ms': 20, 'base_registrations': bases,
             'robot': {'status': 'live', 'qpos': [0., 1., -1., .5, .1, .2],
                       'joints': list(JOINTS), 'sample_age_at_send_ms': 20,
                       'registration': bases['follower']},
             'robot_feedback': {'arm1': {'status': 'live', 'qpos': [.3, .8, -1.2, .3, .2, .1],
                                        'joints': list(JOINTS), 'sample_age_at_send_ms': 20}}}
    (tmp_path/'marker.png').write_bytes((ROOT/'assets/tabletop/marker.png').read_bytes())
    (tmp_path/'robot_assets').symlink_to(ROOT/'assets/so101/assets', target_is_directory=True)
    return state, config


def test_white_live_joints_update_independently_and_stop_on_stale_feedback(tmp_path, two_arm_scene):
    from lerobot_arm_control.visualization.tabletop_scene import update_scene_robots

    state, config = two_arm_scene
    path = tmp_path/'scene.xml'
    path.write_text(scene_xml(state, config, 'robot_assets'))
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    update_scene_robots(model, data, state, config, .01)
    np.testing.assert_allclose(data.qpos[:6], state['robot']['qpos'])
    np.testing.assert_allclose(data.qpos[6:], state['robot_feedback']['arm1']['qpos'])
    white_joint = model.joint('arm1__shoulder_pan').qposadr[0]
    state['robot_feedback']['arm1'].update(sample_age_at_send_ms=600, qpos=[.9]*6)
    update_scene_robots(model, data, state, config, .01)
    assert data.qpos[white_joint] == pytest.approx(.3)
    np.testing.assert_allclose(data.qpos[:6], state['robot']['qpos'])


def test_workspace_and_saved_keyframe_cover_both_current_arms(tmp_path, two_arm_scene):
    from lerobot_arm_control.perception.tabletop_geometry import workspace_bounds
    from lerobot_arm_control.visualization.tabletop_snapshot import save_scene_snapshot

    state, config = two_arm_scene
    assert workspace_bounds(state, config)[1] > 1.4
    state['robot_feedback']['arm1'] = {'status': 'uncalibrated'}
    parameters = save_scene_snapshot(tmp_path, state, {'config': config})
    assert parameters['arms']['arm1']['joint_pose_source'] == 'unmeasured_preview'
    assert parameters['arms']['arm1']['feedback_status'] == 'uncalibrated'
    assert json.loads((tmp_path/'state.json').read_text()) == state
    model = mujoco.MjModel.from_xml_path(str(tmp_path/'scene.xml'))
    assert model.nq == 12 and model.nu == 0
    np.testing.assert_allclose(model.key_qpos[0, :6], state['robot']['qpos'])
    np.testing.assert_allclose(model.key_qpos[0, 6:], parameters['arms']['arm1']['qpos'])
    np.testing.assert_allclose(model.body('arm1__robot_root').pos, [1.2, .2, .0024])


def test_independent_mapping_is_preserved_in_the_saved_scene(tmp_path, two_arm_scene):
    from lerobot_arm_control.visualization.tabletop_scene import scene_arms, update_scene_robots
    from lerobot_arm_control.visualization.tabletop_snapshot import save_scene_snapshot

    state, config = two_arm_scene
    state['robot_feedback']['arm1'].update(calibration_mode='independent',
        calibration_source_arm='arm1', independently_calibrated=True)
    assert scene_arms(state, config)[1]['joint_pose_source'] == 'live_encoders'
    parameters = save_scene_snapshot(tmp_path, state, {'config': config})
    assert parameters['arms']['arm1']['calibration_source_arm'] == 'arm1'
    assert parameters['arms']['arm1']['independently_calibrated'] is True
    model = mujoco.MjModel.from_xml_path(str(tmp_path/'scene.xml'))
    data = mujoco.MjData(model)
    update_scene_robots(model, data, state, config, .01)
    np.testing.assert_allclose(data.qpos[6:], state['robot_feedback']['arm1']['qpos'])
    assert model.geom('arm1__base_marker_6_0_0').rgba[3] == pytest.approx(1.)


def test_observed_materials_compile_and_preserve_calibrated_joints(tmp_path):
    from PIL import Image

    config = json.loads((ROOT / "configs/tabletop.json").read_text())
    model_shape = {**config["t_model"], "height": 0.006}
    state = {
        "models": {"wood_t": model_shape},
        "objects": {"wood_t": {"xy": [0.1, 0.1], "yaw": 0.4}},
        "robot": {
            "status": "live",
            "qpos": [-0.4, -1.45, 1.5, 0.88, 0.08, 0.31],
            "registration": {"position": [0.2, -0.3, 0.0024], "quaternion": [1, 0, 0, 0]},
        },
        "appearance": {
            "version": 1,
            "bounds": [-0.3, 0.4, -0.5, 0.4],
            "robot_plastic_rgb": [0.06, 0.07, 0.08],
            "texture_files": ["tabletop.png", "wood_t_cap.png", "wood_t_stem.png"],
        },
    }
    directory = tmp_path / "appearance"
    directory.mkdir()
    for name in state["appearance"]["texture_files"]:
        Image.new("RGB", (16, 16), (220, 205, 120)).save(directory / name)
    (tmp_path / "marker.png").write_bytes((ROOT / "assets/tabletop/marker.png").read_bytes())
    path = tmp_path / "scene.xml"
    path.write_text(scene_xml(state, config))
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nq == 6 and model.nu == 0
    assert model.ntex == 5  # Observed surfaces, anchor and room background.
    assert model.geom("wood_t_cap").matid[0] == model.mat("observed_wood_t_cap").id
    np.testing.assert_allclose(model.key_qpos[0], state["robot"]["qpos"])
    assert model.light_castshadow.tolist() == [True, False]
    for material in range(model.nmat):
        if model.mat(material).name.endswith("_material"):
            np.testing.assert_allclose(model.mat_rgba[material, :3], [0.06, 0.07, 0.08])
