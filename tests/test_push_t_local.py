"""Local planning and MuJoCo prediction; no hardware is used."""

import copy
import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from test_push_t_remote import observed as observed  # noqa: F401

pytest.importorskip("mujoco")

from arm_control.push_t import ROOT, Push  # noqa: E402
from arm_control.push_t_motor import (  # noqa: E402
    joint_limit_issue,
    joint_segments,
)


@pytest.mark.parametrize('fixed', [False, True])
def test_remote_recovery_plans_a_reachable_clearance_from_original_jaw_contact(tmp_path, fixed):
    from arm_control.push_t import PushSimulation, pose
    from arm_control.push_t_local import LocalPush

    source = ROOT/'data/push_t/measured'
    if not source.is_dir():
        options = sorted((ROOT/'data/push_t').glob('remote-*/initial_observation'))
        assert options, 'Capture a remote live observation before testing its clearance paths'
        source = options[-1]
    sim = PushSimulation(snapshot=source, output=tmp_path/'sim')
    action = sim.plan()
    assert action is not None
    sim.execute(action)
    state = copy.deepcopy(sim.state)
    wood = pose(sim.data)
    state['objects']['wood_t'].update(xy=wood[:2].tolist(), yaw=float(wood[2]))
    state['robot']['qpos'] = sim.data.qpos[sim.qadr].tolist()
    state['robot']['registration'].update(calibrated=True, status='tracked', pose_age_s=0,
                                         pose_source='known_marker_mounts')
    if fixed:
        state['robot']['registration'].update(status='fixed', fixed_to_table=True,
                                             pose_source='fixed_marker_calibration')
    state['capture_age_at_send_ms'] = 20
    state['robot']['sample_age_at_send_ms'] = 20
    hidden = copy.deepcopy(state)
    hidden['marker_visible'] = False
    hidden['objects'] = {}
    submitted = []
    robot = SimpleNamespace(motor=SimpleNamespace(
        cancel=lambda reason: None, status=lambda: {'phase': 'stopped'},
        submit=lambda reference, segments, **kw: submitted.append((reference, segments, kw))))
    coordinator = LocalPush.__new__(LocalPush)
    coordinator.sim, coordinator.robot = sim, robot
    coordinator.lock = threading.Lock()
    coordinator.stopped = threading.Event()
    coordinator.output = tmp_path
    coordinator.last_visible = (state, time.monotonic())
    coordinator._latest = lambda: (hidden, time.monotonic(), True, 0)
    assert coordinator._recover(hidden, time.monotonic())
    assert submitted[0][2]['recovery']
    assert sum(s['seconds'] for s in submitted[0][1]) <= 7
    final = submitted[0][1][-1]['end']
    sim.arm.data.qpos[sim.qadr] = final
    import mujoco

    mujoco.mj_forward(sim.model, sim.arm.data)
    assert sim.arm.data.site('push_contact').xpos[2] > .025


def test_joint_preflight_names_violations_without_clipping_feedback():
    limits = np.array([[-1., 1.]]*6)
    measured = [0.]*6
    measured[1] = -1.01
    measured[5] = 1.02
    issue = joint_limit_issue(measured, limits)
    assert 'shoulder_lift' in issue and 'gripper' in issue
    assert measured == [0., -1.01, 0., 0., 0., 1.02]
    measured[1] = measured[5] = 0.
    assert joint_limit_issue(measured, limits) is None


def test_height_compensation_raises_command_path_once_and_keeps_nominal_prediction(tmp_path):
    import mujoco

    from arm_control.push_t import PushSimulation
    from arm_control.push_t_local import LocalPush

    sim = PushSimulation(output=tmp_path/'sim')
    action = sim.plan()
    assert action is not None
    initial = sim.data.qpos[sim.qadr].copy()
    original = np.array(action.joints).copy()
    state = sim.data.qpos.copy()
    coordinator = LocalPush.__new__(LocalPush)
    coordinator.sim, coordinator.height_compensation = sim, .005
    raised = coordinator._execution_action(action, initial)
    repeated = coordinator._execution_action(action, initial)
    np.testing.assert_array_equal(action.joints, original)
    np.testing.assert_array_equal(sim.data.qpos, state)
    np.testing.assert_allclose(raised.joints, repeated.joints, atol=1e-10)
    nominal = mujoco.MjData(sim.model)
    actual = mujoco.MjData(sim.model)
    for before, after in zip(action.joints, raised.joints):
        nominal.qpos[sim.qadr], actual.qpos[sim.qadr] = before, after
        mujoco.mj_forward(sim.model, nominal)
        mujoco.mj_forward(sim.model, actual)
        np.testing.assert_allclose(actual.site('push_contact').xpos,
                                   nominal.site('push_contact').xpos+[0, 0, .005], atol=.001)
    segments = joint_segments(initial, raised, sim.arm.limits)
    np.testing.assert_allclose(segments[0]['start'], initial)
    coordinator.height_compensation = 0
    assert coordinator._execution_action(action, initial) is action
    coordinator.height_compensation = 1
    with pytest.raises(ValueError, match='unreachable or in collision'):
        coordinator._execution_action(action, initial)


def test_remote_worker_keeps_observing_invalid_seed_then_plans_after_it_recovers(
        observed, tmp_path, monkeypatch):
    from arm_control.push_t_local import LocalPush

    planned = []
    action = Push(np.zeros(2), np.ones(2), .002, joints=[[.32]*6], approach_count=0)
    sim = SimpleNamespace(arm=SimpleNamespace(limits=np.array([[-1., 1.]]*6)),
                          initial=np.zeros(3), valid_pose=lambda pose: True,
                          plan=lambda: planned.append(True) or action)
    monkeypatch.setattr(LocalPush, '_prepare', lambda self, state: setattr(self, 'sim', sim))
    monkeypatch.setattr(LocalPush, '_update_model', lambda self, state: None)
    observed.update(sequence=1, captured_unix_ns=1, appearance_pngs={})
    observed['robot']['qpos'][1] = -1.01
    coordinator = LocalPush({}, SimpleNamespace(motor=None), tmp_path/'remote', b'')

    def wait_for(phase):
        deadline = time.monotonic()+3
        while coordinator.status()['phase'] != phase:
            assert coordinator.worker.is_alive(), coordinator.status()
            assert time.monotonic() < deadline, coordinator.status()
            coordinator.observe(observed)
            time.sleep(.01)

    try:
        coordinator.observe(observed)
        wait_for('waiting for valid joint pose')
        assert not planned
        assert 'shoulder_lift' in coordinator.status()['reason']
        observed['robot']['qpos'][1] = .3
        observed['sequence'] = 2
        coordinator.observe(observed)
        wait_for('preview ready')
        assert planned and coordinator.worker.is_alive()
        assert (tmp_path/'remote'/'preview.json').exists()
    finally:
        coordinator.close()


@pytest.mark.parametrize('scene_moved', [False, True])
def test_start_reuses_preview_only_while_latest_scene_still_matches(
        observed, tmp_path, monkeypatch, scene_moved):
    from arm_control.push_t_local import LocalPush

    planned, submitted = [], []
    action = Push(np.zeros(2), np.ones(2), .002, joints=[[.32]*6], approach_count=0)
    sim = SimpleNamespace(arm=SimpleNamespace(limits=np.array([[-1., 1.]]*6)),
                          initial=np.zeros(3), valid_pose=lambda pose: True,
                          plan=lambda: planned.append(True) or action)
    monkeypatch.setattr(LocalPush, '_prepare', lambda self, state: setattr(self, 'sim', sim))
    monkeypatch.setattr(LocalPush, '_update_model', lambda self, state: None)
    motor = SimpleNamespace(observe=lambda *args: None, cancel=lambda reason: None,
                            status=lambda: {'phase': 'running' if submitted else 'idle'},
                            submit=lambda *args: submitted.append(args))
    observed.update(sequence=1, captured_unix_ns=1, appearance_pngs={})
    observed['robot']['registration'].update(status='fixed', calibrated=True,
                                            fixed_to_table=True,
                                            pose_source='fixed_marker_calibration')
    coordinator = LocalPush({}, SimpleNamespace(motor=motor), tmp_path/'remote', b'', execute=True)

    def wait_for(predicate):
        deadline = time.monotonic()+3
        while not predicate():
            assert coordinator.worker.is_alive(), coordinator.status()
            assert time.monotonic() < deadline, coordinator.status()
            coordinator.observe(observed)
            time.sleep(.01)

    try:
        coordinator.observe(observed)
        wait_for(lambda: coordinator.status()['phase'] == 'preview ready')
        assert len(planned) == 1
        if scene_moved:
            observed['objects']['wood_t']['xy'][0] += .01
        coordinator.observe(observed)
        coordinator.command('push_start')
        wait_for(lambda: bool(submitted))
        assert len(planned) == (2 if scene_moved else 1)
        lines = (tmp_path/'remote'/'plans.jsonl').read_text().splitlines()
        plans = [json.loads(line) for line in lines]
        if not scene_moved:
            assert plans[-1]['reused_preview'] is True
        else:
            assert any(not plan['reused_preview'] and plan['objects'] == observed['objects']
                       for plan in plans)
        assert submitted[0][0]['objects'] == observed['objects']
    finally:
        coordinator.close()


@pytest.mark.parametrize('prediction', [False, True])
def test_free_approach_never_shortcuts_actual_execution_or_initial_contact(tmp_path, prediction):
    import mujoco

    from arm_control.push_t_local import ShortPushSimulation

    source = ROOT/'data/push_t/measured'
    if not source.is_dir():
        source = sorted((ROOT/'data/push_t').glob('remote-*/initial_observation'))[-1]
    sim = ShortPushSimulation(source, tmp_path/'sim')
    seed = np.clip(sim.data.ctrl, sim.arm.limits[:, 0]+.02, sim.arm.limits[:, 1]-.02)
    sim.data.qpos[sim.qadr] = sim.data.ctrl[:] = seed
    mujoco.mj_forward(sim.model, sim.data)
    action = Push(np.zeros(2), np.ones(2), 0, joints=[seed+.001], approach_count=1)
    if not prediction:
        def fail_shortcut(*args):
            pytest.fail('actual execution must run complete contact dynamics')
        sim.free_approach = fail_shortcut
        sim.play_action(action, sim.model, sim.data)
        assert sim.data.time > .1
    else:
        mujoco.mj_copyData(sim.roll_data, sim.roll_model, sim.data)
        site = sim.roll_data.site('push_contact').xpos.copy()
        sim.roll_data.qpos[sim.wood_qadr:sim.wood_qadr+3] = site
        mujoco.mj_forward(sim.roll_model, sim.roll_data)
        wood = sim.roll_model.body('wood_t').id
        assert any(wood in (sim.roll_model.geom_bodyid[c.geom1],
                            sim.roll_model.geom_bodyid[c.geom2])
                   and 0 not in (sim.roll_model.geom_bodyid[c.geom1],
                                 sim.roll_model.geom_bodyid[c.geom2])
                   for c in sim.roll_data.contact)
        before = sim.roll_data.qpos.copy()
        segments = joint_segments(seed, action, sim.arm.limits)
        assert sim.free_approach(1, segments, sim.roll_model, sim.roll_data) == 0
        np.testing.assert_array_equal(sim.roll_data.qpos, before)
