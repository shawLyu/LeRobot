import mujoco
import numpy as np

from lerobot_arm_control.control.control_defaults import POSITION_FEEDBACK, SIMULATION_ACTUATOR
from lerobot_arm_control.control.simulation_control import SimulationControl
from lerobot_arm_control.robot.so101.model_metadata import MODEL


def test_simulation_pid_updates_at_shared_rate_and_resets_on_time_rewind():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = [0, -.5, .8, -.3, 0, .3]
    control = SimulationControl(model, data)
    assert control.feedback.profile == POSITION_FEEDBACK
    np.testing.assert_array_equal(model.actuator_gainprm[:, 0], SIMULATION_ACTUATOR['kp'])
    np.testing.assert_array_equal(model.actuator_biasprm[:, 2], -SIMULATION_ACTUATOR['kv'])
    reference = data.qpos.copy()
    reference[0] += .1
    ticks = []
    for _ in range(200):
        previous = control.last_control
        control.step(reference)
        if control.last_control != previous:
            ticks.append(control.last_control)
    assert len(ticks) == 60
    assert np.max(np.diff(ticks)) <= .0200001
    mujoco.mj_resetData(model, data)
    control.step([0.]*6)
    assert control.feedback.integral['shoulder_pan'] == 0
    assert control.feedback.derivative['shoulder_pan'] == 0


def test_rollout_clones_control_state_without_sharing_memory():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data, rollout = mujoco.MjData(model), mujoco.MjData(model)
    a, b = SimulationControl(model, data), SimulationControl(model, rollout)
    for _ in range(20):
        a.step([.2]*6)
    mujoco.mj_copyData(rollout, model, data)
    b.copy_state_from(a)
    assert a.feedback.integral is not b.feedback.integral
    assert a.feedback.integral == b.feedback.integral
    for _ in range(30):
        a.step([.2]*6)
        b.step([.2]*6)
    np.testing.assert_array_equal(data.qpos, rollout.qpos)
    np.testing.assert_array_equal(data.ctrl, rollout.ctrl)


def two_joint_model():
    return mujoco.MjModel.from_xml_string('''<mujoco><compiler angle="radian"/>
      <option timestep="0.002" gravity="0 0 0" integrator="implicitfast"/>
      <default><joint armature=".05" damping="1"/></default>
      <worldbody><body><joint name="base" range="-1 1"/><geom type="sphere" size=".05"/>
        <body pos="0 0 .2"><joint name="wrist" range="-.5 .5"/>
          <geom type="sphere" size=".05"/></body></body></worldbody>
      <actuator><position joint="base" ctrlrange="-1 1"/>
        <position joint="wrist" ctrlrange="-.5 .5"/></actuator></mujoco>''')


def test_other_robot_joint_count_and_sampling_rate_are_explicit():
    model = two_joint_model()
    data = mujoco.MjData(model)
    profile = {'version': 1, 'joints': {j: {'kp': .4, 'ki': .5, 'max_correction_rad': .1,
                'correction_rate_rad_s': .1} for j in ('base', 'wrist')}}
    c = SimulationControl(model, data, joints=('base', 'wrist'), feedback_profile=profile,
                          control_hz=50, actuator_settings={'kp': 100, 'kv': 10})
    ticks = []
    for _ in range(200):
        before = c.last_control
        c.step([.2, -.1])
        if c.last_control != before:
            ticks.append(c.last_control)
    assert len(ticks) == 20
    assert set(c.feedback.integral) == {'base', 'wrist'}
    np.testing.assert_allclose(np.diff(ticks), .02)
    assert np.isfinite(data.qpos).all() and data.qpos[0] > 0 and data.qpos[1] < 0


def test_alternative_controller_cannot_bypass_simulated_actuator_limits(monkeypatch):
    from lerobot_arm_control.control import factory

    class Alternative:
        def __init__(self, profile, limits): self.profile = profile
        def reset(self, joints=None): pass
        def finish_reference(self, joints): pass
        def status(self): return {}
        def target(self, *args): return 1000.
    monkeypatch.setattr(factory, '_CONTROLLERS', {**factory._CONTROLLERS, 'alternate': Alternative})
    model = two_joint_model()
    data = mujoco.MjData(model)
    c = SimulationControl(model, data, joints=('base', 'wrist'),
                          feedback_profile={'controller': 'alternate'})
    c.step([.1, .1])
    np.testing.assert_array_equal(data.ctrl, [1., .5])


def test_invalid_alternative_output_does_not_step_the_simulation(monkeypatch):
    import pytest

    from lerobot_arm_control.control import factory

    class Invalid:
        def __init__(self, profile, limits): self.profile = profile
        def reset(self, joints=None): pass
        def finish_reference(self, joints): pass
        def status(self): return {}
        def target(self, *args): return float('nan')
    monkeypatch.setattr(factory, '_CONTROLLERS', {**factory._CONTROLLERS, 'invalid': Invalid})
    model = two_joint_model()
    data = mujoco.MjData(model)
    c = SimulationControl(model, data, joints=('base', 'wrist'),
                          feedback_profile={'controller': 'invalid'})
    with pytest.raises(ValueError, match='non-finite'):
        c.step([0, 0])
    assert data.time == 0
    np.testing.assert_array_equal(data.ctrl, [0, 0])
