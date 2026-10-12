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
