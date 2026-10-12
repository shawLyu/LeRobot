"""Run the same joint PID on local MuJoCo position actuators at the shared rate."""

import math
from copy import deepcopy

from lerobot_arm_control.control.control_defaults import (
    CONTROL_HZ,
    POSITION_FEEDBACK,
    SIMULATION_ACTUATOR,
)
from lerobot_arm_control.control.position_feedback import PositionFeedback
from lerobot_arm_control.transport.realtime import JOINTS


class SimulationControl:
    def __init__(self, model, data):
        self.model, self.data = model, data
        self.qadr = [int(model.joint(j).qposadr[0]) for j in JOINTS]
        self.actuators = []
        for joint in JOINTS:
            matching = [a for a in range(model.nu) if model.actuator_trntype[a] == 0
                        and model.actuator_trnid[a, 0] == model.joint(joint).id]
            if len(matching) != 1:
                raise ValueError(f'{joint} requires one joint position actuator')
            self.actuators.append(matching[0])
        # Use one explicit approximation of the native inner loop in every
        # scene. This changes compiled dynamics, never the calibrated MJCF file.
        # kv/kp approximates the measured low-frequency servo lag. An idealized
        # fast actuator would turn the shared velocity lead into physical overshoot.
        kp, kv = SIMULATION_ACTUATOR['kp'], SIMULATION_ACTUATOR['kv']
        for a in self.actuators:
            model.actuator_gainprm[a, 0] = kp
            model.actuator_biasprm[a, 1:3] = (-kp, -kv)
        self.limits = {j: tuple(model.actuator_ctrlrange[a])
                       for j, a in zip(JOINTS, self.actuators, strict=True)}
        self.feedback = PositionFeedback(POSITION_FEEDBACK, self.limits)
        self.reset()

    def reset(self, reference=None):
        self.feedback.reset()
        self.reference = list(self.data.qpos[self.qadr] if reference is None else reference)
        self.next_control = float(self.data.time)
        self.last_control = self.next_control-1/CONTROL_HZ
        self.last_step = None

    def copy_state_from(self, other):
        """A physics rollout must also clone PID memory and the sampling clock."""
        self.feedback = deepcopy(other.feedback)
        self.reference = list(other.reference)
        self.next_control = other.next_control
        self.last_control = other.last_control
        self.last_step = other.last_step

    def step(self, reference):
        import mujoco

        if len(reference) != 6 or not all(math.isfinite(v) for v in reference):
            raise ValueError('simulation requires six finite joint targets')
        # Rollout copies/resetData must not inherit a previous trial's integral.
        now = float(self.data.time)
        if self.last_step is not None and now <= self.last_step:
            self.reset()
        self.reference = [max(self.limits[j][0], min(self.limits[j][1], float(q)))
                          for j, q in zip(JOINTS, reference, strict=True)]
        if now+1e-10 >= self.next_control:
            for j, q, address, actuator in zip(
                    JOINTS, self.reference, self.qadr, self.actuators, strict=True):
                self.data.ctrl[actuator] = self.feedback.target(
                    j, q, float(self.data.qpos[address]), now-self.last_control)
            self.last_control = now
            self.next_control += (max(1, math.floor((now-self.next_control)*CONTROL_HZ)+1)
                                  /CONTROL_HZ)
        self.last_step = now
        mujoco.mj_step(self.model, self.data)
