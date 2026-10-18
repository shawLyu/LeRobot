"""Run the same joint PID on local MuJoCo position actuators at the shared rate."""

import math
from copy import deepcopy
from numbers import Real

from lerobot_arm_control.control.control_defaults import (
    CONTROL_HZ,
    POSITION_FEEDBACK,
    SIMULATION_ACTUATOR,
)
from lerobot_arm_control.control.factory import create_controller
from lerobot_arm_control.robot.so101.joints import JOINTS


class SimulationControl:
    def __init__(self, model, data, *, joints=JOINTS, feedback_profile=None,
                 control_hz=CONTROL_HZ, actuator_settings=None):
        self.model, self.data = model, data
        self.joints = tuple(joints)
        if (not self.joints or len(set(self.joints)) != len(self.joints)
                or any(not isinstance(j, str) or not j for j in self.joints)
                or type(control_hz) not in (int, float) or not math.isfinite(control_hz)
                or control_hz <= 0):
            raise ValueError('simulation requires distinct joint names and a positive control rate')
        self.control_hz = control_hz
        if any(model.joint(j).type[0] != 3 for j in self.joints):
            raise ValueError('angular control requires scalar hinge joints')
        self.qadr = [int(model.joint(j).qposadr[0]) for j in self.joints]
        self.actuators = []
        for joint in self.joints:
            matching = [a for a in range(model.nu) if model.actuator_trntype[a] == 0
                        and model.actuator_trnid[a, 0] == model.joint(joint).id]
            if len(matching) != 1:
                raise ValueError(f'{joint} requires one joint position actuator')
            self.actuators.append(matching[0])
        # Use one explicit approximation of the native inner loop in every
        # scene. This changes compiled dynamics, never the calibrated MJCF file.
        # kv/kp approximates the measured low-frequency servo lag. An idealized
        # fast actuator would turn the shared velocity lead into physical overshoot.
        settings = SIMULATION_ACTUATOR if actuator_settings is None else actuator_settings
        kp, kv = settings['kp'], settings['kv']
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (kp, kv))
                or kp <= 0 or kv < 0):
            raise ValueError('invalid simulated position actuator gains')
        for a in self.actuators:
            model.actuator_gainprm[a, 0] = kp
            model.actuator_biasprm[a, 1:3] = (-kp, -kv)
        self.limits = {j: tuple(model.actuator_ctrlrange[a])
                       for j, a in zip(self.joints, self.actuators, strict=True)}
        self.feedback = create_controller(
            POSITION_FEEDBACK if feedback_profile is None else feedback_profile, self.limits)
        self.reset()

    def reset(self, reference=None):
        self.feedback.reset()
        self.reference = list(self.data.qpos[self.qadr] if reference is None else reference)
        self.next_control = float(self.data.time)
        self.last_control = self.next_control-1/self.control_hz
        self.last_step = None

    def copy_state_from(self, other):
        """A physics rollout must also clone PID memory and the sampling clock."""
        if (self.joints != other.joints or self.limits != other.limits
                or self.control_hz != other.control_hz):
            raise ValueError('rollout controller model or timing differs')
        self.feedback = deepcopy(other.feedback)
        self.reference = list(other.reference)
        self.next_control = other.next_control
        self.last_control = other.last_control
        self.last_step = other.last_step

    def step(self, reference):
        import mujoco

        if len(reference) != len(self.joints) or not all(math.isfinite(v) for v in reference):
            raise ValueError('simulation requires one finite target per joint')
        # Rollout copies/resetData must not inherit a previous trial's integral.
        now = float(self.data.time)
        if self.last_step is not None and now <= self.last_step:
            self.reset()
        self.reference = [max(self.limits[j][0], min(self.limits[j][1], float(q)))
                          for j, q in zip(self.joints, reference, strict=True)]
        if now+1e-10 >= self.next_control:
            for j, q, address, actuator in zip(
                    self.joints, self.reference, self.qadr, self.actuators, strict=True):
                target = self.feedback.target(
                    j, q, float(self.data.qpos[address]), now-self.last_control)
                if (not isinstance(target, Real) or isinstance(target, bool)
                        or not math.isfinite(target)):
                    raise ValueError('controller returned a non-finite position target')
                self.data.ctrl[actuator] = max(self.limits[j][0], min(self.limits[j][1], target))
            self.last_control = now
            self.next_control += (max(1, math.floor((now-self.next_control)*self.control_hz)+1)
                                  /self.control_hz)
        self.last_step = now
        mujoco.mj_step(self.model, self.data)
