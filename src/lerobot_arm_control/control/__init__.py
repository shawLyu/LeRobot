"""Public control API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "ManualControl": ("control.manual_control", "ManualControl"),
    "PositionFeedback": ("control.position_feedback", "PositionFeedback"),
    "JointPath": ("control.joint_path", "JointPath"),
    "Parking": ("control.parking", "Parking"),
    "plan_motion": ("control.motion", "plan_motion"),
    "execute_motion": ("control.motion", "execute_motion"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
