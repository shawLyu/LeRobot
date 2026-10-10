"""Public runtime API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "Link": ("runtime.realtime_client", "Link"),
    "TableLink": ("runtime.tabletop_client", "TableLink"),
    "RobotFeedback": ("runtime.tabletop_robot", "RobotFeedback"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
