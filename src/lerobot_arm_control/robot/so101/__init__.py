"""Public robot.so101 API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "connected": ("robot.so101.driver", "connected"),
    "read_state": ("robot.so101.driver", "read_state"),
    "joint_ranges": ("robot.so101.model_metadata", "joint_ranges"),
    "MODEL": ("robot.so101.model_metadata", "MODEL"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
