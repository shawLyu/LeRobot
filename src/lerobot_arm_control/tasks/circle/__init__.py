"""Public tasks.circle API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "Circle": ("tasks.circle.circle_trajectory", "Circle"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
