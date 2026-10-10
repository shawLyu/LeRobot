"""Public planning API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "PushSimulation": ("planning.push_t", "PushSimulation"),
    "PushPlanner": ("planning.push_t", "PushPlanner"),
    "PickPlanner": ("planning.pick_t", "PickPlanner"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
