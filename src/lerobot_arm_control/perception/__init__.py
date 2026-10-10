"""Public perception API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "Perception": ("perception.tabletop_perception", "Perception"),
    "TTracker": ("perception.tabletop_tracking", "TTracker"),
    "ToolMarker": ("perception.tool_marker", "ToolMarker"),
    "BaseRegistration": ("perception.tabletop_registration", "BaseRegistration"),
    "read_config": ("perception.tabletop_perception", "read_config"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
