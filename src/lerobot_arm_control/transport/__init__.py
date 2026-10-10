"""Public transport API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "MotorLink": ("transport.push_t_link", "MotorLink"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
