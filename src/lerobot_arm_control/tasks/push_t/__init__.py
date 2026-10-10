"""Public tasks.push_t API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "LocalPush": ("tasks.push_t.push_t_local", "LocalPush"),
    "PushMotor": ("tasks.push_t.push_t_motor", "PushMotor"),
    "PushReceiver": ("tasks.push_t.push_t_receiver", "PushReceiver"),
    "capture_snapshot": ("tasks.push_t.push_t_capture", "capture_snapshot"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
