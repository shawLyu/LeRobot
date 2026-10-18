"""Robot metadata and motor adapters; imports never open hardware."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    'RobotArm': ('robot.arm', 'RobotArm'),
    'RobotModel': ('robot.model', 'RobotModel'),
    'MotorBackend': ('robot.backends', 'MotorBackend'),
    'register_backend': ('robot.backends', 'register_backend'),
    'connected': ('robot.connection', 'connected'),
    'read_state': ('robot.connection', 'read_state'),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
