"""Explicit adapters for LeRobot-compatible motor buses; registration performs no I/O."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass

from lerobot_arm_control.robot import feetech


@dataclass(frozen=True)
class MotorBackend:
    factory: Callable
    validate: Callable
    model_numbers: Mapping[str, int]
    ensure_parameters: Callable | None = None

    def check(self, arm):
        if arm.model not in self.model_numbers:
            raise ValueError(f'{arm.name}: unsupported motor model {arm.model!r}')
        if (not isinstance(arm.port, str) or not arm.port
                or type(arm.baudrate) is not int or arm.baudrate <= 0
                or not isinstance(arm.motors, dict) or not arm.motors
                or any(not isinstance(j, str) or not j or type(i) is not int
                       for j, i in arm.motors.items())):
            raise ValueError(f'{arm.name}: invalid port, baudrate or motor IDs')
        if len(set(arm.motors.values())) != len(arm.motors):
            raise ValueError(f'{arm.name}: duplicate motor IDs')
        self.validate(arm)


def _ensure_feetech(bus, arm):
    from lerobot_arm_control.robot.parameters import ensure_parameters

    return ensure_parameters(bus, arm)


_BACKENDS = {'feetech': MotorBackend(feetech.make_bus, feetech.validate_arm,
                                     {'sts3215': 777}, _ensure_feetech)}


def register_backend(name, backend):
    """Register explicitly in the application bootstrap before loading arm configs."""
    if (not isinstance(name, str) or not name or name in _BACKENDS
            or not isinstance(backend, MotorBackend)
            or not callable(backend.factory) or not callable(backend.validate)
            or not backend.model_numbers
            or any(not isinstance(k, str) or not k or type(v) is not int or v <= 0
                   for k, v in backend.model_numbers.items())):
        raise ValueError('invalid or already registered motor backend')
    _BACKENDS[name] = backend


def get_backend(name):
    try:
        return _BACKENDS[name]
    except KeyError:
        raise ValueError(f'unknown motor backend: {name!r}; '
                         'register it before loading arms') from None
