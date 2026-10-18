"""Position-controller contract and explicit selection shared by hardware and simulation."""

from collections.abc import Callable
from copy import deepcopy
from typing import Protocol, runtime_checkable

from lerobot_arm_control.control.position_feedback import PositionFeedback


@runtime_checkable
class JointController(Protocol):
    profile: dict

    def reset(self, joints=None): ...
    def finish_reference(self, joints): ...
    def target(self, joint, reference, measured, elapsed): ...
    def status(self): ...


_CONTROLLERS: dict[str, Callable] = {'pid': PositionFeedback}


def register_controller(name, factory):
    """Register a profile/limits factory in the application bootstrap, without I/O."""
    if not isinstance(name, str) or not name or name in _CONTROLLERS or not callable(factory):
        raise ValueError('invalid or already registered joint controller')
    _CONTROLLERS[name] = factory


def create_controller(profile, limits):
    if not isinstance(profile, dict):
        raise ValueError('invalid joint controller profile')
    name = profile.get('controller', 'pid')
    if not isinstance(name, str) or name not in _CONTROLLERS:
        raise ValueError(f'unknown joint controller: {name!r}; register it before construction')
    controller = _CONTROLLERS[name](deepcopy(profile), limits)
    if (not isinstance(controller, JointController) or not isinstance(controller.profile, dict)
            or any(not callable(getattr(controller, method, None)) for method in
                   ('reset', 'finish_reference', 'target', 'status'))):
        raise TypeError('joint controller must implement profile, reset, finish_reference, '
                        'target, status')
    return controller
