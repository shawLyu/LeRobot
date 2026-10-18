"""An arm owns its connection, saved parameters and the active controller."""

import hashlib
from contextlib import contextmanager

from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.robot.backends import get_backend


class RobotArm:
    def __init__(self, config):
        self.config = config
        self.bus = None
        self.controller = None
        self.startup_report = None

    @classmethod
    def from_name(cls, name):
        return cls(load_arms()[name])

    def check_resources(self):
        for name, reference in self.config.resources.items():
            path = resource_path(reference['path'])
            if hashlib.sha256(path.read_bytes()).hexdigest() != reference['sha256']:
                raise ValueError(f'{self.config.name}: {name} differs from saved robot config')

    def check_parameters(self):
        self.check_resources()
        if self.config.parameters is not None:
            manager = get_backend(self.config.backend).ensure_parameters
            if manager is None:
                raise ValueError(f'{self.config.backend}: no hardware parameter adapter registered')
            self.startup_report = manager(self.bus, self.config)
        return self.startup_report

    def attach(self, bus, *, allow_unverified=False):
        self.bus = bus
        bus.robot_arm = self
        try:
            self.check_parameters()
        except RuntimeError as exc:
            if not allow_unverified:
                raise
            # Inspection/release remain available; Workbench latches this error
            # and refuses enable until an explicit clear rechecks the baseline.
            self.startup_report = {'arm': self.config.name, 'checked': [],
                                   'missing': [j for j in self.config.motors
                                               if j not in bus.motors],
                                   'restored': [], 'error': str(exc)}
        return self

    def create_controller(self, calibration, raw_limits, **options):
        from lerobot_arm_control.control.manual_control import ManualControl

        if self.bus is None:
            raise RuntimeError('connect the arm before creating a controller')
        if self.controller is not None:
            raise RuntimeError('the arm already has a controller')
        if self.startup_report and self.startup_report.get('error'):
            self.check_parameters()
        self.controller = ManualControl(self.bus, calibration, raw_limits, **options)
        return self.controller

    @contextmanager
    def connect(self, *, allow_alarm=False):
        from lerobot_arm_control.robot.connection import connected

        if self.bus is not None:
            raise RuntimeError('the arm is already connected')
        try:
            with connected(self.config, allow_alarm=allow_alarm, robot=self):
                try:
                    yield self
                finally:
                    if self.controller is not None:
                        self.controller.close()
        finally:
            self.bus = None
            self.controller = None
