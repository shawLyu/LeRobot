"""The robot configuration owns connections, hardware baselines and control profiles."""

import json
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from lerobot_arm_control.core.paths import workspace_root


@dataclass(frozen=True)
class ArmConfig:
    name: str
    port: str
    role: str
    model: str
    baudrate: int
    motors: dict[str, int]
    backend: str = 'feetech'
    parameters: dict | None = field(default=None, repr=False)
    resources: dict = field(default_factory=dict, repr=False)


def load_robot_config(path: Path | None = None):
    path = path or workspace_root()/'configs/robot.json'
    config = json.loads(path.read_text())
    if config.get('version') != 1:
        raise ValueError('unsupported robot configuration version')
    # Local connection overrides do not replace measured parameters or controllers.
    if path == workspace_root()/'configs/robot.json':
        override = path.with_name('robot.local.json')
        if override.exists():
            local = json.loads(override.read_text())
            for name, values in local['arms'].items():
                if set(values)-{'port'}:
                    raise ValueError('robot.local.json may override USB ports only')
                config['arms'][name].update(values)
    return config


def load_arms(path: Path | None = None) -> dict[str, ArmConfig]:
    from lerobot_arm_control.robot.backends import get_backend

    if path is not None and path.suffix == '.toml':
        with path.open('rb') as stream:
            data = tomllib.load(stream)
    else:
        data = load_robot_config(path)
    arms = {}
    for name, values in data['arms'].items():
        arm = ArmConfig(name=name, **values)
        get_backend(arm.backend).check(arm)
        if arm.parameters is not None and set(arm.parameters) != set(arm.motors):
            raise ValueError(f'{name}: hardware baseline must cover every configured motor')
        arms[name] = arm
    if not arms:
        raise ValueError('No arms configured')
    ports = [str(Path(arm.port).resolve()) for arm in arms.values()]
    if len(set(ports)) != len(ports):
        raise ValueError('Two configured arms must not share a USB port')
    return arms
