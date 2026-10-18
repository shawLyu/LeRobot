"""Load explicit USB port and motor mappings."""

import tomllib
from dataclasses import dataclass
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


def load_arms(path: Path | None = None) -> dict[str, ArmConfig]:
    from lerobot_arm_control.robot.backends import get_backend

    if path is None:
        path = workspace_root() / "configs/arms.toml"
    with path.open("rb") as stream:
        data = tomllib.load(stream)
    arms = {}
    for name, values in data["arms"].items():
        arm = ArmConfig(name=name, **values)
        get_backend(arm.backend).check(arm)
        arms[name] = arm
    if not arms:
        raise ValueError("No arms configured")
    ports = [arm.port for arm in arms.values()]
    if len(set(ports)) != len(ports):
        raise ValueError("Two configured arms must not share a USB port")
    return arms
