"""Load explicit USB port and motor mappings."""

import tomllib
from dataclasses import dataclass
from pathlib import Path

from lerobot_arm_control.core.paths import workspace_root

PROJECT_ROOT = workspace_root()
DEFAULT_CONFIG = PROJECT_ROOT / "configs/arms.toml"


@dataclass(frozen=True)
class ArmConfig:
    name: str
    port: str
    role: str
    model: str
    baudrate: int
    motors: dict[str, int]


def load_arms(path: Path | None = None) -> dict[str, ArmConfig]:
    if path is None:
        path = workspace_root() / "configs/arms.toml"
    with path.open("rb") as stream:
        data = tomllib.load(stream)
    arms = {}
    for name, values in data["arms"].items():
        arm = ArmConfig(name=name, **values)
        ids = list(arm.motors.values())
        if arm.model != "sts3215" or arm.baudrate != 1_000_000:
            raise ValueError(f"{name}: this project supports STS3215 at 1,000,000 baud")
        if not ids or any(type(i) is not int or not 1 <= i <= 253 for i in ids):
            raise ValueError(f"{name}: motor IDs must be integers in 1..253")
        if len(set(ids)) != len(ids):
            raise ValueError(f"{name}: duplicate motor IDs")
        if not arm.port.startswith("/dev/serial/by-id/"):
            raise ValueError(f"{name}: use a stable /dev/serial/by-id/ port")
        arms[name] = arm
    if not arms:
        raise ValueError("No arms configured")
    ports = [arm.port for arm in arms.values()]
    if len(set(ports)) != len(ports):
        raise ValueError("Two configured arms must not share a USB port")
    return arms
