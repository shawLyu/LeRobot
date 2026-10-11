"""Preview or run upstream SO-101 Leader/Follower teleoperation on the USB host."""

import argparse
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
from contextlib import ExitStack
from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from lerobot_arm_control.core.config import ArmConfig, load_arms

MOTORS = dict(zip(
    ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"),
    range(1, 7), strict=True,
))


def calibration_root():
    """Match LeRobot's cache selection without importing hardware dependencies."""
    if "HF_LEROBOT_CALIBRATION" in os.environ:
        return Path(os.environ["HF_LEROBOT_CALIBRATION"]).expanduser()
    cache = Path(os.environ.get("XDG_CACHE_HOME", "~/.cache")).expanduser()
    hf_home = Path(os.environ.get("HF_HOME", str(cache / "huggingface"))).expanduser()
    home = Path(os.environ.get("HF_LEROBOT_HOME", str(hf_home / "lerobot"))).expanduser()
    return home / "calibration"


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def device_id(value):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", value):
        raise argparse.ArgumentTypeError("device ID must contain only letters, digits, _ or -")
    return value


def parser():
    command = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    command.add_argument("--config", type=Path, help="arms TOML; defaults to workspace config")
    command.add_argument("--leader", required=True, help="confirmed leader entry in arms TOML")
    command.add_argument("--follower", default="follower", help="follower entry in arms TOML")
    command.add_argument("--leader-id", required=True, type=device_id)
    command.add_argument("--follower-id", required=True, type=device_id)
    command.add_argument("--calibration-root", type=Path, default=calibration_root())
    command.add_argument("--fps", type=int, default=30)
    command.add_argument("--duration", type=positive, help="seconds; omit to run until Ctrl+C")
    command.add_argument("--max-relative-target", type=positive, default=1.0,
                         help="per-command limit: joint degrees / gripper percentage points")
    command.add_argument("--execute", action="store_true", help="open devices and move follower")
    return command


@dataclass(frozen=True)
class Plan:
    leader: ArmConfig
    follower: ArmConfig
    calibration_files: tuple[Path, Path]
    command: list[str]


def build_plan(args):
    arms = load_arms(args.config)
    if args.leader == args.follower:
        raise ValueError("Leader and Follower must be different arm entries")
    selected = []
    for name, role in ((args.leader, "leader"), (args.follower, "follower")):
        if name not in arms:
            raise ValueError(f"Unknown {role} entry: {name}")
        arm = arms[name]
        if arm.role != role:
            raise ValueError(f"{name}: role is {arm.role!r}; "
                             f"confirm hardware before setting {role!r}")
        if arm.motors != MOTORS:
            raise ValueError(f"{name}: upstream SO-101 requires the standard joint IDs 1-6")
        selected.append(arm)
    leader, follower = selected
    if Path(leader.port).resolve() == Path(follower.port).resolve():
        raise ValueError("Leader and Follower resolve to the same USB device")
    if args.leader_id == args.follower_id:
        raise ValueError("Use distinct Leader and Follower IDs to identify calibration files")
    if not 1 <= args.fps <= 60:
        raise ValueError("fps must be in 1..60")
    root = args.calibration_root.expanduser().resolve()
    leader_dir = root / "teleoperators/so_leader"
    follower_dir = root / "robots/so_follower"
    command = [
        sys.executable, "-m", "lerobot.scripts.lerobot_teleoperate",
        "--teleop.type=so101_leader", f"--teleop.port={leader.port}",
        f"--teleop.id={args.leader_id}", f"--teleop.calibration_dir={leader_dir}",
        "--teleop.use_degrees=true",
        "--robot.type=so101_follower", f"--robot.port={follower.port}",
        f"--robot.id={args.follower_id}", f"--robot.calibration_dir={follower_dir}",
        "--robot.use_degrees=true", "--robot.disable_torque_on_disconnect=true",
        f"--robot.max_relative_target={args.max_relative_target}",
        f"--fps={args.fps}", "--display_data=false",
    ]
    if args.duration is not None:
        command.append(f"--teleop_time_s={args.duration}")
    files = (leader_dir / f"{args.leader_id}.json", follower_dir / f"{args.follower_id}.json")
    return Plan(leader, follower, files, command)


def validate_calibration(path):
    if not path.is_file():
        raise ValueError(f"Missing upstream calibration: {path}; run lerobot-calibrate first")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != set(MOTORS):
        raise ValueError(f"{path}: expected calibration for all six SO-101 joints")
    for name, motor_id in MOTORS.items():
        record = data[name]
        fields = ("id", "drive_mode", "homing_offset", "range_min", "range_max")
        if not isinstance(record, dict) or any(type(record.get(k)) is not int for k in fields):
            raise ValueError(f"{path}/{name}: invalid motor calibration record")
        if record["id"] != motor_id or record["drive_mode"] != 0:
            raise ValueError(f"{path}/{name}: incompatible SO-101 motor ID or drive mode")
        if not 0 <= record["range_min"] < record["range_max"] <= 4095:
            raise ValueError(f"{path}/{name}: invalid encoder range")


def run_child(command):
    """Keep bus reservations until the foreground command exits, including on Ctrl+C."""
    child = subprocess.Popen(command, start_new_session=True)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signals = [signal.SIGTERM]
    if hasattr(signal, "SIGHUP"):
        signals.append(signal.SIGHUP)
    previous = {sig: signal.getsignal(sig) for sig in signals}
    try:
        for sig in signals:
            signal.signal(sig, interrupt)
        try:
            return child.wait()
        except KeyboardInterrupt:
            # Also relay SSH hangup/termination to the otherwise isolated child group.
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            while True:
                try:
                    return child.wait()
                except KeyboardInterrupt:
                    continue
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def execute(plan):
    if sys.platform != "linux":
        raise ValueError("Execute on the Linux host where BOTH USB arms are connected")
    try:
        installed = version("lerobot")
    except PackageNotFoundError as exc:
        raise ValueError("LeRobot is missing; install the locked hardware environment") from exc
    if installed != "0.6.1":
        raise ValueError(f"Expected locked LeRobot 0.6.1, found {installed}")
    for arm in (plan.leader, plan.follower):
        if not Path(arm.port).exists():
            raise ValueError(f"USB device is not connected: {arm.port}")
    for path in plan.calibration_files:
        validate_calibration(path)
    from lerobot_arm_control.robot.so101.driver import serial_owner

    with ExitStack() as stack:
        for arm in sorted((plan.leader, plan.follower), key=lambda arm: arm.port):
            stack.enter_context(serial_owner(arm.port))
        return run_child(plan.command)


def main(argv=None):
    command = parser()
    args = command.parse_args(argv)
    try:
        plan = build_plan(args)
        print(f"Leader: {plan.leader.name} -> {plan.leader.port}")
        print(f"Follower: {plan.follower.name} -> {plan.follower.port}")
        for path in plan.calibration_files:
            print(f"Calibration: {path} ({'present' if path.is_file() else 'missing'})")
        print(shlex.join(plan.command), flush=True)
        if not args.execute:
            print("Preview only. No devices opened. Add --execute on the Linux USB host to move.")
            return
        print("Follower will move to follow Leader. Support both arms before starting/stopping.",
              flush=True)
        code = execute(plan)
    except (ValueError, OSError, RuntimeError) as exc:
        command.exit(2, f"Teleoperation: {exc}\n")
    if code:
        raise SystemExit(code if code > 0 else 128 - code)


if __name__ == "__main__":
    main()
