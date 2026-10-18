"""Project commands for environment checks, motor reads, and gripper tests."""

import argparse
import importlib
import importlib.metadata
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot_arm_control.control.motion import execute_motion, plan_motion
from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.robot.connection import read_state
from lerobot_arm_control.robot.feetech import connected_sts3215 as connected


def doctor(arms):
    from serial.tools import list_ports

    versions = {}
    for package, module in [
        ("lerobot", "lerobot"),
        ("feetech-servo-sdk", "scservo_sdk"),
        ("pyserial", "serial"),
        ("torch", "torch"),
        ("torchvision", "torchvision"),
    ]:
        importlib.import_module(module)
        versions[package] = importlib.metadata.version(package)
    ports = []
    for arm in arms.values():
        path = Path(arm.port)
        ok = path.exists() and os.access(path, os.R_OK | os.W_OK)
        ports.append(
            {
                "arm": arm.name,
                "port": arm.port,
                "resolved": str(path.resolve()),
                "read_write_access": ok,
            }
        )
    return {
        "python": sys.executable,
        "python_version": sys.version,
        "versions": versions,
        "configured_ports": ports,
        "serial_ports": [
            {"device": p.device, "serial_number": p.serial_number} for p in list_ports.comports()
        ],
        "success": all(p["read_write_access"] for p in ports),
    }


def parser():
    command = argparse.ArgumentParser(description=__doc__)
    command.add_argument("--config", type=Path, default=workspace_root() / "configs/robot.json")
    command.add_argument("--output", type=Path, help="Save a JSON result (UTC timestamps)")
    sub = command.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor", help="Verify package imports and configured USB permissions")
    sub.add_parser("list", help="Show configured arms and stable ports")
    read = sub.add_parser("read",
                          help="Read motor states after checking/restoring the saved baseline")
    read.add_argument("--arm", default="all")
    read.add_argument("--samples", type=int, default=3)
    read.add_argument("--interval", type=float, default=0.1)
    move = sub.add_parser("move", help="Preview a small ID-6 gripper test; execution is opt-in")
    move.add_argument("--arm", default="follower")
    move.add_argument("--delta", type=int, default=32, help="Encoder ticks, absolute value 16..32")
    move.add_argument("--execute", action="store_true")
    move.add_argument(
        "--confirm-clear",
        action="store_true",
        help="Confirm ID 6 is the gripper, its travel is clear, and someone is onsite",
    )
    return command


def main():
    command = parser()
    args = command.parse_args()
    if args.command == "move" and args.execute and not args.confirm_clear:
        command.error("--execute requires --confirm-clear after checking the physical setup")
    if args.command == "read" and (not 1 <= args.samples <= 1000 or not 0 <= args.interval <= 10):
        command.error("samples must be 1..1000 and interval 0..10 seconds")
    result = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "command": args.command}
    try:
        arms = load_arms(args.config)
        if args.command == "doctor":
            result.update(doctor(arms))
        elif args.command == "list":
            result.update(arms={name: vars(arm) for name, arm in arms.items()}, success=True)
        else:
            selected = list(arms.values()) if args.arm == "all" else [arms[args.arm]]
            if args.command == "read":
                result["arms"] = {}
                for arm in selected:
                    samples = []
                    with connected(arm) as bus:
                        for sample in range(args.samples):
                            samples.append({joint: read_state(bus, joint) for joint in arm.motors})
                            if sample + 1 < args.samples:
                                time.sleep(args.interval)
                    result["arms"][arm.name] = samples
                result["success"] = True
            else:
                if len(selected) != 1:
                    raise ValueError("Motion requires one selected follower")
                with connected(selected[0]) as bus:
                    plan = plan_motion(selected[0], bus, args.delta)
                    result.update(arm=selected[0].name, port=selected[0].port, plan=plan)
                    result["executed"] = args.execute
                    if args.execute:
                        result["motion"] = execute_motion(bus, plan)
                result["success"] = True
    except (Exception, KeyboardInterrupt) as error:
        result.update(success=False, error=f"{type(error).__name__}: {error}")
    serialized = json.dumps(result, indent=2, ensure_ascii=False)
    print(serialized)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized + "\n")
    return 0 if result["success"] else 1
