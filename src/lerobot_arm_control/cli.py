"""Unified source entry points for the LeRobot arm control system."""

import argparse
import importlib
import os
import sys
from contextlib import contextmanager
from pathlib import Path

from . import __version__
from .core.paths import WORKSPACE_ENV

# Leaves pass their arguments directly to existing parsers, preserving execution gates.
COMMANDS = {
    ("hardware",): ("diagnostics.cli", ()),
    ("view", "arm"): ("runtime.realtime_client", ("--backend", "mujoco")),
    ("view", "motors"): ("workbench.server", ()),
    ("view", "tabletop"): ("runtime.tabletop_client", ()),
    ("task", "circle"): ("runtime.realtime_client", ("--backend", "mujoco", "--circle")),
    ("task", "push-t"): ("planning.push_t", ()),
    ("task", "pick-t"): ("tasks.pick_t.pick_t_client", ()),
    ("demo", "follow"): ("runtime.follow_remote", ()),
    ("view", "follow"): ("runtime.follow_client", ()),
    ("reset",): ("runtime.reset_client", ()),
    ("calibrate", "camera"): ("calibration.camera_intrinsics", ()),
    ("calibrate", "height"): ("calibration.height_client", ()),
    ("calibrate", "table"): ("calibration.table_reference_calibration", ()),
    ("calibrate", "markers"): ("calibration.marker_alignment", ()),
    ("calibrate", "sample-markers"): ("calibration.marker_sampling", ()),
    ("verify", "joints"): ("diagnostics.joint_precision", ()),
    ("verify", "circle"): ("planning.circle_simulation", ()),
    ("verify", "response"): ("diagnostics.response", ()),
    ("verify", "native-pid"): ("diagnostics.native_pid", ()),
    ("verify", "motors"): ("diagnostics.motor_response", ()),
    ("verify", "follow-latency"): ("diagnostics.follow_latency", ()),
    ("verify", "tracking"): ("diagnostics.circle_tracking", ()),
    ("inspect", "tool-mount"): ("calibration.tool_mount", ()),
    ("serve", "arm"): ("runtime.realtime_server", ()),
    ("serve", "motors"): ("workbench.worker", ()),
    ("serve", "follow"): ("runtime.follow_server", ()),
    ("serve", "tabletop"): ("runtime.tabletop_server", ()),
    ("serve", "simulation"): ("runtime.simulation_server", ()),
}


@contextmanager
def command_arguments(arguments):
    previous = sys.argv
    sys.argv = ["lerobot-arm", *arguments]
    try:
        yield
    finally:
        sys.argv = previous


def parser():
    command = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    command.add_argument("--version", action="version", version=__version__)
    command.add_argument("--workspace", help="Robot workspace; set before the command")
    sub = command.add_subparsers(dest="command", required=True)
    groups = {}
    for parts in COMMANDS:
        if len(parts) == 1:
            sub.add_parser(parts[0], add_help=False, allow_abbrev=False)
        else:
            if parts[0] not in groups:
                parent = sub.add_parser(parts[0], allow_abbrev=False)
                groups[parts[0]] = parent.add_subparsers(dest="action", required=True)
            groups[parts[0]].add_parser(parts[1], add_help=False, allow_abbrev=False)
    return command


def main(argv=None):
    command = parser()
    args, remaining = command.parse_known_args(argv)
    if args.workspace:
        os.environ[WORKSPACE_ENV] = str(Path(args.workspace).expanduser().resolve())
    parts = (args.command, args.action) if hasattr(args, "action") else (args.command,)
    module, defaults = COMMANDS[parts]
    try:
        entry = importlib.import_module("lerobot_arm_control." + module)
        with command_arguments([*defaults, *remaining]):
            entry.main()
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith("lerobot_arm_control."):
            command.exit(2, "This command requires the complete local source checkout; "
                         "the selected deployment contains only the robot runtime.\n")
        if exc.name and exc.name.split(".")[0] in {"lerobot", "serial", "scservo_sdk", "torch",
                                                  "torchvision"}:
            command.exit(2, "Hardware dependencies are missing; run scripts/bootstrap.sh "
                         "on the Linux robot host with Python 3.12.\n")
        if exc.name and exc.name.split(".")[0] in {"mujoco", "cv2", "numpy", "PIL", "glfw"}:
            command.exit(2, "Local dependencies are missing; install requirements-local.txt "
                         "in the selected Python environment.\n")
        raise
