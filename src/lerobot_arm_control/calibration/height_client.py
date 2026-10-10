"""Coordinate remote read-only samples, local fitting and remote result application."""

import argparse
import json
import subprocess
import sys

from lerobot_arm_control.calibration.height_records import ANGLES, BASE, CANDIDATE, DATA, ROOT
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import source_environment
from lerobot_arm_control.core.processes import remote_python


def remote(host, root, arguments):
    command = remote_python(root, "lerobot_arm_control.calibration.height_sampling", arguments)
    subprocess.run(["ssh", "-t", host, command], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("collect", "resample", "fit", "apply"))
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--poses", type=int, nargs="+")
    parser.add_argument("--native-pitch-scale", action="store_true")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--host", default="lerobot")
    parser.add_argument("--remote-root", default="/home/xy/Research/lerobot")
    args = parser.parse_args()
    if args.action == "resample" and not args.poses:
        parser.error("resample requires --poses")
    if args.action in ("collect", "resample"):
        command = [args.action, "--count", str(args.count)]
        if args.poses:
            command += ["--poses", *map(str, args.poses)]
        remote(args.host, args.remote_root, command)
    # Raw records always come from the hardware host before local fitting/validation.
    for path in (ANGLES, BASE, DATA):
        subprocess.run(["scp", f"{args.host}:{args.remote_root}/{path.relative_to(ROOT)}",
                        str(path)], check=True)
    if args.action == "fit" or (args.action in ("collect", "resample") and args.apply):
        command = [sys.executable, "-m",
                   "lerobot_arm_control.calibration.height_calibration", "fit"]
        if args.native_pitch_scale:
            command.append("--native-pitch-scale")
        subprocess.run(command, cwd=ROOT, env=source_environment(), check=True)
    if args.action == "apply" or args.apply:
        from lerobot_arm_control.calibration.height_records import apply_candidate

        candidate = json.loads(CANDIDATE.read_text())
        if candidate.get("dataset_sha256") != file_digest(DATA):
            raise ValueError("Samples changed after local fitting; fit again before applying")
        # Validate locally before sending. Both hosts enforce the original record fingerprints.
        if not candidate.get("validation", {}).get("passed"):
            raise ValueError("Local height validation failed; candidate cannot be applied")
        subprocess.run(["scp", str(CANDIDATE),
                        f"{args.host}:{args.remote_root}/{CANDIDATE.relative_to(ROOT)}"],
                       check=True)
        remote(args.host, args.remote_root, ["apply"])
        apply_candidate(candidate)


if __name__ == "__main__":
    main()
