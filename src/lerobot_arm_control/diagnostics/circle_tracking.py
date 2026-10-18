"""Observed hardware trial: Reset first, finite circle, hold, retain every state.

This script does not change native motor PID or protection settings. Accuracy
metrics assess the completed experiment; ordinary tracking error never aborts it.
"""

import argparse
import json
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, POSITION_FEEDBACK
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.runtime_config import add_remote_arguments, remote_runtime
from lerobot_arm_control.runtime.realtime_client import Link
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
from lerobot_arm_control.transport.realtime import JOINTS, checked_command

ROOT = workspace_root()


def measure(proof, plan_path=None):
    samples = [s for s in proof["samples"] if s["circle"]["phase"] == "running"]
    if len(samples) < 2:
        return None
    saved_path = ROOT / "data/calibration/follower_mujoco.json"
    saved = json.loads(saved_path.read_text())
    calibration = AngleCalibration(saved_path, saved["fingerprint"])
    plan = Circle() if plan_path is None else Circle(plan_path)
    info = samples[0]["circle"]
    plan.period = info["period_seconds"]
    plan.start(info["initial_qpos"], 0, laps=info["laps"])
    plan.entry_seconds = info["entry_seconds"]
    plan.ramp_seconds = info["ramp_seconds"]
    q = np.array([calibration.angles(s["raw"], [0] * 6) for s in samples])
    reference = np.array([plan.sample(s["circle"]["elapsed_seconds"]) for s in samples])
    error = reference - q
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/so101/scene_mujoco.xml"))
    actual, desired = mujoco.MjData(model), mujoco.MjData(model)
    cartesian = []
    for a, b in zip(q, reference):
        actual.qpos[:] = a
        desired.qpos[:] = b
        mujoco.mj_forward(model, actual)
        mujoco.mj_forward(model, desired)
        cartesian.append(np.linalg.norm(actual.site("gripperframe").xpos
                                        - desired.site("gripperframe").xpos))
    first, last = samples[0], samples[-1]
    return {"running_samples": len(samples),
            "control_hz": (last["control_cycles"] - first["control_cycles"])
            / ((last["time_unix_ns"] - first["time_unix_ns"]) / 1e9),
            "max_control_period_ms": max(s["control"]["max_period_ms"] for s in samples),
            "per_joint": {j: {"rms_degrees": float(np.rad2deg(np.sqrt(np.mean(error[:, i]**2)))),
                              "max_degrees": float(np.rad2deg(abs(error[:, i]).max())),
                              "bias_degrees": float(np.rad2deg(error[:, i].mean()))}
                          for i, j in enumerate(JOINTS)},
            "endpoint_rms_mm": float(np.sqrt(np.mean(np.asarray(cartesian)**2)) * 1000),
            "endpoint_max_mm": float(max(cartesian) * 1000),
            "endpoint_source": "encoder feedback plus calibrated model; not optical measurement"}


def trial(laps, *, host=None, remote_root=None, plan_path=None, reset=True):
    remote = remote_runtime(host, remote_root)
    stamp = time.time_ns()
    proof = {"completed": False, "samples": [], "acks": [], "laps": laps,
             "profile": POSITION_FEEDBACK, "control_config_sha256": CONTROL_CONFIG_SHA256,
             'reset_first': reset,
             'plan_sha256': (Circle().plan_hash if plan_path is None else file_digest(plan_path))}
    trace_name = f"tracking-trace-{stamp}.json"
    remote_trace = f"{remote.root}/logs/{trace_name}"
    link = Link(remote.host, remote.root, "hardware", hardware_control=True,
                circle=True, control_log=remote_trace, circle_plan=plan_path)
    last_seq = None
    last_ping = 0
    last_report = 0

    def observe():
        nonlocal last_seq, last_ping, last_report
        _, state, received, _, _ = link.snapshot()
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or "hardware worker exited")
        if state is not None and time.monotonic() - last_ping > 0.1:
            link.send("ping")
            last_ping = time.monotonic()
        if state is None:
            return None
        if time.monotonic() - received > 0.5:
            raise RuntimeError("stale remote feedback")
        if state["seq"] != last_seq:
            proof["samples"].append(state)
            last_seq = state["seq"]
        if time.monotonic() - last_report > 10:
            parking = state["control"]["parking"]
            print(json.dumps({"event": "trial_progress", "reset": parking["phase"],
                              "reset_remaining_degrees": parking["remaining_degrees"],
                              "circle": state["circle"]["phase"]}), flush=True)
            last_report = time.monotonic()
        if state["control"]["last_fault"]:
            raise RuntimeError(state["control"]["last_fault"]["reason"])
        return state

    def command(op, **kwargs):
        ack = checked_command(link, proof['acks'], op, fallback_error="command failed", **kwargs)
        return ack.get('result')

    try:
        deadline = time.monotonic() + 15
        while observe() is None:
            if time.monotonic() > deadline:
                raise TimeoutError("no hardware feedback")
            time.sleep(0.01)
        active = link.snapshot()[1]["control"].get("position_feedback")
        if (active["profile"] if active else None) != proof["profile"]:
            raise ValueError("remote tracking profile does not match the requested experiment")
        if link.snapshot()[1]["circle"]["profile_sha256"] != CONTROL_CONFIG_SHA256:
            raise ValueError("remote trajectory differs from the shared control configuration")
        if reset:
            command("reset")
            print("Reset started", flush=True)
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                state = observe()
                phase = state["control"]["parking"]["phase"]
                if phase in ("boundary hold", "unpowered rest"):
                    proof["reset_state"] = state
                    break
                if phase not in ("moving", "settling", "verifying release", "ready"):
                    raise RuntimeError(state["control"]["parking"]["notice"])
                time.sleep(0.01)
            else:
                raise TimeoutError("Reset did not settle; circle was not started")
        result = command("circle_start", laps=laps)
        print("Circle started", flush=True)
        deadline = time.monotonic() + result["entry_seconds"] + laps * result["period_seconds"] + 15
        while time.monotonic() < deadline:
            state = observe()
            if state["circle"]["phase"] == "completed":
                proof["completed"] = True
                break
            if state["circle"]["phase"].startswith("stopped"):
                raise RuntimeError(state["circle"]["phase"])
            time.sleep(0.01)
        else:
            raise TimeoutError("finite circle did not complete")
        proof["metrics"] = measure(proof, plan_path)
    except (Exception, KeyboardInterrupt) as exc:
        proof["error"] = str(exc) or type(exc).__name__
    finally:
        try:
            if link.process.poll() is None and link.latest is not None:
                command("hold")
                proof["final_state"] = link.snapshot()[1]
        except Exception as exc:
            proof["cleanup_error"] = str(exc)
        finally:
            link.close()
            try:
                subprocess.run(["scp", f"{remote.host}:{remote_trace}",
                                str(ROOT / "logs" / trace_name)],
                               check=True, timeout=20)
                proof["control_trace"] = str(ROOT / "logs" / trace_name)
            except Exception as exc:
                proof["trace_copy_error"] = str(exc)
            path = ROOT / f"logs/tracking-trial-{stamp}.json"
            path.write_text(json.dumps(proof, indent=2) + "\n")
            summary = {k: v for k, v in proof.items()
                       if k in ("completed", "error", "metrics", "cleanup_error")}
            print(json.dumps(summary | {"proof": str(path)}), flush=True)
    return proof["completed"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_remote_arguments(parser)
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument('--plan', type=Path, help='Reviewed workspace-local circle plan')
    parser.add_argument('--from-current', action='store_true',
                        help='Approach from the observed pose without invoking Reset')
    args = parser.parse_args()
    if not args.execute or not 1 <= args.laps <= 5:
        parser.error("requires --execute and 1..5 laps with an observed, clear arm")
    raise SystemExit(0 if trial(args.laps, host=args.host, remote_root=args.remote_root,
                               plan_path=args.plan, reset=not args.from_current) else 1)


if __name__ == "__main__":
    main()
