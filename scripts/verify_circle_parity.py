"""Execute one remote circle, capture encoder-based tracking, and keep support torque."""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.runtime.realtime_client import Link
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle

ROOT = Path(__file__).resolve().parents[1]


def verify(backend):
    proof = {"backend": backend, "ok": False, "samples": [], "acks": []}
    link = Link("lerobot", "/home/xy/Research/lerobot", backend,
                hardware_control=backend == "hardware", circle=True,
                keep_simulation_alive=backend == "mujoco")
    last_ping = last_seq = 0

    def observe():
        nonlocal last_ping, last_seq
        _, state, received, _, _ = link.snapshot()
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or "remote worker exited")
        if state is None:
            return None
        if time.monotonic() - received > 0.5:
            raise RuntimeError("stale feedback")
        if backend == "hardware" and time.monotonic() - last_ping > 0.1:
            link.send("ping")
            last_ping = time.monotonic()
        if state["seq"] != last_seq:
            proof["samples"].append(state)
            last_seq = state["seq"]
        if backend == "hardware":
            if any(state["telemetry"]["Status"].values()):
                raise RuntimeError("motor fault")
            if state["control"]["last_fault"]:
                raise RuntimeError(state["control"]["last_fault"]["reason"])
        return state

    def command(operation, **kwargs):
        ack = link.wait_ack(link.send(operation, **kwargs))
        proof["acks"].append(ack)
        if not ack["ok"]:
            raise RuntimeError(ack.get("error", "command failed"))
        return ack

    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and observe() is None:
            time.sleep(0.02)
        if link.latest is None:
            raise TimeoutError("no hardware state")
        # Verify holding first; no new movement target in this command.
        command("hold")
        for _ in range(10):
            observe()
            time.sleep(0.05)
        result = command("circle_start", laps=1)["result"]
        proof["started"] = True
        deadline = time.monotonic() + result["entry_seconds"] + 15
        while time.monotonic() < deadline:
            state = observe()
            phase = state["circle"]["phase"]
            if phase == "completed":
                break
            if phase not in ("entering", "running"):
                # A state published just before the start acknowledgement may be ready.
                if phase != "ready":
                    raise RuntimeError(phase)
            time.sleep(0.01)
        else:
            raise TimeoutError("one-lap execution did not complete")
        for _ in range(10):
            observe()
            time.sleep(0.05)
        plan = Circle(simulation=backend == "mujoco")
        plan.start(result["initial_qpos"], 0, laps=1)
        model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/so101/scene_mujoco.xml"))
        actual_data, target_data = mujoco.MjData(model), mujoco.MjData(model)
        saved_path = ROOT / "data/calibration/follower_mujoco.json"
        saved = json.loads(saved_path.read_text())
        calibration = AngleCalibration(saved_path, saved["fingerprint"])
        cartesian_errors, joint_errors = [], []
        running = [s for s in proof["samples"] if s.get("circle", {}).get("phase") == "running"]
        for s in running:
            actual = (calibration.angles(s["raw"], [0] * 6) if backend == "hardware"
                      else s["qpos"])
            expected = plan.sample(s["circle"]["elapsed_seconds"])
            actual_data.qpos[:] = actual
            target_data.qpos[:] = expected
            mujoco.mj_forward(model, actual_data)
            mujoco.mj_forward(model, target_data)
            joint_errors.append(float(np.max(np.abs(np.asarray(actual) - expected))))
            cartesian_errors.append(float(np.linalg.norm(
                actual_data.site("gripperframe").xpos - target_data.site("gripperframe").xpos)))
        first, last = running[0], running[-1]
        elapsed = (last["time_unix_ns"] - first["time_unix_ns"]) / 1e9
        metrics = {"period_seconds": plan.period,
                   "control_hz": (last["control_cycles"] - first["control_cycles"]) / elapsed,
                   "max_joint_tracking_error_rad": max(joint_errors),
                   "max_encoder_model_tracking_error_m": max(cartesian_errors),
                   "tracking_source": "encoders plus calibrated model" if backend == "hardware"
                   else "MuJoCo measured state",
                   "running_samples": len(running)}
        proof["metrics"] = metrics
        proof["completed"] = True
        proof["ok"] = (0.9 * plan.control_hz < metrics["control_hz"] < 1.1 * plan.control_hz
                       and max(joint_errors) < 0.01 and max(cartesian_errors) < 0.005)
        if not proof["ok"]:
            proof["error"] = "tracking performance did not meet the acceptance bounds"
    except (Exception, KeyboardInterrupt) as exc:
        proof["error"] = str(exc)
    finally:
        try:
            if link.process.poll() is None and link.latest is not None:
                command("hold")
                proof["final_state"] = link.snapshot()[1]
        except Exception as exc:
            proof["cleanup_error"] = str(exc)
            proof["ok"] = False
        finally:
            link.close()
        path = ROOT / f"logs/parity-{backend}-one-lap-{time.time_ns()}.json"
        path.write_text(json.dumps(proof, indent=2) + "\n")
        print(json.dumps({"ok": proof["ok"], "error": proof.get("error"),
                          "metrics": proof.get("metrics"), "proof": str(path)}), flush=True)
    return proof["ok"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "hardware"), default="mujoco")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.backend == "hardware" and not args.execute:
        parser.error("hardware verification requires --execute and a clear, observed arm")
    raise SystemExit(0 if verify(args.backend) else 1)
