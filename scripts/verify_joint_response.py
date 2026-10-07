"""Measure one bounded elbow excursion and return using the revised position stream."""

import argparse
import hashlib
import json
import time
from pathlib import Path

import mujoco

from arm_control.angle_calibration import AngleCalibration
from arm_control.circle_simulation import check_path, entry_points
from arm_control.realtime_client import Link

ROOT = Path(__file__).resolve().parents[1]
JOINT = "elbow_flex"
DELTA = -0.06  # 3.44 degrees, then return to the initial measured angle.


def verify():
    proof = {"ok": False, "joint": JOINT, "delta_rad": DELTA, "samples": [], "acks": []}
    link = Link("lerobot", "/home/xy/Research/lerobot", "hardware", hardware_control=True)
    calibration = None
    last_seq, last_ping = -1, 0.0

    def observe():
        nonlocal last_seq, last_ping
        hello, state, received, _, _ = link.snapshot()
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or "hardware worker exited")
        if state is None:
            return None
        if time.monotonic() - received > 0.5:
            raise RuntimeError("stale hardware feedback")
        if time.monotonic() - last_ping >= 0.1:
            link.send("ping")
            last_ping = time.monotonic()
        if state["seq"] != last_seq:
            proof["samples"].append(state)
            last_seq = state["seq"]
        if any(state["telemetry"]["Status"].values()) or state["control"]["last_fault"]:
            raise RuntimeError("motor/controller fault; stop the response test")
        return state

    def command(operation, **kwargs):
        ack = link.wait_ack(link.send(operation, **kwargs))
        proof["acks"].append(ack)
        if not ack["ok"]:
            raise RuntimeError(ack.get("error", "command failed"))
        return ack

    def move(target):
        command("joint_target", joint=JOINT, angle=target)
        deadline = time.monotonic() + 4
        previous, consecutive = None, 0
        while time.monotonic() < deadline:
            state = observe()
            if state["seq"] != previous:
                previous = state["seq"]
                actual = calibration.angles(state["raw"], [0] * 6)[2]
                sent = calibration.angles(state["servo_goal_raw"], [0] * 6)[2]
                if abs(sent - actual) > 0.08:
                    raise RuntimeError("elbow response exceeds 0.08 rad tracking bound")
                arrived = abs(actual - target) <= 0.01 and abs(sent - target) <= 0.002
                consecutive = consecutive + 1 if arrived else 0
                if consecutive >= 4:
                    return actual
            time.sleep(0.01)
        raise TimeoutError("elbow did not settle within 0.01 rad in four seconds")

    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and observe() is None:
            time.sleep(0.02)
        hello, state, _, _, _ = link.snapshot()
        if state is None:
            raise TimeoutError("no initial hardware state")
        identity = {**hello["fingerprint"], "model": "so101", "mjcf_sha256": hashlib.sha256(
            (ROOT / "assets/so101/so101.xml").read_bytes()).hexdigest()}
        calibration = AngleCalibration(ROOT / "data/calibration/follower_mujoco.json", identity)
        command("hold")
        for _ in range(5):
            observe()
            time.sleep(0.05)
        state = observe()
        initial = calibration.angles(state["raw"], [0] * 6)
        destination = initial[:]
        destination[2] += DELTA
        low, high = state["control"]["limits"][JOINT]
        if not low + 0.01 < destination[2] < initial[2] < high - 0.01:
            raise ValueError("elbow is too close to a calibrated boundary")
        model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/so101/so101.xml"))
        check_path(model, entry_points(initial, destination))
        proof["initial_qpos"] = initial
        proof["destination_qpos"] = destination
        command("enable_joint", joint=JOINT)
        proof["outward_angle"] = move(destination[2])
        if initial[2] - proof["outward_angle"] < 0.04:
            raise RuntimeError("insufficient measured elbow movement")
        proof["return_angle"] = move(initial[2])
        proof["ok"] = True
    except (Exception, KeyboardInterrupt) as exc:
        proof["error"] = str(exc)
    finally:
        try:
            if link.process.poll() is None and link.latest is not None:
                command("hold")
        except Exception as exc:
            proof["cleanup_error"] = str(exc)
            proof["ok"] = False
        finally:
            link.close()  # Keeps support torque and restores the original RAM acceleration.
        path = ROOT / f"logs/joint-response-{time.time_ns()}.json"
        path.write_text(json.dumps(proof, indent=2) + "\n")
        print(json.dumps({"ok": proof["ok"], "error": proof.get("error"),
                          "outward_angle": proof.get("outward_angle"),
                          "return_angle": proof.get("return_angle"), "proof": str(path)}))
    return proof["ok"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    if not parser.parse_args().execute:
        parser.error("--execute is required; close other motor tools and observe the clear arm")
    raise SystemExit(0 if verify() else 1)
