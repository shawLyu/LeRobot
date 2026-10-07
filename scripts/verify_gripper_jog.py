"""Verify one +0.03 rad gripper jog and return, without a second active viewer."""

import argparse
import json
import time
from pathlib import Path

from arm_control.realtime_client import Link

ROOT = Path(__file__).resolve().parents[1]


def verify():
    link = Link("lerobot", "/home/xy/Research/lerobot", "hardware", hardware_control=True)
    proof = {"ok": False, "samples": [], "acks": []}
    original_torque = None
    last_ping = 0.0
    last_seq = -1

    def sample():
        nonlocal last_ping, last_seq
        hello, state, received_at, _, _ = link.snapshot()
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or "remote worker exited")
        if not state:
            return None
        if time.monotonic() - received_at > 0.5:
            raise RuntimeError("stale hardware feedback")
        if time.monotonic() - last_ping > 0.1:
            link.send("ping")
            last_ping = time.monotonic()
        if state["seq"] != last_seq:
            proof["samples"].append(state)
            last_seq = state["seq"]
        if state["control"]["last_fault"]:
            raise RuntimeError(state["control"]["last_fault"]["reason"])
        return state

    def command(operation, **kwargs):
        ack = link.wait_ack(link.send(operation, **kwargs))
        proof["acks"].append(ack)
        if not ack["ok"]:
            raise RuntimeError(ack["error"])
        return ack

    def await_target(angle):
        deadline = time.monotonic() + 4
        consecutive = 0
        previous = None
        while time.monotonic() < deadline:
            state = sample()
            if state and state["seq"] != previous:
                actual = state["control"]["target_qpos"]["gripper"] - state[
                    "control"]["error_rad"]["gripper"]
                consecutive = consecutive + 1 if abs(actual - angle) <= 0.007 else 0
                previous = state["seq"]
                if consecutive >= 3:
                    return actual
            time.sleep(0.01)
        raise TimeoutError("gripper failed to reach the jog target")

    try:
        deadline = time.monotonic() + 15
        while link.latest is None and time.monotonic() < deadline:
            sample()
            time.sleep(0.02)
        state = sample()
        if state is None:
            raise TimeoutError("no initial state")
        original_torque = state["telemetry"]["Torque_Enable"]["gripper"]
        if original_torque:
            raise ValueError("gripper must already be released for this verification")
        start = state["control"]["target_qpos"]["gripper"]
        low, high = state["control"]["limits"]["gripper"]
        if not low + 0.1 < start < high - 0.1:
            raise ValueError("gripper must be clear of its endpoints")
        proof["start_angle"] = start
        command("enable_joint", joint="gripper")
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            sample()
            time.sleep(0.01)
        command("joint_target", joint="gripper", angle=start + 0.03)
        outward = await_target(start + 0.03)
        if outward - start < 0.015:
            raise RuntimeError("no sufficient measured opening")
        command("joint_target", joint="gripper", angle=start)
        proof["outward_angle"] = outward
        proof["return_angle"] = await_target(start)
        proof["ok"] = True
    except Exception as exc:
        proof["error"] = str(exc)
    finally:
        try:
            state = link.snapshot()[1]
            if original_torque == 0 and state and "gripper" in state["control"]["enabled"]:
                command("release_joint", joint="gripper")
            command("hold")
            time.sleep(0.1)
            proof["final_state"] = link.snapshot()[1]
        except Exception as exc:
            proof["cleanup_error"] = str(exc)
        finally:
            link.close()
        path = ROOT / "logs/gripper-jog-after-review.json"
        path.write_text(json.dumps(proof, indent=2) + "\n")
        print(json.dumps({"ok": proof["ok"], "error": proof.get("error"), "proof": str(path)}))
    return proof["ok"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    if not parser.parse_args().execute:
        parser.error("explicit --execute is required; close the hardware viewer first")
    raise SystemExit(0 if verify() else 1)
