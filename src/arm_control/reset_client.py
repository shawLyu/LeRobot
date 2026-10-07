"""Trigger the remote Reset worker without importing local MuJoCo or opening a window."""

import argparse
import json
import time

from .realtime_client import Link


def run_reset(link, timeout=120, *, clock=time.monotonic, sleep=time.sleep):
    startup_deadline = clock() + 12
    while link.snapshot()[1] is None:
        if link.error or link.process.poll() is not None:
            raise RuntimeError(link.error or "SSH worker exited")
        if clock() >= startup_deadline:
            raise TimeoutError("no remote state received")
        sleep(0.05)
    triggered = False
    try:
        triggered = True
        ack = link.wait_ack(link.send("reset"))
        if not ack["ok"]:
            raise RuntimeError(ack.get("error") or str(ack))
        deadline = clock() + timeout
        while clock() < deadline:
            # This is a heartbeat, not a position target or trajectory sample.
            link.send("ping")
            sleep(0.1)
            _, state, received, _, _ = link.snapshot()
            if link.error or link.process.poll() is not None:
                raise RuntimeError(link.error or "SSH worker exited")
            if clock() - received > 0.5:
                raise RuntimeError("stale remote feedback")
            parking = state["control"]["parking"]
            if any(state["telemetry"]["Status"].values()):
                raise RuntimeError("motor fault during Reset")
            if parking["phase"] in ("boundary hold", "unpowered rest"):
                return state
            if parking["phase"] not in ("moving", "settling", "verifying release"):
                raise RuntimeError(parking["notice"])
        raise TimeoutError("Reset timed out")
    except BaseException:
        if triggered:
            try:
                link.wait_ack(link.send("hold"), timeout=2)
            except Exception:
                pass  # EOF/remote watchdog remains the fallback if SSH is unavailable.
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="lerobot")
    parser.add_argument("--remote-root", default="/home/xy/Research/lerobot")
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 300:
        parser.error("timeout must be between 1 and 300 seconds")
    link = Link(args.host, args.remote_root, "hardware", hardware_control=True)
    try:
        state = run_reset(link, args.timeout)
        print(json.dumps({"parking": state["control"]["parking"],
                          "raw": state["raw"], "torque": state["telemetry"]["Torque_Enable"]}))
    except (RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
        parser.exit(1, f"Reset stopped: {exc}\n")
    finally:
        link.close()


if __name__ == "__main__":
    main()
