"""Shared timing, stdio protocol and deferred logging for runtime workers."""

import json
import queue
import sys
import threading
import time

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, CONTROL_HZ
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.transport.realtime import VERSION

ROOT = workspace_root()
SEND_LOCK = threading.Lock()


def next_deadline(previous, finished, period):
    """Keep a fixed time grid; skip missed slots instead of issuing catch-up bursts."""
    deadline = previous + period
    if deadline < finished:
        deadline += (int((finished - deadline) / period) + 1) * period
    return deadline


def send(message):
    with SEND_LOCK:
        print(json.dumps({"version": VERSION, **message}, allow_nan=False), flush=True)


def receive(commands, disconnected):
    try:
        for line in sys.stdin:
            try:
                if len(line) > 8192:
                    raise ValueError("command too large")
                commands.put_nowait((time.monotonic(), json.loads(line)))
            except (ValueError, queue.Full) as exc:
                send({"type": "error", "error": str(exc)})
    finally:
        disconnected.set()


def save_control_trace(path, frames, gc_pauses=(), metadata=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"version": 1, "control_hz": CONTROL_HZ,
                                "frames": list(frames), "gc_pauses": list(gc_pauses),
                                "control_config_sha256": CONTROL_CONFIG_SHA256,
                                "metadata": metadata or {}},
                               separators=(",", ":")) + "\n")


def save_control_events(events):
    # Persist after holding/closing the arm. Filesystem stalls must not interrupt
    # the position stream; faults and recovered warnings also remain in state.
    if events:
        directory = ROOT / "logs"
        directory.mkdir(parents=True, exist_ok=True)
        for name, event in events:
            (directory / name).write_text(json.dumps(event, indent=2) + "\n")


def defer_cyclic_gc(stack):
    import gc

    # Tick data is acyclic and bounded; reference counting still releases it.
    # Large cyclic collections otherwise scan the retained trace mid-motion.
    if gc.isenabled():
        gc.disable()
        stack.callback(gc.enable)

