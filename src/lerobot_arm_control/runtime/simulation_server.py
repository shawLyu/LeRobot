"""Local-only MuJoCo worker speaking the same stdio protocol as the hardware worker."""

import argparse
import queue
import threading
import time
from pathlib import Path

import mujoco

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.runtime.realtime_server import next_deadline, receive, send
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
from lerobot_arm_control.transport.realtime import (
    JOINTS,
    VERSION,
    WATCHDOG_SECONDS,
    validate_target,
)

ROOT = workspace_root()


def serve(args):
    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    if (tuple(model.joint(i).name for i in range(model.njnt)) != JOINTS
            or tuple(model.actuator(i).name for i in range(model.nu)) != JOINTS):
        raise ValueError("model must have exactly the six named SO joints and actuators")
    ranges = model.actuator_ctrlrange.tolist()
    data.ctrl[:] = data.qpos
    circle = Circle(simulation=True) if args.circle else None
    if circle:
        data.qpos[:] = circle.points[0]
        data.ctrl[:] = data.qpos
        mujoco.mj_forward(model, data)
    commands, disconnected = queue.Queue(maxsize=16), threading.Event()
    send({"type": "hello", "backend": "mujoco", "joints": JOINTS,
          "control_config_sha256": CONTROL_CONFIG_SHA256, "units": "radians",
          "ranges": ranges, "raw_limits": None, "fingerprint": None,
          "hardware_motion_enabled": False, "hardware_control": False,
          "hz": args.hz, "control_hz": circle.control_hz if circle else None,
          "circle_available": circle is not None})
    threading.Thread(target=receive, args=(commands, disconnected), daemon=True).start()
    sequence = cycles = 0
    last_command = last_physics = next_control = next_state = time.monotonic()
    watchdog = False
    while not disconnected.is_set():
        now = time.monotonic()
        while not commands.empty():
            received, command = commands.get_nowait()
            command_id = command.get("id") if isinstance(command, dict) else None
            try:
                if not isinstance(command, dict) or command.get("version") != VERSION:
                    raise ValueError("unsupported protocol")
                if now - received > WATCHDOG_SECONDS:
                    raise ValueError("stale command")
                operation, result = command.get("op"), None
                if circle and circle.active and operation not in ("ping", "circle_start"):
                    circle.cancel("manual command")
                if operation == "circle_start" and circle:
                    circle.start(data.qpos.tolist(), time.monotonic(), laps=command.get("laps"))
                    result = circle.status(time.monotonic())
                elif operation == "ping":
                    result = "pong"
                elif operation == "target":
                    data.ctrl[:] = validate_target(command.get("qpos"), data.ctrl, ranges)
                elif operation == "hold":
                    data.ctrl[:] = data.qpos
                else:
                    raise ValueError("operation unavailable for simulation")
                last_command, watchdog = time.monotonic(), False
                send({"type": "ack", "id": command_id, "op": operation,
                      "ok": True, "result": result})
            except (ValueError, RuntimeError) as exc:
                send({"type": "ack", "id": command_id, "ok": False, "error": str(exc)})
        if now - last_command > WATCHDOG_SECONDS and not watchdog:
            data.ctrl[:] = data.qpos
            if circle:
                circle.cancel("heartbeat timeout")
            watchdog = True
        steps = min(20, int((now - last_physics) / model.opt.timestep))
        for _ in range(steps):
            if circle and circle.active and last_physics + model.opt.timestep >= next_control:
                data.ctrl[:] = circle.sample(last_physics + model.opt.timestep)
                cycles += 1
                next_control = next_deadline(next_control, last_physics + model.opt.timestep,
                                             1 / circle.control_hz)
            mujoco.mj_step(model, data)
            last_physics += model.opt.timestep
        if steps and now - last_physics > .1:
            last_physics = now
        if now >= next_state:
            send({"type": "state", "seq": sequence, "time_unix_ns": time.time_ns(),
                  "qpos": data.qpos.tolist(), "qvel": data.qvel.tolist(),
                  "target": data.ctrl.tolist(), "sim_time": data.time,
                  "watchdog_hold": watchdog, "control_cycles": cycles,
                  "circle": circle.status(time.monotonic()) if circle else None})
            sequence += 1
            next_state = next_deadline(next_state, time.monotonic(), 1 / args.hz)
        time.sleep(.001)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=resource_path('assets/so101/so101.xml'))
    parser.add_argument("--hz", type=float, default=20)
    parser.add_argument("--circle", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.hz <= 50:
        parser.error("hz must be in 1..50")
    try:
        serve(args)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    except Exception as exc:
        send({"type": "error", "error": str(exc)})
        raise


if __name__ == "__main__":
    main()
