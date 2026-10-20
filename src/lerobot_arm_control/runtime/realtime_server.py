"""Remote hardware worker; stdout carries the stdio protocol."""

import argparse
import json
import queue
import signal
import threading
import time
from contextlib import ExitStack
from pathlib import Path

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, CONTROL_HZ
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.runtime.common import (
    defer_cyclic_gc,
    next_deadline,
    receive,
    save_control_events,
    save_control_trace,
    send,
)
from lerobot_arm_control.transport.latest_output import LatestOutput
from lerobot_arm_control.transport.realtime import (
    JOINTS,
    VERSION,
    WATCHDOG_SECONDS,
    preview_joint_target,
)

ROOT = workspace_root()

def serve(args):
    commands = queue.Queue(maxsize=16)
    disconnected = threading.Event()
    with ExitStack() as stack:
        from collections import deque

        trace = deque(maxlen=36000)
        control_events = deque(maxlen=256)
        stack.callback(save_control_events, control_events)
        trace_path = getattr(args, "control_log", None)
        trace_metadata = {}
        autonomous = getattr(args, 'standalone_circle_validation', None) is not None
        output_closed = threading.Event() if autonomous else disconnected
        publish = stack.enter_context(LatestOutput(send, output_closed))
        if trace_path:
            import gc

            gc_pauses = deque(maxlen=256)
            gc_started = [0.0]

            def gc_timing(phase, info):
                if phase == "start":
                    gc_started[0] = time.monotonic()
                else:
                    ended = time.monotonic()
                    gc_pauses.append({"started": gc_started[0], "ended": ended,
                                      "generation": info["generation"],
                                      "duration_ms": (ended - gc_started[0]) * 1000})

            gc.callbacks.append(gc_timing)
            stack.callback(gc.callbacks.remove, gc_timing)
            stack.callback(save_control_trace, trace_path, trace, gc_pauses, trace_metadata)
        controller = None
        circle = None
        joint_path = None
        last_fault = None
        last_status_warning = None
        from lerobot_arm_control.core.config import load_arms
        from lerobot_arm_control.robot.feetech import connected_sts3215 as connected

        arm = load_arms()[args.arm]
        bus = stack.enter_context(connected(arm))
        if trace_path:
            def capture_final_parameters():
                try:
                    trace_metadata['final_native_parameters'] = {
                        r: bus.sync_read(r, normalize=False) for r in (
                            'P_Coefficient', 'I_Coefficient', 'D_Coefficient', 'Lock',
                            'Acceleration', 'Goal_Velocity', 'Torque_Enable')}
                except Exception as exc:
                    trace_metadata['final_parameters_error'] = str(exc)
            # Registered before controller.close: capture after RAM restoration,
            # while the bus is still connected, then persist after disconnect.
            stack.callback(capture_final_parameters)
        low = bus.sync_read("Min_Position_Limit", normalize=False)
        high = bus.sync_read("Max_Position_Limit", normalize=False)
        limits = {name: [int(low[name]), int(high[name])] for name in JOINTS}
        ranges = None
        fingerprint = {"port": arm.port, "motors": arm.motors, "raw_limits": limits,
                       "homing_offsets": bus.sync_read("Homing_Offset", normalize=False),
                       "phase": bus.sync_read("Phase", normalize=False),
                       "mode": bus.sync_read("Operating_Mode", normalize=False)}
        if args.circle:
            from lerobot_arm_control.tasks.circle.circle_trajectory import Circle

            circle = Circle(getattr(args, 'circle_plan', None) or
                            resource_path('data/trajectories/circle.json'))
            trace_metadata['plan_sha256'] = circle.plan_hash
        if args.hardware_control:
            from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
            from lerobot_arm_control.control.manual_control import ManualControl

            identity = {**fingerprint, "model": "so101", "mjcf_sha256": file_digest(args.model)}
            calibration = AngleCalibration(ROOT / "data/calibration/follower_mujoco.json",
                                           identity)
            controller = ManualControl(bus, calibration, limits,
                                       rest_path=ROOT / "data/calibration/follower_rest.json",
                                       feedback_profile=circle.feedback_profile if circle else None)
            trace_metadata['position_feedback_profile'] = controller.position_feedback.profile
            trace_metadata['joint_calibration_sha256'] = file_digest(calibration.path)
            defer_cyclic_gc(stack)
            stack.callback(lambda: controller.close(preserve_targets=bool(
                circle and circle.phase == 'completed' and not controller.stopped)))
        if circle and controller:
            controller.trajectory = circle
        publish({"type": "hello", "backend": "hardware", "joints": JOINTS,
              "control_config_sha256": CONTROL_CONFIG_SHA256,
              "units": "radians" if ranges else "encoder_ticks", "ranges": ranges,
              "raw_limits": limits, "hardware_motion_enabled": args.allow_gripper,
              "fingerprint": fingerprint,
              'joint_calibration_sha256': (file_digest(
                  ROOT / 'data/calibration/follower_mujoco.json') if controller else None),
              'joint_angle_mapping': ({joint: calibration.fit(joint) for joint in JOINTS}
                                      if controller else None),
              'native_parameters': ({register: bus.sync_read(register, normalize=False)
                                     for register in ('P_Coefficient', 'I_Coefficient',
                                                      'D_Coefficient', 'Lock', 'Acceleration',
                                                      'Goal_Velocity')} if controller else None),
              "hardware_control": controller is not None,
              "voltage_limits": ({register: bus.sync_read(register, normalize=False)
                                  for register in ("Min_Voltage_Limit", "Max_Voltage_Limit")}
                                 if controller else None),
              "hz": args.hz, "control_hz": (circle.control_hz if circle else
                                            CONTROL_HZ if controller else None),
              "circle_available": circle is not None, 'autonomous_circle': autonomous})
        input_closed = threading.Event() if autonomous else disconnected
        threading.Thread(target=receive, args=(commands, input_closed), daemon=True).start()
        if autonomous:
            validation = json.loads(Path(args.standalone_circle_validation).read_text())
            controller.heartbeat()
            circle.start_hardware(controller, validation, laps=args.circle_laps)
            autonomous_deadline = (controller.clock()+circle.entry_seconds
                                   +circle.period*args.circle_laps+circle.ramp_seconds+5)
            trace_metadata['autonomous_circle'] = True
            trace_metadata['entry_validation'] = validation
        sequence = 0
        control_cycles = 0
        next_state = next_control = time.monotonic()
        hardware_state = None
        telemetry = {}
        last_control_id = None

        def save_verification(message):
            directory = ROOT / "logs"
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"gripper-verification-{time.time_ns()}.json"
            path.write_text(json.dumps(message, indent=2) + "\n")

        def motion_feedback(phase, target):
            nonlocal sequence, telemetry
            if disconnected.is_set():
                raise RuntimeError("SSH disconnected during gripper verification")
            raw = bus.sync_read("Present_Position", normalize=False)
            telemetry = {
                register: bus.sync_read(register, normalize=False)
                for register in ("Torque_Enable", "Status", "Present_Voltage")
            }
            if any(telemetry["Status"].values()):
                raise RuntimeError("motor fault during gripper verification")
            sample = {"phase": phase, "target_raw": target,
                      "position_raw": int(raw["gripper"]),
                      "voltage_raw": int(telemetry["Present_Voltage"]["gripper"]),
                      "status": int(telemetry["Status"]["gripper"]),
                      "torque": int(telemetry["Torque_Enable"]["gripper"])}
            feedback.append(sample)
            publish({"type": "state", "seq": sequence, "time_unix_ns": time.time_ns(),
                  "raw": {name: int(raw[name]) for name in JOINTS},
                  "telemetry": telemetry, "gripper_verification": sample})
            sequence += 1

        while not disconnected.is_set():
            now = time.monotonic()
            if autonomous and now > autonomous_deadline:
                controller.stop('finite circle deadline: hold')
                break
            while not commands.empty():
                received, command = commands.get_nowait()
                command_id = command.get("id") if isinstance(command, dict) else None
                diagnostics = None
                try:
                    if not isinstance(command, dict) or command.get("version") != VERSION:
                        raise ValueError("unsupported protocol")
                    if now - received > WATCHDOG_SECONDS:
                        raise ValueError("stale command")
                    operation = command.get("op")
                    if autonomous and operation not in ('ping', 'hold'):
                        raise ValueError('finite remote circle accepts only hold or ping')
                    result = None
                    if (controller and controller.trajectory and controller.trajectory.active
                            and operation in ('circle_start', 'joint_path_start')):
                        raise ValueError('another automatic motion is running')
                    if (joint_path and joint_path.active
                            and operation not in ('ping', 'joint_path_start')):
                        controller.stop('joint path interrupted by manual command')
                    if (circle and circle.active and operation not in ("ping", "circle_start")):
                        if controller:
                            controller.stop("circle interrupted by manual command")
                        else:
                            circle.cancel("manual command")
                    if operation == 'joint_path_start':
                        if not controller:
                            raise ValueError('joint path requires calibrated hardware control')
                        from lerobot_arm_control.control.joint_path import JointPath

                        candidate = JointPath()
                        candidate.start(controller, command.get('plan'), command.get('validation'))
                        joint_path = candidate
                        last_control_id = command_id
                        result = joint_path.status()
                    elif operation == "circle_start":
                        if not circle:
                            raise ValueError("circle was not enabled for this worker")
                        if controller:
                            controller.trajectory = circle
                            circle.start_hardware(controller, command.get("validation"),
                                                  laps=command.get("laps"))
                            last_control_id = command_id
                        result = circle.status(time.monotonic())
                    elif operation == "ping":
                        if controller:
                            controller.heartbeat()
                        result = "pong"
                    elif controller and operation in (
                        "enable_joint", "enable_all", "joint_target", "settle_joint", "hold",
                        "release_joint",
                        "reset", "save_rest_pose",
                    ):
                        joint = command.get("joint")
                        if operation == "enable_joint":
                            controller.enable(joint)
                        elif operation == "enable_all":
                            controller.enable_all()
                        elif operation == "joint_target":
                            controller.target(joint, command.get("angle"))
                        elif operation == "settle_joint":
                            controller.settle_target(joint)
                        elif operation == "release_joint":
                            controller.release(joint)
                        elif operation == "reset":
                            controller.parking.start()
                        elif operation == "save_rest_pose":
                            controller.parking.save()
                        else:
                            controller.stop()
                        last_control_id = command_id
                        result = controller.status()
                    elif operation == "joint_preview":
                        result = preview_joint_target(bus, command.get("joint"),
                                                      command.get("delta"))
                    elif operation == "gripper_test":
                        from lerobot_arm_control.control.motion import execute_motion, plan_motion

                        plan = plan_motion(arm, bus, command.get("delta", 16))
                        if command.get("execute", False):
                            if not args.allow_gripper:
                                raise ValueError("hardware motion disabled; use preview")
                            from lerobot_arm_control.calibration.angle_calibration import (
                                AngleCalibration,
                            )

                            identity = {**fingerprint, "model": "so101",
                                        "mjcf_sha256": file_digest(args.model)}
                            calibration = AngleCalibration(
                                ROOT / "data/calibration/follower_mujoco.json", identity)
                            calibration.check_gripper_test(plan)
                            feedback = []
                            diagnostics = {"plan": plan, "feedback": feedback}
                            try:
                                motion_feedback("preflight", plan["start_raw"])
                                result = execute_motion(bus, plan, observer=motion_feedback)
                                motion_feedback("completed", plan["start_raw"])
                                result["feedback"] = feedback
                                result["final_torque"] = feedback[-1]["torque"]
                                save_verification({"ok": True, "result": result})
                            finally:
                                # This launch authorizes one bounded test, never repeated by a key.
                                args.allow_gripper = False
                        else:
                            result = {"preview": True, **plan}
                    else:
                        raise ValueError("operation unavailable for this backend")
                    publish({"type": "ack", "id": command_id, "op": operation,
                          "ok": True, "result": result})
                except (ValueError, RuntimeError, TimeoutError) as exc:
                    message = {"type": "ack", "id": command_id,
                               "op": command.get("op") if isinstance(command, dict) else None,
                               "ok": False, "error": str(exc)}
                    if diagnostics is not None:
                        try:
                            diagnostics["final_torque"] = int(bus.read(
                                "Torque_Enable", "gripper", normalize=False))
                            diagnostics["final_position_raw"] = int(bus.read(
                                "Present_Position", "gripper", normalize=False))
                        except Exception as cleanup_error:
                            diagnostics["final_state_error"] = str(cleanup_error)
                        message["diagnostics"] = diagnostics
                        save_verification(message)
                    publish(message)
            now = time.monotonic()
            if controller and now >= next_control:
                from lerobot_arm_control.control.servo_feedback import read_control_feedback

                read_started = time.monotonic()
                telemetry = read_control_feedback(bus)
                read_finished = time.monotonic()
                raw = telemetry.pop("Present_Position")
                if (autonomous and controller.initialized
                        and read_finished-controller.last_tick > WATCHDOG_SECONDS):
                    controller.stop('control feedback deadline: hold')
                if autonomous and circle.active:
                    # Only this explicitly accepted finite remote plan owns the
                    # reference. A slow observer cannot pause its control clock.
                    controller.heartbeat()
                controller.step(raw, telemetry)
                write_finished = time.monotonic()
                if trace_path:
                    trace.append({"time_monotonic": controller.last_tick,
                                  "command_id": last_control_id,
                                  "raw": dict(raw), "goal_raw": dict(controller.sent),
                                  "nominal_qpos": dict(controller.targets),
                                  "telemetry": {k: dict(v) for k, v in telemetry.items()},
                                  "correction": (dict(controller.position_feedback.correction)
                                                 if controller.position_feedback else None),
                                  "circle": circle.status(controller.last_tick) if circle else None,
                                  'joint_path': joint_path.status() if joint_path else None,
                                  "reset_phase": controller.parking.phase})
                    trace[-1]["read_ms"] = (read_finished - read_started) * 1000
                    trace[-1]["step_ms"] = (write_finished - read_finished) * 1000
                control_cycles += 1
                hardware_state = {"raw": {name: int(raw[name]) for name in JOINTS},
                                  "qpos": calibration.angles(raw, [0.] * len(JOINTS)),
                                  "telemetry": telemetry, "control": controller.status()}
                hardware_state["control"]["last_command_id"] = last_control_id
                if controller.last_fault is not last_fault:
                    last_fault = controller.last_fault
                    control_events.append((f"manual-control-fault-{time.time_ns()}.json",
                                           last_fault))
                if controller.status_warning is not last_status_warning:
                    last_status_warning = controller.status_warning
                    control_events.append((f"motor-status-recovered-{time.time_ns()}.json",
                                           last_status_warning))
                next_control = next_deadline(next_control, time.monotonic(), 1 / CONTROL_HZ)
                if trace_path:
                    trace[-1]["processing_ms"] = (time.monotonic() - read_started) * 1000
            if now >= next_state or autonomous and not circle.active:
                if controller:
                    # Sync writes have no per-motor ACK. Record the actual RAM goal,
                    # rather than treating the last transmitted value as delivery proof.
                    state = {**hardware_state,
                             "servo_goal_raw": hardware_state["telemetry"]["Goal_Position"]}
                else:
                    raw = bus.sync_read("Present_Position", normalize=False)
                    if sequence % max(1, round(args.hz)) == 0:
                        telemetry = {
                            register: bus.sync_read(register, normalize=False)
                            for register in ("Torque_Enable", "Status", "Present_Voltage", "Moving")
                        }
                    state = {"raw": {name: int(raw[name]) for name in JOINTS},
                             "telemetry": telemetry}
                publish({"type": "state", "seq": sequence, "time_unix_ns": time.time_ns(),
                      **state, "circle": circle.status(time.monotonic()) if circle else None,
                      'joint_path': joint_path.status() if joint_path else None,
                      "control_cycles": control_cycles})
                sequence += 1
                next_state = next_deadline(next_state, time.monotonic(), 1 / args.hz)
            if autonomous and not circle.active:
                break
            time.sleep(0.001)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=resource_path('assets/so101/so101.xml'))
    parser.add_argument("--arm", default="follower")
    parser.add_argument("--hz", type=float, default=20)
    parser.add_argument("--allow-gripper", action="store_true",
                        help="Enable existing bounded gripper roundtrip; requires checked power")
    parser.add_argument("--hardware-control", action="store_true",
                        help="Calibrated, slow manual control; joints enabled individually")
    parser.add_argument("--circle", action="store_true",
                        help="Load a circle; start via circle_start or a bound standalone proof")
    parser.add_argument('--circle-plan', type=Path,
                        help='Reviewed circle plan; slower plan timing is retained')
    parser.add_argument('--standalone-circle-validation', type=Path,
                        help='Run a finite remotely owned circle using this bound local proof')
    parser.add_argument('--circle-laps', type=int, default=1)
    parser.add_argument("--control-log", type=Path,
                        help="Retain up to ten minutes of full-rate feedback; write on exit")
    args = parser.parse_args()
    if not 1 <= args.hz <= 50:
        parser.error("hz must be in 1..50")
    if args.hardware_control and args.allow_gripper:
        parser.error("hardware control requires launch without --allow-gripper")
    if args.circle and (args.allow_gripper or not args.hardware_control):
        parser.error("circle requires calibrated hardware control and local validation")
    if args.circle_plan and not args.circle:
        parser.error('--circle-plan requires --circle')
    if args.standalone_circle_validation and (not args.circle or not 1 <= args.circle_laps <= 5):
        parser.error('standalone circle requires --circle and 1..5 finite laps')
    if args.control_log and not args.hardware_control:
        parser.error("--control-log requires hardware control")
    try:
        from lerobot_arm_control.core.processes import interrupt

        for signum in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(signum, interrupt)
        serve(args)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    except Exception as exc:
        send({"type": "error", "error": str(exc)})
        raise


if __name__ == "__main__":
    main()
