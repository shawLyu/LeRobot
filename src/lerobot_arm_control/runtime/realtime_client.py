"""Local MuJoCo simulation, or local display of the remote hardware worker."""

import argparse
import hashlib
import json
import math
import queue
import subprocess
import sys
import textwrap
import threading
import time
from collections import deque
from pathlib import Path

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.control_input import JogCommands, ResetThenCircle
from lerobot_arm_control.core.paths import resource_path, source_environment, workspace_root
from lerobot_arm_control.core.processes import remote_python
from lerobot_arm_control.transport.realtime import JOINTS, VERSION, raw_to_angles
from lerobot_arm_control.visualization.ui import motor_fault_text

ROOT = workspace_root()


class Link:
    def __init__(self, host, remote_root, backend, *, allow_gripper=False, hardware_control=False,
                 circle=False, keep_simulation_alive=False,
                 simulation_model=None, control_log=None, arm='follower'):
        if backend not in ("mujoco", "hardware"):
            raise ValueError("unknown backend")
        if keep_simulation_alive and backend != "mujoco":
            raise ValueError("independent heartbeat is simulation-only")
        if arm != 'follower' and (hardware_control or allow_gripper):
            raise ValueError('Additional arms support read-only angle calibration only')
        worker_args = ["-m", "lerobot_arm_control.runtime.simulation_server" if backend == "mujoco"
                       else "lerobot_arm_control.runtime.realtime_server"]
        if allow_gripper:
            worker_args.append("--allow-gripper")
        if hardware_control:
            worker_args.append("--hardware-control")
        if control_log is not None:
            if not hardware_control:
                raise ValueError("control logging requires hardware control")
            worker_args.extend(("--control-log", str(control_log)))
        if circle:
            worker_args.append("--circle")
        if backend == "mujoco":
            if simulation_model is not None:
                worker_args.extend(("--model", str(Path(simulation_model).resolve())))
            launch = [sys.executable, "-u", *worker_args]
            self.worker_location = "local"
        else:
            if arm != 'follower':
                worker_args.extend(('--arm', arm))
            command = remote_python(remote_root, worker_args[1], worker_args[2:])
            launch = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                      "-o", "ControlMaster=no", "-o", "ControlPath=none",
                      "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", host, command]
            self.worker_location = "remote"
        self.process = subprocess.Popen(
            launch, cwd=ROOT, env=source_environment(),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1,
        )
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.keep_simulation_alive = keep_simulation_alive
        self.hardware_control = hardware_control
        self.backend = backend
        self.hello = None
        self.latest = None
        self.received_at = 0.0
        self.received_count = 0
        self.acks = {}
        self.pending = {}
        self.last_ack = None
        self.last_action_ack = None
        self.error = None
        self.next_id = 0
        self.states = deque(maxlen=20)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        last_simulation_ping = 0.0
        try:
            for line in self.process.stdout:
                message = json.loads(line)
                if message.get("version") != VERSION:
                    raise ValueError("worker protocol mismatch")
                with self.lock:
                    kind = message.get("type")
                    if kind == "hello":
                        if tuple(message["joints"]) != JOINTS:
                            raise ValueError("joint map mismatch")
                        if (self.hardware_control and message.get("control_config_sha256")
                                != CONTROL_CONFIG_SHA256):
                            raise ValueError(
                                "Local/remote control configuration differs; deploy code")
                        self.hello = message
                    elif kind == "state":
                        self.latest = message
                        self.states.append(message)
                        self.received_at = time.monotonic()
                        self.received_count += 1
                    elif kind == "ack":
                        sent_at = self.pending.pop(message["id"], None)
                        if sent_at is not None:
                            message["rtt_ms"] = (time.monotonic() - sent_at) * 1000
                        self.acks[message["id"]] = message
                        if len(self.acks) > 64:
                            self.acks.pop(next(iter(self.acks)))
                        self.last_ack = message
                        if message.get("op") != "ping":
                            self.last_action_ack = message
                    elif kind == "error":
                        self.error = message["error"]
                # Fresh incoming simulation state maintains heartbeat even if drawing stalls.
                # Hardware heartbeat must continue to depend on the operator's control loop.
                if (kind == "state" and self.keep_simulation_alive
                        and time.monotonic() - last_simulation_ping >= 0.25):
                    self.send("ping")
                    last_simulation_ping = time.monotonic()
        except (ValueError, KeyError, RuntimeError, OSError) as exc:
            with self.lock:
                self.error = str(exc)

    def send(self, operation, **payload):
        if operation == "circle_start" and self.backend == "hardware":
            from lerobot_arm_control.planning.circle_simulation import validate_hardware_entry
            from lerobot_arm_control.tasks.circle.circle_trajectory import Circle

            hello, state, received, *_ = self.snapshot()
            if (not hello or not state or time.monotonic() - received > .5
                    or not state.get("qpos")):
                raise ValueError("fresh calibrated feedback required for local circle validation")
            circle = Circle()
            circle.validate_laps(payload.get("laps"))
            payload["validation"] = validate_hardware_entry(circle, state["qpos"])
        with self.lock:
            if len(self.pending) >= 16:
                raise RuntimeError("too many unacknowledged commands")
            self.next_id += 1
            command_id = self.next_id
            self.pending[command_id] = time.monotonic()
        with self.write_lock:
            self.process.stdin.write(json.dumps(
                {"version": VERSION, "id": command_id, "op": operation, **payload},
                allow_nan=False,
            ) + "\n")
            self.process.stdin.flush()
        return command_id

    def snapshot(self):
        with self.lock:
            return self.hello, self.latest, self.received_at, self.received_count, self.last_ack

    def wait_ack(self, command_id, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                ack = self.acks.get(command_id)
            if ack is not None:
                return ack
            if self.error or self.process.poll() is not None:
                raise RuntimeError(self.error or "state/control worker exited")
            time.sleep(0.01)
        raise TimeoutError("no command acknowledgement")

    def close(self):
        self.keep_simulation_alive = False
        if self.process.stdin:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def angles_for_state(hello, state, ranges):
    if hello["backend"] == "mujoco":
        return state["qpos"]
    return raw_to_angles(state["raw"], hello["raw_limits"], ranges)


def calibration_text(count, testing, joint, degrees, mode, notice):
    instructions = "R: next pose, C: capture" if mode else "B: enter calibration"
    motion = "gripper verification RUNNING" if testing else "motor writes OFF"
    return (f"\nAngle alignment {count}/6 | {motion}\nReference: {joint} {degrees} deg"
            f" | {instructions}\n{notice}")


def run(args):
    import mujoco

    model_path = args.model or resource_path('assets/so101/scene_mujoco.xml')
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    if tuple(model.joint(i).name for i in range(model.njnt)) != JOINTS:
        raise ValueError("unexpected model joint map")
    ranges = model.jnt_range.tolist()
    link = Link(args.host, args.remote_root, args.backend, allow_gripper=args.verify_gripper,
                hardware_control=args.hardware_control, circle=args.circle,
                keep_simulation_alive=args.backend == "mujoco",
                simulation_model=model_path,
                control_log=getattr(args, "control_log", None),
                arm=getattr(args, 'arm', 'follower'))
    started = time.monotonic()
    try:
        while link.latest is None:
            if link.error or link.process.poll() is not None:
                raise RuntimeError(link.error or "state/control worker exited")
            if time.monotonic() - started > 20:
                raise TimeoutError("no worker state within 20 seconds")
            time.sleep(0.02)
        hello, state, _, _, _ = link.snapshot()
        calibration = None
        calibration_error = None
        if args.backend == "hardware":
            from lerobot_arm_control.calibration.angle_calibration import AngleCalibration

            fingerprint = {**hello["fingerprint"], "model": "so101",
                           "mjcf_sha256": hashlib.sha256(
                               (resource_path('assets/so101/so101.xml')).read_bytes()).hexdigest()}
            try:
                calibration = AngleCalibration(args.calibration, fingerprint)
            except ValueError as exc:
                calibration_error = str(exc)
        print(json.dumps({"event": "connected", "worker_location": link.worker_location,
                          "hello": hello}), flush=True)
        ack = link.wait_ack(link.send("ping"))
        if not ack["ok"]:
            raise RuntimeError(ack)
        print(json.dumps({"event": "ping_ack", **ack}), flush=True)
        if (args.verify_gripper or args.hardware_control) and (
                calibration is None or calibration.count() != 6):
            raise ValueError("hardware motion requires all six validated angle mappings")
        if args.hardware_control and not hello.get("hardware_control"):
            raise ValueError("remote manual control was not enabled")
        if args.smoke_test:
            if args.backend == "mujoco":
                target = list(state["target"])
                target[4] += 0.06
                ack = link.wait_ack(link.send("target", qpos=target))
                if not ack["ok"]:
                    raise RuntimeError(ack)
                deadline = time.monotonic() + 2
                initial = state["qpos"][4]
                while time.monotonic() < deadline:
                    if abs(link.snapshot()[1]["qpos"][4] - initial) > 0.02:
                        break
                    time.sleep(0.02)
                else:
                    raise RuntimeError("local simulation did not respond to target")
                print(json.dumps({"event": "local_motion_verified", "ack": ack}), flush=True)
            else:
                ack = link.wait_ack(link.send("gripper_test", delta=16, execute=False))
                if not ack["ok"]:
                    raise RuntimeError(ack)
                print(json.dumps({"event": "gripper_preview_ack", **ack}), flush=True)
        viewer = None
        reference_renderer = None
        keys = queue.Queue()
        if not args.headless:
            from lerobot_arm_control.visualization.control_viewer import ControlViewer

            data.qpos[:] = angles_for_state(hello, state, ranges)
            mujoco.mj_forward(model, data)
            viewer = ControlViewer(model, data, keys.put,
                                   f"SO101 {getattr(args, 'arm', 'follower')} - live arm control"
                                   if args.backend == "hardware"
                                   else "SO101 - local MuJoCo control",
                                   hold_on_focus_loss=args.backend == "hardware")
            with viewer.lock():
                viewer.cam.lookat[:] = (data.xpos[1:].min(axis=0)
                                       + data.xpos[1:].max(axis=0)) / 2
                viewer.cam.distance = 0.75
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
            if args.backend == "hardware" and not args.hardware_control:
                reference_renderer = mujoco.Renderer(model, height=300, width=420)
                reference_data = mujoco.MjData(model)
                reference_camera = mujoco.MjvCamera()
                reference_camera.distance = 0.75
                reference_camera.azimuth = 135
                reference_camera.elevation = -22
            print("LOCAL_MUJOCO_VIEWER_READY", flush=True)
        selected = 5 if args.hardware_control else 0
        reference_degrees = 0
        calibration_mode = False
        calibration_notice = calibration_error or "B: calibration mode; capture measured poses only"
        last_reference = None
        screenshot_saved = False
        last_ping = last_log = 0.0
        jogs = JogCommands()
        loop_started = time.monotonic()
        baseline_count = link.received_count
        circle_startup = None
        if args.circle:
            if args.hardware_control:
                circle_startup = ResetThenCircle(link, getattr(args, "circle_laps", None))
            else:
                laps = getattr(args, "circle_laps", None)
                ack = link.wait_ack(link.send(
                    "circle_start", **({"laps": laps} if laps is not None else {})))
                if not ack["ok"]:
                    raise RuntimeError(ack.get("error", "circle start failed"))
                print(json.dumps({"event": "circle_started", "ack": ack}), flush=True)
        verification_id = None
        verification_done = False
        if args.verify_gripper:
            verification_id = link.send("gripper_test", delta=32, execute=True)
        while args.duration is None or time.monotonic() - loop_started < args.duration:
            hello, state, received_at, count, ack = link.snapshot()
            now = time.monotonic()
            age = now - received_at
            if age >= 0.5:
                jogs.clear()
            if link.error:
                raise RuntimeError(link.error)
            if link.process.poll() is not None:
                raise RuntimeError("state/control worker disconnected; last state is frozen")
            if viewer and not viewer.is_running():
                break
            if verification_id and not verification_done:
                with link.lock:
                    verification_ack = link.acks.get(verification_id)
                if verification_ack:
                    verification_done = True
                    proof = ROOT / "logs/gripper-verification-after-calibration.json"
                    proof.write_text(json.dumps(verification_ack, indent=2) + "\n")
                    print(json.dumps({"event": "gripper_verification_finished",
                                      "ok": verification_ack["ok"], "proof": str(proof)}),
                          flush=True)
                elif now - loop_started > 12:
                    raise TimeoutError("gripper verification did not finish within 12 seconds")
            testing = bool(verification_id and not verification_done)
            if (args.backend != "mujoco" and not testing and age < 0.5
                    and now - last_ping > 0.25):
                link.send("ping")
                last_ping = now
            while not keys.empty():
                key = keys.get_nowait()
                if testing:
                    continue
                if circle_startup and key in (69, 83, 88, 32, 82, 80, 265, 264):
                    circle_startup.cancel()
                if 49 <= key <= 54:
                    selected = key - 49
                elif age < 0.5 and args.hardware_control and key == 69:
                    jogs.clear()
                    link.send("enable_all")
                elif age < 0.5 and args.hardware_control and key == 83:
                    jogs.clear()
                    link.send("enable_joint", joint=JOINTS[selected])
                elif age < 0.5 and args.hardware_control and key == 88:
                    jogs.clear()
                    link.send("release_joint", joint=JOINTS[selected])
                elif age < 0.5 and args.hardware_control and key == 32:
                    jogs.clear()
                    link.send("hold")
                elif age < 0.5 and args.hardware_control and key in (82, 80):
                    jogs.clear()
                    link.send("reset" if key == 82 else "save_rest_pose")
                elif args.backend == "hardware" and not args.hardware_control and key == 66:
                    calibration_mode = not calibration_mode
                elif args.backend == "hardware" and calibration_mode and key == 82:
                    reference_degrees = (reference_degrees + 30) % 90
                elif args.backend == "hardware" and calibration_mode and key == 67:
                    try:
                        if age >= 0.5 or calibration is None:
                            raise ValueError(calibration_error or "stale feedback")
                        with link.lock:
                            samples = list(link.states)
                        fit = calibration.capture(JOINTS[selected], reference_degrees, samples)
                        suffix = "; check passed" if fit else "; more poses needed"
                        calibration_notice = (
                            f"Saved {JOINTS[selected]} {reference_degrees} deg" + suffix)
                    except ValueError as exc:
                        calibration_notice = str(exc)
                elif age < 0.5 and args.backend == "hardware" and key in (265, 264):
                    joint = JOINTS[selected]
                    if args.hardware_control:
                        control = state["control"]
                        measured = calibration.angles(state["raw"], [0] * 6)[selected]
                        jogs.request(joint, 0.03 if key == 265 else -0.03, control, measured)
                    else:
                        fit = calibration.fit(joint) if calibration else None
                        delta = round(0.03 / fit["radians_per_tick"]) if fit else 20
                        link.send("joint_preview", joint=joint,
                                  delta=delta if key == 265 else -delta)
                elif age < 0.5 and args.backend == "mujoco" and key in (265, 264):
                    target = list(state["target"])
                    target[selected] += 0.03 if key == 265 else -0.03
                    target[selected] = max(ranges[selected][0], min(ranges[selected][1],
                                                                 target[selected]))
                    link.send("target", qpos=target)
                elif age < 0.5 and args.backend == "mujoco" and key == 32:
                    link.send("hold")
                elif (age < 0.5 and args.backend == "hardware"
                      and not args.hardware_control and key == 71):
                    link.send("gripper_test", delta=16, execute=False)
            if circle_startup:
                circle_startup.advance(link, state, age)
            if args.hardware_control and age < 0.5:
                if state["control"]["parking"]["phase"] in ("moving", "settling"):
                    jogs.clear()
                else:
                    jogs.flush(link, state["control"])
            qpos = angles_for_state(hello, state, ranges)
            if calibration:
                qpos = calibration.angles(state["raw"], qpos)
            if viewer:
                with viewer.lock():
                    data.qpos[:] = qpos
                    data.qvel[:] = state.get("qvel", [0.0] * 6)
                    data.time = state.get("sim_time", count / hello["hz"])
                    mujoco.mj_forward(model, data)
                status = "LIVE" if age < 0.5 else "STALE: controls blocked"
                mode = ("Local MuJoCo | 1-6 select, arrows +/-0.03 rad, Space hold"
                        if args.backend == "mujoco" else
                        "REAL ARM | 1-6 select, E enable all, S enable joint, Space hold, X release"
                        if args.hardware_control else
                        "LIVE ARM | 1-6 select, arrows: control preview, B: calibration")
                detail = (f"{status} | age {age * 1000:.0f} ms | frame {state['seq']}\n"
                          f"Selected: {JOINTS[selected]} | RTT {ack.get('rtt_ms', 0):.1f} ms\n"
                          + ", ".join(f"{v:.3f}" for v in qpos))
                if state.get("circle"):
                    circle = state["circle"]
                    detail += (f"\nCircle: {circle['phase']} | radius "
                               f"{circle['radius_m'] * 1000:.0f} mm"
                               f" | period {circle['period_seconds']:.1f} s\nSpace: stop and hold")
                    if circle_startup:
                        detail += "\n" + circle_startup.notice
                if args.backend == "hardware":
                    calibrated = calibration.count() if calibration else 0
                    if args.hardware_control:
                        control = state["control"]
                        rate = control.get("update_rate_hz")
                        if rate is not None:
                            detail = (f"Control: {rate:.1f} Hz | feedback: {hello['hz']:.0f} Hz\n"
                                      + detail)
                        parking = control["parking"]
                        viewer.set_reset_status(parking)
                        detail += (f"\nReset (R): {parking['phase']}"
                                   f"\n{parking['notice']}")
                        joint = JOINTS[selected]
                        enabled = f"{len(control['enabled'])}/6 (E: all, S: selected)"
                        detail += (f"\nREAL CONTROL | calibrated {calibrated}/6"
                                   f" | enabled: {enabled}"
                                   f"\nActual {qpos[selected]:.4f} rad | "
                                   f"target {control['target_qpos'][joint]:.4f} rad | "
                                   f"error {control['error_rad'].get(joint, 0):+.4f} rad"
                                   f"\nLeRobot position control | {control['control_hz']} Hz"
                                   " | each arrow 0.03 rad"
                                   "\nGripper: hold Up to open, Down to close"
                                   "\nX: support arm before releasing torque; close: HOLD")
                        if control.get("position_feedback"):
                            correction = control["position_feedback"]["correction_rad"][joint]
                            detail += (f"\nPosition feedback ON | correction "
                                       f"{math.degrees(correction):+.2f} deg")
                        entry = control["boundary_entry_qpos"][joint]
                        if abs(entry - qpos[selected]) > 0.003:
                            detail += (f"\nBoundary entry: {math.degrees(entry):.2f} deg; "
                                       "jog inward only")
                        if control["stopped"]:
                            reasons = list(dict.fromkeys(control["stopped"].values()))
                            detail += (f"\nHold: {len(control['stopped'])} joint(s): "
                                       + "; ".join(reasons))
                        if control.get("status_warning"):
                            warning = control["status_warning"]
                            detail += (f"\nRecovered motor status: {warning['joint']} "
                                       f"{warning['reported_status']} -> 0 (confirmed)")
                        if control.get("last_fault"):
                            detail += "\nRecorded fault: " + control["last_fault"]["reason"]
                            if control["last_fault"]["telemetry"]["Status"].get("gripper", 0) & 1:
                                detail += ("\nCheck gripper power/cable; "
                                           "E cannot repair voltage faults")
                        status_code = int(state["telemetry"]["Status"][joint])
                        voltage = state["telemetry"]["Present_Voltage"][joint] / 10
                        detail += (f"\nVoltage: {voltage:.1f} V | Status {status_code}: "
                                   + motor_fault_text(status_code))
                        if parking["remaining_degrees"] is not None:
                            detail += f"\nRest pose error: {parking['remaining_degrees']:.2f} deg"
                    else:
                        detail += calibration_text(
                            calibrated, testing, JOINTS[selected], reference_degrees,
                            calibration_mode, calibration_notice)
                    reference_key = (selected, reference_degrees)
                    if reference_renderer and reference_key != last_reference:
                        reference_data.qpos[:] = 0
                        reference_data.qpos[selected] = reference_degrees * 3.141592653589793 / 180
                        mujoco.mj_forward(model, reference_data)
                        reference_camera.lookat[:] = (
                            reference_data.xpos[1:].min(axis=0)
                            + reference_data.xpos[1:].max(axis=0)) / 2
                        reference_renderer.update_scene(reference_data, camera=reference_camera)
                        viewer.set_images((mujoco.MjrRect(12, 55, 420, 300),
                                           reference_renderer.render()))
                        last_reference = reference_key
                action_ack = link.last_action_ack
                if action_ack:
                    message = "accepted" if action_ack["ok"] else action_ack.get("error", "failed")
                    detail += f"\nCommand {action_ack['id']}: {message}"
                    if isinstance(action_ack.get("result"), dict):
                        preview = action_ack["result"]
                        if preview.get("preview"):
                            detail += (f" | preview {preview['start_raw']}"
                                       f" -> {preview['target_raw']} ticks")
                display = "\n".join(textwrap.fill(line, width=74)
                                    for line in (mode + "\n" + detail).splitlines())
                overlays = [(mujoco.mjtFontScale.mjFONTSCALE_100, None, display, "")]
                if reference_renderer:
                    overlays.append((None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                     "REFERENCE: other joints at zero; measure 0/30/60 deg", ""))
                viewer.set_texts(overlays)
                capture_path = getattr(args, 'screenshot', None) if not screenshot_saved else None
                if capture_path:
                    capture_path.parent.mkdir(parents=True, exist_ok=True)
                viewer.sync(capture_path=capture_path)
                if capture_path:
                    screenshot_saved = True
            if now - last_log > 1:
                report = {"backend": args.backend, "worker_location": link.worker_location,
                          'arm': getattr(args, 'arm', 'follower'),
                          'fingerprint': hello.get('fingerprint'),
                          'joint_calibration_file': (str(args.calibration)
                                                     if args.backend == 'hardware' else None),
                          "state": state,
                          "state_age_ms": age * 1000, "frames_received": count,
                          "last_ack": ack, "qpos_display": qpos,
                          "last_action_ack": link.last_action_ack,
                          "calibration": ("model" if args.backend == "mujoco" else
                                          f"{calibration.count() if calibration else 0}/6 aligned"),
                          "reference_joint": JOINTS[selected],
                          "reference_degrees": reference_degrees,
                          "calibration_notice": calibration_notice,
                          "viewer": viewer.diagnostics() if viewer else None}
                if args.log:
                    args.log.parent.mkdir(parents=True, exist_ok=True)
                    args.log.write_text(json.dumps(report, indent=2) + "\n")
                print(json.dumps({"event": "live", "seq": state["seq"],
                                  "state_age_ms": age * 1000,
                                  "rtt_ms": ack.get("rtt_ms")}), flush=True)
                last_log = now
            time.sleep(1 / 60)
        elapsed = time.monotonic() - loop_started
        rate = (link.received_count - baseline_count) / elapsed
        if args.headless and args.duration and (link.received_count - baseline_count < 2):
            raise RuntimeError("no continuous worker state stream")
        print(json.dumps({"event": "finished", "received_hz": rate}), flush=True)
    finally:
        try:
            if "reference_renderer" in locals() and reference_renderer:
                reference_renderer.close()
        finally:
            try:
                if "viewer" in locals() and viewer:
                    viewer.close()
            finally:
                link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="lerobot", help="Hardware worker SSH host")
    parser.add_argument("--remote-root", default="/home/xy/Research/lerobot",
                        help="Hardware worker project directory; unused in simulation")
    parser.add_argument("--backend", choices=("hardware", "mujoco"), default="hardware")
    parser.add_argument('--arm', default='follower', help='Configured physical arm identity')
    parser.add_argument("--model", type=Path)
    parser.add_argument("--calibration", type=Path,
                        help='Defaults to data/calibration/<arm>_mujoco.json')
    parser.add_argument("--headless", action="store_true")
    parser.add_argument('--screenshot', type=Path, help='Save the first rendered local GUI frame')
    parser.add_argument("--smoke-test", action="store_true")
    parser.add_argument("--verify-gripper", action="store_true",
                        help="Execute one +32 tick gripper roundtrip after onsite clearance")
    parser.add_argument("--hardware-control", action="store_true",
                        help="Control real calibrated motors from this single window")
    parser.add_argument("--circle", action="store_true",
                        help="Start the saved circle in local simulation or on remote hardware")
    parser.add_argument("--circle-laps", type=int,
                        help="Finite lap count; default repeats until Space or close")
    parser.add_argument("--control-log",
                        help="Optional full-rate trace path on the remote hardware worker")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--log", type=Path, default=ROOT / "logs/realtime-latest.json")
    args = parser.parse_args()
    if Path(args.arm).name != args.arm or args.arm in ('.', '..'):
        parser.error('arm must be a configured name, not a path')
    if args.arm != 'follower' and (args.hardware_control or args.verify_gripper):
        parser.error('Additional arms support read-only angle calibration only')
    if args.calibration is None:
        args.calibration = ROOT/'data/calibration'/f'{args.arm}_mujoco.json'
    if args.verify_gripper and args.backend != "hardware":
        parser.error("--verify-gripper is available only for the physical follower")
    if args.hardware_control and (args.backend != "hardware" or args.verify_gripper):
        parser.error("hardware control requires hardware backend without --verify-gripper")
    if args.circle and (args.smoke_test or args.verify_gripper or
                        (args.backend == "hardware" and not args.hardware_control)):
        parser.error("circle requires MuJoCo or calibrated hardware control")
    if args.circle_laps is not None and (not args.circle or args.circle_laps < 1):
        parser.error("--circle-laps requires --circle and a positive count")
    if args.control_log and not args.hardware_control:
        parser.error("--control-log requires hardware control")
    run(args)


if __name__ == "__main__":
    main()
