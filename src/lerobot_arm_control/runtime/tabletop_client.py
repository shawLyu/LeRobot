"""Local storage and MuJoCo mirror of remote tabletop perception via SSH."""

import argparse
import base64
import io
import json
import math
import shutil
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.core.files import atomic_json, file_digest
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.core.processes import remote_python
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.visualization.tabletop_scene import (
    scene_arms,
    scene_xml,
    update_scene_robots,
)

ROOT = workspace_root()



class TableLink:
    def __init__(self, host, remote_root, output, *, server_args=()):
        self.server_args = tuple(server_args)
        command = remote_python(remote_root, "lerobot_arm_control.runtime.tabletop_server",
                                server_args)
        self.process = subprocess.Popen(
            ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
             "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", host, command],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )
        self.output = output
        self.hardware_control = "--hardware-control" in server_args
        (output / "frames").mkdir(parents=True)
        (output / "annotated").mkdir()
        (output / "native").mkdir()
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.acks = {}
        self.next_command_id = 0
        self.planner = self.motor = self.appearance = None
        self.push_enabled = "--push-t" in server_args
        self.stopped = threading.Event()
        self.latest = self.hello = self.error = None
        self.received_at = 0.0
        self.count = 0
        self.started = time.monotonic()
        self.saved_model_names = set()
        self.saved_placed_names = set()
        self.saved_robot_placed = False
        self.saved_arm_names = set()
        self.saved_appearance_version = 0
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        if self.hardware_control:
            self.heartbeat = threading.Thread(target=self._heartbeat, daemon=True)
            self.heartbeat.start()

    def _read(self):
        try:
            with ((self.output / "states.jsonl").open("a") as log,
                  (self.output / "motors.jsonl").open("a") as motor_log):
                for line in self.process.stdout:
                    message = json.loads(line)
                    received_at = time.monotonic()
                    if message.get("version") != 1:
                        raise ValueError("Tabletop stream protocol mismatch")
                    if message["type"] == "hello":
                        if ((self.hardware_control or self.push_enabled)
                                and message.get("control_config_sha256")
                                != CONTROL_CONFIG_SHA256):
                            raise ValueError(
                                "Local/remote control configuration differs; deploy code")
                        robot = message.get("robot") or {}
                        fingerprint = robot.get("fingerprint")
                        model_hash = (fingerprint.get("mjcf_sha256")
                                      if isinstance(fingerprint, dict) else None)
                        if message["config"].get("robot", {}).get("enabled"):
                            shutil.copytree(resource_path('assets/so101/assets'),
                                            self.output / "robot_assets")
                            shutil.copyfile(resource_path('assets/so101/so101.xml'),
                                            self.output / "so101_source.xml")
                        if fingerprint is not None:
                            source = resource_path('assets/so101/so101.xml')
                            if file_digest(source) != model_hash:
                                raise ValueError("Local SO101 model differs from joint calibration")
                        marker = base64.b64decode(message.pop("tag_png"), validate=True)
                        (self.output / "marker.png").write_bytes(marker)
                        atomic_json(self.output / "calibration.json", message)
                        if message["config"].get("appearance", {}).get("enabled"):
                            from lerobot_arm_control.visualization.appearance_worker import (
                                AppearanceWorker,
                            )

                            self.appearance = AppearanceWorker(message["config"])
                        if self.push_enabled:
                            from lerobot_arm_control.tasks.push_t.push_t_local import LocalPush
                            from lerobot_arm_control.transport.push_t_link import MotorLink

                            if (message.get("planning_location") != "local"
                                    or message.get("execution_protocol") != 1):
                                raise ValueError(
                                    "Remote worker must accept local plans; deploy code")
                            if self.hardware_control and (
                                    not model_hash or robot.get('status') in ('error', 'stale')
                                    or robot.get('motion_available') is not True):
                                reason = robot.get('error') or (
                                    'verified motor identity and hardware executor are unavailable')
                                raise RuntimeError(
                                    'Remote SO101 is not ready for hardware control: '+reason)
                            self.motor = (MotorLink(self, model_hash)
                                          if self.hardware_control else None)
                            feedback = SimpleNamespace(metadata=robot, motor=self.motor)
                            self.planner = LocalPush(
                                message["config"], feedback, self.output / "push_t", marker,
                                execute=self.hardware_control,
                                publish=lambda value: self.send("push_result", result=value))
                        with self.lock:
                            self.hello = message
                        continue
                    if message["type"] == "ack":
                        with self.lock:
                            self.acks[message.get('id')] = message
                            if len(self.acks) > 128:
                                self.acks.pop(next(iter(self.acks)))
                        if self.motor:
                            self.motor.acknowledge(message)
                        if not message["ok"] and self.planner:
                            self.planner.command("push_stop", reason=message.get(
                                'error', 'remote command rejected: hold'))
                        continue
                    if message["type"] != "state" or self.hello is None:
                        raise ValueError("Unexpected tabletop message")
                    if self.appearance:
                        self.appearance.offer(message)
                        message.update(self.appearance.take())
                    if self.motor:
                        self.motor.update(message.get("push_t", {}))
                    if self.planner:
                        self.planner.observe(message)
                        message["push_t"] = self.planner.status()
                    raw = base64.b64decode(message.pop("jpeg"), validate=True)
                    annotation = base64.b64decode(message.pop("annotated_jpeg"), validate=True)
                    for sample in message.pop("motor_samples", []):
                        motor_log.write(json.dumps(sample, allow_nan=False) + "\n")
                    motor_log.flush()
                    if "appearance_pngs" in message:
                        directory = self.output / "appearance"
                        directory.mkdir(exist_ok=True)
                        for filename, data in message.pop("appearance_pngs").items():
                            if Path(filename).name != filename or not filename.endswith(".png"):
                                raise ValueError("Unexpected observed texture filename")
                            (directory / filename).write_bytes(
                                base64.b64decode(data, validate=True))
                        atomic_json(directory / "appearance.json", message["appearance"])
                    filename = f'{message["sequence"]:08d}.jpg'
                    if "native_jpeg" in message:
                        (self.output / "native" / filename).write_bytes(
                            base64.b64decode(message.pop("native_jpeg"), validate=True))
                        message["native_frame"] = f"native/{filename}"
                    (self.output / "frames" / filename).write_bytes(raw)
                    (self.output / "annotated" / filename).write_bytes(annotation)
                    message.update(frame=f"frames/{filename}", annotated=f"annotated/{filename}",
                                   received_unix_ns=time.time_ns(),
                                   local_elapsed_s=received_at-self.started)
                    log.write(json.dumps(message, allow_nan=False) + "\n")
                    log.flush()
                    atomic_json(self.output / "latest.json", message)
                    names = set(message["models"])
                    placed = set(message["objects"])
                    robot_placed = "position" in message.get("robot", {}).get("registration", {})
                    arm_names = {arm['arm'] for arm in scene_arms(message, self.hello['config'])}
                    appearance_version = message.get("appearance", {}).get("version", 0)
                    if (names != self.saved_model_names or not placed <= self.saved_placed_names
                            or (robot_placed and not self.saved_robot_placed)
                            or appearance_version != self.saved_appearance_version
                            or not arm_names <= self.saved_arm_names):
                        manifest = {"units": message["units"], "models": message["models"],
                                    "reference": "anchor AprilTag center; +X right; +Y up; +Z out",
                                    "anchor": {key: self.hello["config"][key]
                                               for key in ("marker_id", "dictionary")},
                                    "table_extent": "workspace patch; desk perimeter not measured",
                                    "wood_height_measured": message["wood_height_measured"]}
                        if "appearance" in message:
                            manifest["appearance"] = message["appearance"]
                        manifest['arms'] = {arm['arm']: {
                            'registration': arm['registration'],
                            'joint_pose_source': arm['joint_pose_source'], 'qpos': arm['qpos']}
                            for arm in scene_arms(message, self.hello['config'])}
                        if self.hello.get("robot"):
                            manifest["robot"] = {**self.hello["robot"],
                                'registration': message['robot']['registration']}
                        atomic_json(self.output / "model.json", manifest)
                        xml = scene_xml(message, self.hello["config"], "robot_assets")
                        (self.output / "scene.xml").write_text(xml)
                        assets = ROOT / "assets/tabletop"
                        assets.mkdir(exist_ok=True)
                        if (self.output / "appearance").exists():
                            shutil.copytree(self.output / "appearance", assets / "appearance",
                                            dirs_exist_ok=True)
                        if (self.output / "robot_assets").exists():
                            shutil.copytree(self.output / "robot_assets", assets / "robot_assets",
                                            dirs_exist_ok=True)
                        marker = (self.output / "marker.png").read_bytes()
                        (assets / "marker.png").write_bytes(marker)
                        (assets / "scene.xml").write_text(xml)
                        atomic_json(assets / "model.json", manifest)
                        self.saved_model_names = names
                        self.saved_placed_names.update(placed)
                        self.saved_robot_placed |= robot_placed
                        self.saved_arm_names.update(arm_names)
                        self.saved_appearance_version = appearance_version
                    with self.lock:
                        self.latest = (message, annotation)
                        self.received_at = received_at
                        self.count += 1
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def snapshot(self):
        with self.lock:
            return self.hello, self.latest, self.received_at, self.count, self.error

    def _heartbeat(self):
        try:
            while not self.stopped.wait(.1):
                if ((self.planner or '--pick-t' in self.server_args) and self.process.poll() is None
                        and time.monotonic() - self.received_at < .5):
                    self.send("ping")
        except (BrokenPipeError, OSError):
            self.stopped.set()

    def send(self, operation, **payload):
        if operation in ('push_start', 'push_stop') and self.planner and not payload:
            if operation == 'push_start':
                self.planner.command(operation)
                return None
            # A planner stop also cancels its remote MotorLink through command().
            self.planner.command(operation)
            if self.motor:
                return None
        if operation not in ('stop', 'push_stop', 'push_execute', 'push_result', 'ping',
                             'pick_execute', 'pick_stop', 'pick_result'):
            raise ValueError('unsupported remote operation')
        with self.command_lock:
            self.next_command_id += 1
            command_id = self.next_command_id
            message = json.dumps({'version': 1, 'id': command_id, 'op': operation,
                                  'control_config_sha256': CONTROL_CONFIG_SHA256, **payload},
                                 allow_nan=False)+'\n'
            if len(message) > 65536:
                raise ValueError('local plan too large')
            self.process.stdin.write(message)
            self.process.stdin.flush()
        return command_id

    def close(self):
        self.stopped.set()
        if self.planner:
            self.planner.close()
        if self.appearance:
            self.appearance.close()
        if self.process.poll() is None:
            try:
                self.send('stop')
            except BrokenPipeError:
                pass
        self.process.stdin.close()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=3)
        self.reader.join(timeout=3)
        if self.hardware_control:
            self.heartbeat.join(timeout=1)


def run(args):
    import mujoco

    output = args.output or ROOT / "data/tabletop" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    server_args = ['--stream-fps', str(max(10, args.fps) if args.push_t else args.fps)]
    if args.push_t:
        server_args.extend(('--push-t', '--omit-native-images'))
    if args.hardware_control:
        server_args.append('--hardware-control')
    if args.recalibrate_base:
        server_args.append('--recalibrate-base')
    if args.recalibrate_table:
        server_args.append('--recalibrate-table')
    if args.camera_only:
        server_args.append('--camera-only')
    link = TableLink(args.host, args.remote_root, output, server_args=server_args)
    viewer = model = data = None
    model_names, last_image_sequence, last_view_size = set(), None, (0, 0)
    placed_names = set()
    robot_was_placed = False
    scene_arm_names = set()
    appearance_version = 0
    screenshot_saved = False
    started, last_report = time.monotonic(), 0
    print(json.dumps({"event": "tabletop_session", "output": str(output)}), flush=True)
    try:
        while args.duration is None or time.monotonic() - started < args.duration:
            hello, latest, received_at, count, error = link.snapshot()
            if error:
                raise RuntimeError(error)
            if link.process.poll() is not None:
                raise RuntimeError("Remote perception disconnected; recording stopped")
            if latest is None:
                if time.monotonic() - started > 20:
                    raise TimeoutError("No camera state arrived within 20 seconds")
                time.sleep(.02)
                continue
            state, annotation = latest
            names = set(state["models"])
            robot_placed = "position" in state.get("robot", {}).get("registration", {})
            current_arm_names = {arm["arm"] for arm in scene_arms(state, hello["config"])}
            state_appearance = state.get("appearance", {}).get("version", 0)
            if (model is None or names != model_names
                    or not set(state["objects"]) <= placed_names
                    or (robot_placed and not robot_was_placed)
                    or state_appearance != appearance_version
                    or not current_arm_names <= scene_arm_names):
                if viewer:
                    viewer.close()
                model_path = output / "scene.xml"
                if not names:
                    model_path = output / "awaiting-model.xml"
                    model_path.write_text(scene_xml(state, hello["config"], "robot_assets"))
                model = mujoco.MjModel.from_xml_path(str(model_path))
                data = mujoco.MjData(model)
                if model.nkey:
                    mujoco.mj_resetDataKeyframe(model, data, 0)
                for geom in range(model.ngeom):
                    material = model.geom_matid[geom]
                    if model.geom_group[geom] == 2 and material >= 0:
                        # Explicit geom alpha overrides its material; carry the original
                        # RGB along so uncertainty transparency preserves CAD colors.
                        model.geom_rgba[geom, :3] = model.mat_rgba[material, :3]
                model_names = names
                placed_names.update(state["objects"])
                robot_was_placed |= robot_placed
                scene_arm_names.update(current_arm_names)
                appearance_version = state_appearance
                if not args.headless:
                    import glfw

                    from lerobot_arm_control.visualization.control_viewer import ControlViewer

                    def on_key(key):
                        if key == glfw.KEY_ESCAPE:
                            glfw.set_window_should_close(viewer.window, True)
                        elif args.push_t and key == glfw.KEY_P:
                            link.send('push_start')
                        elif args.push_t and key == glfw.KEY_SPACE:
                            link.send('push_stop')

                    title = ("Table calibration - camera and arm bases"
                             if getattr(args, "calibration_mode", False)
                             else "Tabletop + SO101 - live mirror")
                    viewer = ControlViewer(model, data, on_key, title)
                    scale = hello["config"]["marker_size_m"] or 1
                    workspace = model.geom("tabletop_workspace")
                    viewer.cam.lookat[:] = [*workspace.pos[:2], 0]
                    viewer.cam.distance = max(scale*7, max(workspace.size[:2])*4)
                    viewer.cam.azimuth, viewer.cam.elevation = 90, -70
                    if appearance_version and "camera_position" in state:
                        observer = state["camera_position"]
                        dx, dy = observer[0]-workspace.pos[0], observer[1]-workspace.pos[1]
                        viewer.cam.azimuth = 180 + math.degrees(math.atan2(dy, dx))
                        viewer.cam.elevation = -math.degrees(
                            math.atan2(observer[2], math.hypot(dx, dy)))
                        viewer.cam.distance = max(math.sqrt(dx*dx+dy*dy+observer[2]**2)*1.3,
                                                  max(workspace.size[:2])*3.1)
                    last_image_sequence = None
                    print("LOCAL_TABLETOP_MUJOCO_READY", flush=True)
            age = time.monotonic() - received_at
            frame_age = age + state.get("capture_age_at_send_ms", 0)/1000
            valid = frame_age < .5 and state["marker_visible"]
            robot = state.get("robot", {})
            registration = robot.get('registration', state.get('base_registrations', {}).get(
                hello['config'].get('robot', {}).get('arm'), {}))
            registered = registration.get('status') == 'fixed' or (
                valid and registration.get("status") in ("tracked", "held"))
            estimated = valid and registration.get("status") in ("visual_estimate", "visual_cached")
            if model:
                update_scene_robots(model, data, state, hello["config"], age)
                for name in model_names:
                    obj = state["objects"].get(name) if valid else None
                    body = model.body(name).id
                    mocap = model.body_mocapid[body]
                    if obj:
                        height = state["models"][name]["height"] or (
                            (hello["config"]["marker_size_m"] or 1) * .002)
                        data.mocap_pos[mocap] = [*obj["xy"], height / 2 + (
                            (hello["config"]["marker_size_m"] or 1) * .002)]
                        data.mocap_quat[mocap] = [math.cos(obj["yaw"]/2), 0, 0,
                                                  math.sin(obj["yaw"]/2)]
                    for suffix in ("_cap", "_stem"):
                        status = obj.get("tracking_status", "observed") if obj else "lost"
                        alpha = {"observed": 1., "partial": 1., "static_cached": .65,
                                 "held": .45, "lost": .15}[status]
                        if obj is None and name not in state["objects"]:
                            alpha = 0
                        model.geom_rgba[model.geom(name+suffix).id, 3] = alpha
                mujoco.mj_forward(model, data)
            if viewer:
                import glfw
                import numpy as np
                from PIL import Image

                if not viewer.is_running():
                    break
                units = "meters" if state["units"] == "m" else "tag-side units (scale pending)"
                statuses = "\n".join(
                    f"{name}: {obj.get('tracking_status', 'observed')}"
                    for name, obj in state["objects"].items()) or "Waiting for first T observation"
                fixed_text = ('Base fixed (saved calibration; joints update live)'
                              if robot.get('status') == 'live' else
                              'Base fixed (saved calibration; joints unmeasured)')
                base_text = (fixed_text
                             if registration.get('status') == 'fixed' else
                             "Base aligned (mount calibration)" if registered else
                             "Base: VISUAL ESTIMATE" if estimated else
                             "Robot position: UNREGISTERED PREVIEW")
                other_bases = '\n'.join(
                    f"{name} base: {pose.get('status', 'disabled')} | joints: "
                    f"{state.get('robot_feedback', {}).get(name, {}).get('status', 'unmeasured')}"
                    for name, pose in state.get('base_registrations', {}).items()
                    if name != hello['config'].get('robot', {}).get('arm'))
                distances = '\n'.join(
                    f"{' / '.join(pair['arms'])}: {pair['planar_distance_m']*1000:.1f} mm"
                    for pair in state.get('base_distances', []))
                text = (f"Remote: {hello['perception_host']}\n"
                        f"Frames saved: {count} | age {frame_age:.2f}s\n"
                        f"Anchor ID {hello['config']['marker_id']}: "
                        f"{'TRACKING' if valid else 'LOST / STALE'}\n"
                        f"Coordinates: {units}\n"
                        f"{statuses}\n"
                        f"SO101 motors: {robot.get('status', 'disabled')} "
                        f"({'read only' if robot.get('read_only', True) else 'hardware control'})\n"
                        f"Base: {registration.get('status', 'disabled')}\n"
                        f"{base_text}\n"
                        f"Materials: {'observed' if appearance_version else 'pending'}\n"
                        "T: 48 x 60 mm | wood: 6 mm\n"
                        "Wood height: "
                        f"{'measured' if state['wood_height_measured'] else 'unknown'}\n"
                        "Desk: visible workspace plane\nEsc: close | drag: orbit | wheel: zoom")
                for detail in (other_bases, distances):
                    if detail:
                        text += '\n'+detail
                tool = state.get('tool_marker')
                if tool:
                    from lerobot_arm_control.visualization.ui import tool_marker_text

                    text += '\n'+tool_marker_text(tool)
                if robot.get('error'):
                    text += f"\nMotor error: {robot['error']}"
                if args.push_t:
                    push = state.get('push_t', {})
                    upcoming = push.get('next_push') or {}
                    text += (f"\n\nREMOTE Push-T: {push.get('phase', 'waiting')}\n"
                             f"Pushes: {push.get('pushes', 0)}\n"
                             f"Planning elapsed: {push.get('planning_elapsed_s', 0):.1f} s\n"
                             f"Executable: {push.get('executable', False)}\n"
                             f"Start requested: {push.get('start_requested', False)}\n"
                             f"Reason: {push.get('reason') or '-'}\n"
                             f"Motor: {push.get('motor', {}).get('phase', '-')}\n"
                             f"Motor reason: {push.get('motor', {}).get('reason') or '-'}\n"
                             f"Next stroke: {upcoming.get('distance_m', 0)*1000:.1f} mm\n"
                             f"Local: display only, {args.fps:g} FPS\n"
                             + ("P: start real push | Space: hold | Esc: stop and close"
                                if args.hardware_control else
                                "Read-only preview | Esc: close"))
                if getattr(args, "calibration_mode", False):
                    from lerobot_arm_control.calibration.table import calibration_text

                    text = calibration_text(hello["config"], state, frame_age)
                viewer.set_texts([(0, 0, text)])
                width, height = glfw.get_framebuffer_size(viewer.window)
                if state["sequence"] != last_image_sequence or (width, height) != last_view_size:
                    image = Image.open(io.BytesIO(annotation)).convert("RGB")
                    image.thumbnail((round(width*.35), round(height*.43)))
                    image_width, image_height = image.size
                    viewer.set_images((mujoco.MjrRect(12, 16, image_width, image_height),
                                       np.asarray(image)))
                    last_image_sequence, last_view_size = state["sequence"], (width, height)
                if (args.screenshot and not screenshot_saved and valid
                        and len(state["objects"]) == 2 and count >= 10
                        and (not hello["config"].get("robot", {}).get("enabled")
                             or registered or estimated)
                        and (not hello["config"].get("appearance", {}).get("enabled")
                             or appearance_version)):
                    args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                    viewer.sync(capture_path=args.screenshot)
                    screenshot_saved = True
                else:
                    viewer.sync()
            if time.monotonic() - last_report >= 5:
                last_report = time.monotonic()
                print(json.dumps({"event": "tabletop_live", "frames": count,
                                  "marker_visible": state["marker_visible"],
                                  "objects": {name: {key: obj[key] for key in (
                                      "xy", "yaw", "iou", "tracking_status") if key in obj}
                                              for name, obj in state["objects"].items()},
                                  "frame_age_s": frame_age, "units": state["units"],
                                  "robot_status": robot.get("status"),
                                  "robot_error": robot.get("error"),
                                  "base_status": registration.get("status"),
                                  'base_registrations': {
                                      name: pose.get('status') for name, pose
                                      in state.get('base_registrations', {}).items()},
                                  'joint_feedback': {name: feedback.get('status') for name, feedback
                                                     in state.get('robot_feedback', {}).items()},
                                  "tool_marker": {key: state.get('tool_marker', {}).get(key)
                                                  for key in ('id', 'status', 'position_valid',
                                                              'orientation_valid',
                                                              'source_short_edge_px')},
                                  "motor_sequence": robot.get("sequence"),
                                  "processing_ms": state["processing_ms"]}), flush=True)
            time.sleep(1/args.fps)
    finally:
        if viewer:
            viewer.close()
        link.close()
        hello, latest, *_ = link.snapshot()
        if hello and latest:
            from lerobot_arm_control.visualization.tabletop_snapshot import save_scene_snapshot

            save_scene_snapshot(output, latest[0], hello)
        summary = {"frames": link.count, "output": str(output), "model_names": sorted(model_names),
                   "elapsed_s": time.monotonic()-started, "error": link.error}
        atomic_json(output / "summary.json", summary)
        print(json.dumps({"event": "tabletop_stopped", **summary}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_remote_arguments(parser)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--screenshot", type=Path, help="Save the first tracked local GUI frame")
    parser.add_argument('--push-t', action='store_true',
                        help='Local Push-T planning from remote camera observations')
    parser.add_argument('--hardware-control', action='store_true',
                        help='Allow remote real execution after P and calibrated registration')
    parser.add_argument('--fps', type=float, default=5, help='Local display and stream rate')
    parser.add_argument('--recalibrate-base', action='store_true',
                        help='Archive saved fixed base pose and calibrate again from the marker')
    parser.add_argument('--recalibrate-table', action='store_true',
                        help='Archive the table map and both bases after moving table markers')
    parser.add_argument('--camera-only', action='store_true',
                        help='Observe camera and markers without opening the remote motor bus')
    args = parser.parse_args()
    if not 0 < args.fps <= 30:
        parser.error('fps must be in (0, 30]')
    if args.hardware_control and not args.push_t:
        parser.error('hardware-control requires push-t')
    if args.camera_only and args.push_t:
        parser.error('camera-only cannot execute Push-T')
    run(args)


if __name__ == "__main__":
    main()
