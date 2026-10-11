"""Stream remote camera frames and remote perception over an SSH stdio pipe."""

import argparse
import base64
import json
import math
import os
import sys
import threading
import time
import tomllib
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.perception.tabletop_perception import Perception, read_config
from lerobot_arm_control.perception.tabletop_registration import base_registration_configs
from lerobot_arm_control.transport.tabletop_stream import DisplayStream

ROOT = workspace_root()


def write_display(value, lock):
    with lock:
        print(value, flush=True)


def archive_reference(config, include_table=False):
    """Keep old observations recoverable before changing the physical table frame."""
    paths = [ROOT / entry['fixed_base']['file'] for entry in
             base_registration_configs(config) if entry.get('fixed_base')]
    if include_table:
        paths.append(ROOT / 'data/tabletop/static_map.json')
    suffix = '-before-' + str(time.time_ns())
    for path in paths:
        if path.exists():
            path.rename(path.with_name(path.stem + suffix + path.suffix))


def configure_focus(camera, config):
    """Lock the lens for repeatable geometry, with required hardware readback."""
    controls = config.get('camera_controls')
    if controls is None:
        return None
    if (set(controls) != {'autofocus', 'focus'} or controls['autofocus'] is not False
            or type(controls['focus']) is not int or not 0 <= controls['focus'] <= 1023):
        raise ValueError('Camera controls require autofocus=false and integer focus in 0..1023')
    for prop, value in ((cv2.CAP_PROP_AUTOFOCUS, 0), (cv2.CAP_PROP_FOCUS, controls['focus'])):
        accepted = camera.set(prop, value)
        observed = camera.get(prop)
        if not accepted or not math.isfinite(observed) or abs(observed-value) > .5:
            raise RuntimeError('Camera focus lock could not be confirmed')
    return {'autofocus': False, 'focus': camera.get(cv2.CAP_PROP_FOCUS), 'verified': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/tabletop.json")
    parser.add_argument("--image", type=Path, help="Process one recorded image, without camera")
    parser.add_argument('--push-t', action='store_true',
                        help='Receive locally planned Push-T segments and results')
    parser.add_argument('--pick-t', action='store_true',
                        help='Receive checked local T pick/place stages')
    parser.add_argument('--hardware-control', action='store_true',
                        help='Permit real Push-T after push_start and measured base registration')
    parser.add_argument('--stream-fps', type=float, default=5,
                        help='Display stream rate; independent of perception and motor control')
    parser.add_argument('--recalibrate-base', action='store_true',
                        help='Archive saved fixed base calibration before observing its marker')
    parser.add_argument('--recalibrate-table', action='store_true',
                        help='Archive the table map and both bases after moving table markers')
    parser.add_argument('--camera-only', action='store_true',
                        help='Capture camera/base/tool observations without opening a motor bus')
    parser.add_argument('--marker-only', action='store_true',
                        help='With camera-only, skip object fitting and base registration')
    parser.add_argument('--omit-native-images', action='store_true',
                        help='Keep native marker detection; stream smaller preview images')
    args = parser.parse_args()
    if not 0 < args.stream_fps <= 30:
        parser.error('stream-fps must be in (0, 30]')
    if args.push_t and args.pick_t:
        parser.error('choose push-t or pick-t')
    if args.hardware_control and not (args.push_t or args.pick_t):
        parser.error('hardware-control requires push-t or pick-t')
    if args.camera_only and (args.push_t or args.pick_t):
        parser.error('camera-only cannot execute robot policies')
    if args.marker_only and (not args.camera_only or args.recalibrate_base
                             or args.recalibrate_table):
        parser.error('marker-only requires camera-only without reference recalibration')
    config = read_config(args.config)
    if args.recalibrate_base and not args.recalibrate_table:
        fixed_bases = [entry['fixed_base'] for entry in base_registration_configs(config)
                       if entry.get('fixed_base')]
        if not fixed_bases:
            parser.error('recalibrate-base requires a fixed_base configuration')
    if args.recalibrate_base or args.recalibrate_table:
        archive_reference(config, include_table=args.recalibrate_table)
    perception = Perception(config)
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise ValueError(f"Cannot read {args.image}")
        print(json.dumps(perception.process(frame, 1, time.time_ns(),
                                           marker_only=args.marker_only), allow_nan=False))
        return
    static_map = ROOT / "data/tabletop/static_map.json"
    perception.restore_static_map(static_map)
    last_map_save = 0
    camera_config = tomllib.loads((ROOT / "configs/camera.toml").read_text())["camera"]
    device = camera_config["device"]
    if not os.access(device, os.R_OK | os.W_OK):
        raise PermissionError(f"Cannot access camera {device}")
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError("Cannot open remote camera")
    capture_size = config.get("capture_size", config["image_size"])
    if capture_size != config["image_size"]:
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, capture_size[0])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_size[1])
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    try:
        camera_controls = configure_focus(camera, config)
    except Exception:
        camera.release()
        raise
    stopped, lock = threading.Event(), threading.Lock()
    robot = None
    from lerobot_arm_control.runtime.tabletop_robot import start_additional_feedback

    additional_feedback = start_additional_feedback(config, camera_only=args.camera_only)
    if config.get("robot", {}).get("enabled") and not args.camera_only:
        from lerobot_arm_control.runtime.tabletop_robot import RobotFeedback

        robot = RobotFeedback(config["robot"], hardware_control=args.hardware_control,
                              task='pick_t' if args.pick_t else 'push_t')
        robot.start()
    if (args.push_t or args.pick_t) and robot is None:
        raise ValueError('Robot policies require configured SO101 feedback')
    push = None
    send_lock = threading.Lock()

    def send(message):
        with send_lock:
            print(json.dumps(message, allow_nan=False), flush=True)
    latest, failure = [], []

    def capture():
        count = 0
        while not stopped.is_set():
            ok, frame = camera.read()
            if not ok:
                failure.append("Remote camera stopped returning frames")
                stopped.set()
                break
            count += 1
            with lock:
                latest[:] = [frame, count, time.time_ns(), time.monotonic()]

    def receive():
        try:
            for line in sys.stdin:
                command = None
                try:
                    if len(line) > 65536:
                        raise ValueError('command too large')
                    command = json.loads(line)
                    if not isinstance(command, dict):
                        raise ValueError('command must be an object')
                    if command.get('op') == 'stop':
                        break
                    if not push:
                        raise ValueError('local plan receiver was not enabled')
                    result = push.handle(command)
                    send({'type': 'ack', 'version': 1, 'id': command.get('id'),
                          'ok': True, 'result': result})
                except (ValueError, KeyError, TypeError, RuntimeError) as exc:
                    send({'type': 'ack', 'version': 1,
                          'id': command.get('id') if isinstance(command, dict) else None,
                          'ok': False, 'error': str(exc)})
        finally:
            stopped.set()
            if push:
                push.close()

    worker = threading.Thread(target=capture, daemon=True)
    worker.start()
    tag = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["dictionary"])),
        config["marker_id"], 256,
    )
    encoded = cv2.imencode(".png", tag)[1]
    if args.push_t or args.pick_t:
        from lerobot_arm_control.tasks.push_t.push_t_receiver import PushReceiver

        directory = ROOT/'data/push_t'/('remote-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        push = PushReceiver(robot, directory, task='pick_t' if args.pick_t else 'push_t')
    send({"type": "hello", "version": 1, "config": config,
                      "control_config_sha256": CONTROL_CONFIG_SHA256,
                      "device": device, "perception_host": os.uname().nodename,
                      'camera_controls': camera_controls,
                      "robot": robot.metadata_snapshot() if robot else None,
                      'robot_feedback': {name: feedback.metadata_snapshot()
                                         for name, feedback in additional_feedback.items()},
                      'push_t': push.status() if push else None,
                      "tag_png": base64.b64encode(encoded).decode("ascii"),
          "planning_location": "local", "execution_protocol": 1})
    threading.Thread(target=receive, daemon=True).start()
    last_count, sequence = 0, 0
    display = DisplayStream(args.stream_fps, stopped,
                            write=lambda value: write_display(value, send_lock))
    try:
        while not stopped.is_set():
            loop = time.monotonic()
            with lock:
                packet = list(latest)
            if not packet or packet[1] == last_count:
                stopped.wait(.005)
                continue
            frame, last_count, captured_ns, captured_at = packet
            if loop - captured_at > .5:
                raise RuntimeError("Camera capture is stale")
            sequence += 1
            if list(frame.shape[1::-1]) != capture_size:
                raise ValueError("Camera did not provide the configured capture resolution")
            native_frame = None
            mapping = config.get("low_to_capture_affine")
            if capture_size != config["image_size"]:
                if mapping is None:
                    raise ValueError("High-resolution capture needs a measured intrinsic mapping")
                native_frame = frame
                frame = cv2.warpAffine(frame, cv2.invertAffineTransform(np.array(mapping)),
                                       tuple(config["image_size"]), flags=cv2.INTER_AREA)
            state = perception.process(frame, sequence, captured_ns, native_frame, mapping,
                                       robot.snapshot() if robot else None,
                                       marker_only=args.marker_only)
            if not args.marker_only and time.monotonic()-last_map_save > 1:
                perception.save_static_map(static_map, state)
                last_map_save = time.monotonic()
            if native_frame is not None and not args.omit_native_images:
                state["native_jpeg"] = base64.b64encode(cv2.imencode(
                    ".jpg", native_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])[1]).decode("ascii")
            if robot:
                state["robot"] = robot.snapshot()
                state["robot"]["registration"] = state.pop("robot_registration")
            if robot:
                registration = state["robot"]["registration"]
                state["robot"] = robot.snapshot()
                state["robot"]["registration"] = registration
                state["motor_samples"] = robot.drain_samples()
            state['robot_feedback'] = {name: feedback.snapshot()
                                       for name, feedback in additional_feedback.items()}
            state["capture_sequence"] = last_count
            state["capture_age_at_send_ms"] = (time.monotonic() - captured_at) * 1000
            if push:
                push.observe(state)
                state['push_t'] = push.status()
            state['stream_queued_unix_ns'] = time.time_ns()
            display.offer(state)
            stopped.wait(max(0, 1 / config["fps"] - (time.monotonic() - loop)))
        if failure:
            raise RuntimeError(failure[0])
    except BrokenPipeError:
        pass
    finally:
        stopped.set()
        worker.join(timeout=2)
        camera.release()
        if push:
            push.close()
        if robot:
            robot.close()
        for feedback in additional_feedback.values():
            feedback.close()


if __name__ == "__main__":
    main()
