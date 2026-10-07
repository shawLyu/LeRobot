"""Preview a remote V4L2 camera in an X11 window forwarded by ssh -X."""

import argparse
import json
import os
import threading
import time
import tkinter as tk
import tomllib
from pathlib import Path

import cv2
from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/camera.toml")
    parser.add_argument("--device", help="Override the configured V4L2 device")
    parser.add_argument("--width", type=int, help="Override the configured capture width")
    parser.add_argument("--height", type=int, help="Override the configured capture height")
    parser.add_argument("--snapshot", type=Path, help="Save one verification frame without a GUI")
    args = parser.parse_args()
    with args.config.open("rb") as stream:
        config = tomllib.load(stream)["camera"]
    device = args.device or config["device"]
    width = config["width"] if args.width is None else args.width
    height = config["height"] if args.height is None else args.height
    if width <= 0 or height <= 0:
        parser.error("width and height must be positive")
    if not os.access(device, os.R_OK | os.W_OK):
        parser.error(f"No camera read/write access: {device}; check the video group")
    if not args.snapshot and not os.environ.get("DISPLAY"):
        parser.error("DISPLAY is unset. Launch through ssh -X from a running local X11 server.")
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        parser.error(f"Could not open {device}; check permissions and whether another app uses it")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    info = {
        "device": device,
        "display": os.environ.get("DISPLAY"),
        "requested_width": width,
        "requested_height": height,
        "width": camera.get(cv2.CAP_PROP_FRAME_WIDTH),
        "height": camera.get(cv2.CAP_PROP_FRAME_HEIGHT),
        "capture_fps": camera.get(cv2.CAP_PROP_FPS),
    }
    print(json.dumps(info), flush=True)
    try:
        if args.snapshot:
            for _ in range(5):
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("Camera returned no frame")
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.snapshot), frame):
                raise RuntimeError(f"Could not save {args.snapshot}")
            print(f"Snapshot saved: {args.snapshot}", flush=True)
            return
        show_live(camera, config["display_fps"], device)
    finally:
        camera.release()


def show_live(camera, fps, device):
    root = tk.Tk()
    root.title(f"LeRobot Camera — {Path(device).name} — live via ssh -X")
    root.resizable(False, False)
    image_label = tk.Label(root)
    image_label.pack()
    status = tk.Label(root, text="Waiting for camera…", anchor="w")
    status.pack(fill="x")
    stopped = threading.Event()
    lock = threading.Lock()
    latest = [None, 0, 0.0]
    failure = []

    def capture():
        count = 0
        while not stopped.is_set():
            ok, frame = camera.read()
            if not ok:
                failure.append("Camera stopped returning frames")
                stopped.set()
                return
            count += 1
            with lock:
                latest[:] = [frame, count, time.monotonic()]

    worker = threading.Thread(target=capture, name="camera-capture", daemon=True)
    worker.start()
    displayed = 0
    started = time.monotonic()
    last_count = 0

    def update():
        nonlocal displayed, last_count
        if failure:
            print(failure[0], flush=True)
            root.destroy()
            return
        with lock:
            frame, count, captured_at = latest
        if frame is not None and count != last_count:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            photo = ImageTk.PhotoImage(Image.fromarray(rgb))
            image_label.configure(image=photo)
            image_label.image = photo
            displayed += 1
            last_count = count
            status.configure(
                text=(
                    f"Live · frame {count} · {frame.shape[1]}×{frame.shape[0]} · "
                    f"display {displayed / max(time.monotonic() - started, 0.001):.1f} fps · "
                    f"frame age {time.monotonic() - captured_at:.2f}s · Esc / close to exit"
                )
            )
            if displayed == 1:
                print("LIVE_WINDOW_READY", flush=True)
        root.after(max(1, round(1000 / fps)), update)

    root.bind("<Escape>", lambda _: root.destroy())
    root.after(0, update)
    try:
        root.mainloop()
    finally:
        stopped.set()
        worker.join(timeout=2)
    if failure:
        raise RuntimeError(failure[0])


if __name__ == "__main__":
    main()
