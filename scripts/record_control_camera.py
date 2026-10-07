"""Record timestamped camera evidence independently of the motor worker."""

import argparse
import json
import time
import tomllib
from pathlib import Path

import cv2


def record(output, seconds):
    cv2.setNumThreads(1)
    config_path = Path(__file__).resolve().parents[1] / "configs/camera.toml"
    config = tomllib.loads(config_path.read_text())
    camera = cv2.VideoCapture(config["camera"]["device"], cv2.CAP_V4L2)
    writer = None
    timestamps = []
    try:
        if not camera.isOpened():
            raise RuntimeError("could not open camera")
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        output.parent.mkdir(parents=True, exist_ok=True)
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            ok, frame = camera.read()
            captured = time.monotonic()
            if not ok:
                raise RuntimeError("camera stopped returning frames")
            if writer is None:
                fps = 15
                writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"),
                                         fps, (frame.shape[1], frame.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError("video encoder unavailable")
                print(json.dumps({"ready": True, "fps": fps, "output": str(output)}), flush=True)
            writer.write(frame)
            timestamps.append({"frame": len(timestamps), "time_monotonic": captured,
                               "time_unix_ns": time.time_ns()})
    finally:
        camera.release()
        if writer:
            writer.release()
        output.with_suffix(".timestamps.json").write_text(json.dumps({
            "frames": timestamps, "capture_timestamps_are_after_read": True}) + "\n")
    print(json.dumps({"frames": len(timestamps), "seconds": time.monotonic() - started}),
          flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=60)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 180:
        parser.error("seconds must be in 1..180")
    record(args.output, args.seconds)
