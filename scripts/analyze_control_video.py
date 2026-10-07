"""Track gripper image motion and export playback using capture timestamps.

Pixel motion is not a calibrated Cartesian accuracy measurement. The camera's
actual sampling rate limits which vibrations can be resolved.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def analyze(video, trace, roi):
    cv2.setNumThreads(1)
    frames = json.loads(video.with_suffix(".timestamps.json").read_text())["frames"]
    timestamps = np.array([f["time_monotonic"] for f in frames])
    trace_frames = json.loads(trace.read_text())["frames"]
    control = [f for f in trace_frames
               if f["circle"] and f["circle"]["phase"] == "running"]
    if not control:
        raise ValueError("no running circle")
    start, end = control[0]["time_monotonic"], control[-1]["time_monotonic"]
    if timestamps[0] > start or timestamps[-1] < end:
        raise ValueError("video does not cover the complete circle")
    first = int(np.searchsorted(timestamps, start))
    last = int(np.searchsorted(timestamps, end))
    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_POS_FRAMES, first)
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError("video could not be read")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(gray)
    x, y, w, h = roi
    mask[y:y + h, x:x + w] = 255
    mask[gray > 105] = 0
    points = cv2.goodFeaturesToTrack(gray, 40, .01, 3, mask=mask)
    if points is None or len(points) < 4:
        raise RuntimeError("insufficient gripper features; inspect the first frame and ROI")
    origin = points.copy()
    for p in points[:, 0]:
        cv2.circle(frame, tuple(np.round(p).astype(int)), 3, (0, 255, 0), 1)
    cv2.imwrite(str(video.with_suffix(".tracking-start.jpg")), frame)
    motion = [{"time_monotonic": float(timestamps[first]), "offset_px": [0., 0.],
               "features": len(points)}]
    for index in range(first + 1, last + 1):
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError("video ended before the control trace")
        current = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(gray, current, points, None)
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(current, gray, tracked, None)
        good = ((status[:, 0] == 1) & (back_status[:, 0] == 1)
                & (np.linalg.norm(backward[:, 0] - points[:, 0], axis=1) < 1))
        if sum(good) < 4:
            raise RuntimeError("gripper tracking lost; no smoothness verdict")
        origin = origin[good]
        points = tracked[good]
        offset = np.median(points[:, 0] - origin[:, 0], axis=0)
        motion.append({"time_monotonic": float(timestamps[index]),
                       "offset_px": offset.tolist(), "features": len(points)})
        gray = current
    capture.release()
    t = np.array([m["time_monotonic"] for m in motion])
    xy = np.array([m["offset_px"] for m in motion])
    # Compare runs at a common coarse cadence; capture rates can differ with
    # exposure. This cannot resolve vibrations faster than the camera sampling.
    grid = np.arange(t[0], t[-1], .15)
    xy = np.column_stack([np.interp(grid, t, xy[:, i]) for i in range(2)])
    t = grid
    speed = np.linalg.norm(np.diff(xy, axis=0), axis=1) / np.diff(t)
    steady = (t[1:] > start + 1.2) & (t[1:] < end - 1.2)
    pauses = (speed < 1) & steady
    begun = None
    lengths = []
    for i, pause in enumerate(pauses):
        if pause and begun is None:
            begun = i
        if begun is not None and (not pause or i == len(pauses) - 1):
            finish = i if not pause else i + 1
            lengths.append(float(t[finish] - t[begun]))
            begun = None
    result = {"actual_capture_fps": float(1 / np.median(np.diff(timestamps))),
              "camera_max_interval_seconds": float(np.diff(timestamps).max()),
              "running_duration_seconds": end - start,
              "period_seconds": control[0]["circle"]["period_seconds"],
              "roi_xywh": roi, "remaining_features": len(points),
              "analysis_cadence_seconds": .15,
              "steady_speed_px_s_p50_p95": np.quantile(speed[steady], [.5, .95]).tolist(),
              "steady_pause_fraction_below_1px_s": float(pauses.sum() / steady.sum()),
              "longest_image_pause_seconds": max(lengths, default=0),
              "source": "gripper image features; not calibrated millimeters or high-rate vibration",
              "motion": motion}
    video.with_suffix(".tracking.json").write_text(json.dumps(result, indent=2) + "\n")

    # Uniform playback timeline; duplicate captured frames across real gaps.
    capture = cv2.VideoCapture(str(video))
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError("cannot resample video")
    output = video.with_name(video.stem + "-realtime.mp4")
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"),
                             15, (frame.shape[1], frame.shape[0]))
    if not writer.isOpened():
        raise RuntimeError("playback encoder unavailable")
    reset_frames = [f for f in trace_frames if f["reset_phase"] in ("moving", "settling")]
    review_start = reset_frames[0]["time_monotonic"] if reset_frames else start
    review_end = min(end + 1, timestamps[-1])
    review = video.with_name(video.stem + "-review.mp4")
    review_writer = cv2.VideoWriter(str(review), cv2.VideoWriter_fourcc(*"mp4v"),
                                    15, (frame.shape[1], frame.shape[0]))
    if not review_writer.isOpened():
        raise RuntimeError("review encoder unavailable")
    trace_times = np.array([f["time_monotonic"] for f in trace_frames])
    thumbnails = []
    next_thumbnail = review_start
    index = 0
    for stamp in np.arange(timestamps[0], timestamps[-1], 1 / 15):
        while index + 1 < len(timestamps) and timestamps[index + 1] <= stamp:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("missing captured frame")
            index += 1
        writer.write(frame)
        if review_start <= stamp <= review_end:
            trace_index = max(0, int(np.searchsorted(trace_times, stamp, side="right")) - 1)
            state = trace_frames[trace_index]
            phase = ("Reset: " + state["reset_phase"]
                     if state["reset_phase"] in ("moving", "settling")
                     else "Circle: " + state["circle"]["phase"])
            marked = frame.copy()
            cv2.rectangle(marked, (0, 0), (640, 32), (20, 20, 20), -1)
            cv2.putText(marked, f"{stamp - review_start:.1f}s | {phase} | real-time playback",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            review_writer.write(marked)
            if stamp >= next_thumbnail and len(thumbnails) < 16:
                thumbnails.append(cv2.resize(marked, (320, 240)))
                next_thumbnail += (review_end - review_start) / 16
    capture.release()
    writer.release()
    review_writer.release()
    if len(thumbnails) == 16:
        sheet = np.concatenate([np.concatenate(thumbnails[i:i + 4], axis=1)
                                for i in range(0, 16, 4)], axis=0)
        cv2.imwrite(str(video.with_suffix(".review.jpg")), sheet)
    return {k: v for k, v in result.items() if k != "motion"} | {
        "playback": str(output), "review": str(review)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--roi", type=int, nargs=4, default=[450, 90, 50, 55])
    args = parser.parse_args()
    print(json.dumps(analyze(args.video, args.trace, args.roi)))
