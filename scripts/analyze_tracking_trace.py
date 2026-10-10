"""Analyze full-rate feedback without issuing any hardware commands."""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.transport.realtime import JOINTS

ROOT = Path(__file__).resolve().parents[1]


def analyze(path):
    frames = [f for f in json.loads(path.read_text())["frames"]
              if f["circle"] and f["circle"]["phase"] == "running"]
    if len(frames) < 2:
        raise ValueError("trace contains no completed running interval")
    saved_path = ROOT / "data/calibration/follower_mujoco.json"
    saved = json.loads(saved_path.read_text())
    calibration = AngleCalibration(saved_path, saved["fingerprint"])
    t = np.array([f["time_monotonic"] for f in frames])
    periods = np.diff(t)
    reference = np.array([[f["nominal_qpos"][j] for j in JOINTS] for f in frames])
    measured = np.array([calibration.angles(f["raw"], [0] * 6) for f in frames])
    error = np.rad2deg(reference - measured)
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/so101/scene_mujoco.xml"))
    actual, desired = mujoco.MjData(model), mujoco.MjData(model)
    endpoint_errors = []
    for a, b in zip(measured, reference):
        actual.qpos[:] = a
        desired.qpos[:] = b
        mujoco.mj_forward(model, actual)
        mujoco.mj_forward(model, desired)
        endpoint_errors.append(np.linalg.norm(actual.site("gripperframe").xpos
                                              - desired.site("gripperframe").xpos) * 1000)
    result = {"running_frames": len(frames),
              "period_p50_p95_p99_max_ms": (
                  np.quantile(periods, [.5, .95, .99, 1]) * 1000).tolist(),
              "intervals_over_25ms": int(sum(periods > .025)), "per_joint": {}}
    result.update(endpoint_rms_mm=float(np.sqrt(np.mean(np.square(endpoint_errors)))),
                  endpoint_max_mm=float(max(endpoint_errors)),
                  endpoint_source="encoder feedback plus calibrated model; not optical measurement")
    for i, joint in enumerate(JOINTS):
        unchanged = np.diff(measured[:, i]) == 0
        start = None
        episodes = []
        for k, stationary in enumerate(unchanged):
            if stationary and start is None:
                start = k
            if start is not None and (not stationary or k == len(unchanged) - 1):
                end = k if not stationary else k + 1
                travel = float(np.rad2deg(abs(np.diff(reference[start:end + 1, i])).sum()))
                # Exclude stationary references and the intended near-zero speed
                # at a reversal. This is encoder evidence, not an optical test.
                if travel >= .5:
                    episodes.append({"seconds": float(t[end] - t[start]),
                                     "reference_travel_degrees": travel})
                start = None
        result["per_joint"][joint] = {
            "rms_degrees": float(np.sqrt(np.mean(error[:, i]**2))),
            "max_degrees": float(abs(error[:, i]).max()),
            "longest_unchanged_encoder_with_moving_reference": max(
                episodes, key=lambda e: e["seconds"], default=None),
            "max_encoder_step_degrees": float(np.rad2deg(abs(np.diff(measured[:, i])).max()))}
    for field in ("read_ms", "step_ms", "processing_ms"):
        values = [f[field] for f in frames if field in f]
        if values:
            result[field + "_p50_p99_max"] = np.quantile(values, [.5, .99, 1]).tolist()
    result["slow_intervals"] = [
        {"elapsed_seconds": frames[i + 1]["circle"]["elapsed_seconds"],
         "period_ms": float(period * 1000),
         "previous_processing_ms": frames[i].get("processing_ms"),
         **{field: frames[i + 1].get(field) for field in ("read_ms", "step_ms", "processing_ms")}}
        for i, period in enumerate(periods) if period > .025]
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    result = analyze(args.trace)
    output = args.trace.with_name(args.trace.stem + "-analysis.json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result | {"analysis": str(output)}))
