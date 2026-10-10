"""Local-only circle generation, collision checking and dynamics verification."""

import argparse
import hashlib
import json
import math
from pathlib import Path

from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.tasks.circle.circle_trajectory import PLAN, ROOT, Circle, load_profile
from lerobot_arm_control.transport.realtime import JOINTS


def check_path(model, points, *, allow_start_tolerance=False, check_joint_limits=True):
    import mujoco
    import numpy as np

    data = mujoco.MjData(model)
    for index, q in enumerate(points):
        margin = -math.radians(2) if allow_start_tolerance else math.radians(2)
        low, high = model.jnt_range[:, 0] + margin, model.jnt_range[:, 1] - margin
        if check_joint_limits and (np.any(np.asarray(q) < low) or np.any(np.asarray(q) > high)):
            raise ValueError(f"trajectory joint limit at sample {index}")
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        if any(c.dist < -0.001 for c in data.contact):
            raise ValueError(f"trajectory collision at sample {index}")


def entry_points(initial, target):
    return [[a + (b - a) * smooth_fraction(i / 200) for a, b in zip(initial, target)]
            for i in range(201)]


def validate_hardware_entry(circle, initial):
    """Return the local collision check result bound to the exact entry and shared files."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(resource_path('assets/so101/so101.xml')))
    check_path(model, circle.points, check_joint_limits=False)
    check_path(model, entry_points(initial, circle.points[0]), check_joint_limits=False)
    return {"checked": True, "initial_qpos": list(initial),
            "model_sha256": circle.model_hash, "plan_sha256": circle.plan_hash,
            "control_config_sha256": circle.profile_hash}


def generate(path):
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(resource_path('assets/so101/scene_mujoco.xml')))
    data = mujoco.MjData(model)
    q = np.array([0, -0.5, 0.8, -0.3, 0, 0.3])
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    site = model.site("gripperframe").id
    center = data.site_xpos[site].copy()
    radius = 0.015
    points = []
    errors = []
    for angle in np.linspace(0, 2 * np.pi, 721):
        target = center + radius * np.array([np.cos(angle), 0, np.sin(angle)])
        for _ in range(100):
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            error = target - data.site_xpos[site]
            if np.linalg.norm(error) < 1e-7:
                break
            jac = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jac, None, site)
            j = jac[:, :3]
            dq = j.T @ np.linalg.solve(j @ j.T + np.eye(3) * 1e-8, error)
            q[:3] += np.clip(dq, -0.05, 0.05)
        else:
            raise ValueError("circle IK did not converge")
        points.append(q.tolist())
        errors.append(float(np.linalg.norm(error)))
    if max(abs(a - b) for a, b in zip(points[0], points[-1])) > 1e-5:
        raise ValueError("IK branch does not close")
    points[-1] = list(points[0])
    check_path(model, points)
    period = load_profile()[0]["period_seconds"]
    saved = {"version": 1, "joints": list(JOINTS),
             "model_sha256": hashlib.sha256((resource_path('assets/so101/so101.xml')).read_bytes())
             .hexdigest(), "site": "gripperframe", "plane": "XZ", "radius_m": radius,
             "center_m": center.tolist(), "period_seconds": period, "qpos": points,
             "ik_max_error_m": max(errors)}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(saved, indent=2) + "\n")
    return model, saved


def verify(model, path, report_path, *, simulation=False):
    import mujoco
    import numpy as np

    circle = Circle(path, simulation=simulation)
    data = mujoco.MjData(model)
    data.qpos[:] = circle.points[0]
    data.ctrl[:] = data.qpos
    circle.start(data.qpos, 0)
    errors = []
    reference = mujoco.MjData(model)
    tracking = []
    joint_tracking = []
    speed = []
    penetrating = 0
    duration = circle.entry_seconds + circle.ramp_seconds + 2 * circle.period
    steps = round(duration / model.opt.timestep)
    next_control = 0.0
    for _ in range(steps):
        if data.time + 1e-10 >= next_control:
            data.ctrl[:] = circle.sample(data.time)
            next_control += 1 / circle.control_hz
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise ValueError("simulation diverged")
        if data.time > circle.entry_seconds + circle.ramp_seconds:
            mujoco.mj_forward(model, data)
            reference.qpos[:] = circle.sample(data.time)
            mujoco.mj_forward(model, reference)
            position = data.site("gripperframe").xpos
            tracking.append(float(np.linalg.norm(position - reference.site("gripperframe").xpos)))
            joint_tracking.append(float(np.max(np.abs(data.qpos - reference.qpos))))
            radial = np.linalg.norm((position - circle.center)[[0, 2]])
            errors.append(float(np.hypot(radial - circle.radius,
                                         position[1] - circle.center[1])))
            speed.append(float(np.max(np.abs(data.qvel))))
            penetrating += sum(c.dist < -0.001 for c in data.contact)
    result = {"duration_sim_seconds": data.time, "steps": steps,
              "circle_radius_m": circle.radius, "period_seconds": circle.period,
              "max_circle_error_m": max(errors), "max_joint_speed_rad_s": max(speed),
              "max_cartesian_tracking_error_m": max(tracking),
              "max_joint_tracking_error_rad": max(joint_tracking),
              "penetrating_contacts": penetrating,
              "profile": "simulation" if simulation else "hardware",
              "passed": (max(errors) < 0.005 and max(tracking) < 0.005
                         and max(joint_tracking) < 0.01
                         and penetrating == 0)}
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(result, indent=2) + "\n")
    if not result["passed"]:
        raise ValueError(f"circle simulation failed: {result}")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PLAN)
    parser.add_argument("--simulation", action="store_true",
                        help="Verify the shared circle timing against simulated dynamics")
    args = parser.parse_args()
    model, _ = generate(args.output)
    report = "circle-simulation-fast.json" if args.simulation else "circle-simulation.json"
    print(json.dumps(verify(model, args.output, ROOT / "logs" / report,
                            simulation=args.simulation)))


if __name__ == "__main__":
    main()
