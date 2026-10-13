# Shared circle trajectory

The absolute joint path is stored in `data/trajectories/circle.json`: a 15 mm Cartesian circle for the SO101 model.
Path points and model identity come from the accepted trajectory input. The historical period in the plan remains provenance.
With the smoothness-first reference shaper (2026-10-13), normal playback uses `trajectory.period_seconds=16` from `configs/control.json`.

```sh
./scripts/lerobot-arm view arm --backend mujoco --circle --circle-laps 3
./scripts/lerobot-arm view arm --backend hardware --hardware-control --circle --circle-laps 3
```

The hardware entry performs Reset first, approaches the original absolute path with quintic interpolation,
then executes the requested laps and holds. Simulation and hardware share approach speed 0.8 rad/s,
minimum approach duration 0.35 s, speed ramp 1 s, and 60 Hz updates. Reset retains 0.5 rad/s.
Period and approach speed come only from shared configuration; launch arguments no longer override them separately.

Locally, the measured remote pose is used to check collisions along the circle and approach segment before sending the result.
The remote host checks calibrated endpoints, model/trajectory/control hashes, and entry pose.
A pose change over 0.003 rad requires a new check. Remote playback does not depend on MuJoCo.
Tracking error and action completion are assessed separately; retired error thresholds do not stop execution.
Motor faults, lost torque, connection loss, or heartbeat loss still trigger hold.
Read-only model/unit checks do not cause motion; simulation objects cannot enable hardware control.

See [Control performance](control-performance.md) for the latest physical experiment errors and camera records.
Physical playback requires separate validation. See [Code layout](code-layout.md) for shared parameters and deployment.
