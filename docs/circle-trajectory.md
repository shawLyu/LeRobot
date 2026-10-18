# Shared circle trajectory

The absolute joint path is stored in `data/trajectories/circle.json`: a 15 mm Cartesian circle for the SO101 model.
Path points and model identity come from the accepted trajectory input. The default
lap takes 16 seconds. A reviewed plan with a longer `period_seconds` keeps that
slower timing instead of being accelerated to the shared default.

```sh
./scripts/lerobot-arm view arm --backend mujoco --circle --circle-laps 3
./scripts/lerobot-arm view arm --backend hardware --hardware-control --circle --circle-laps 3
```

The hardware entry performs Reset first, approaches the original absolute path with quintic interpolation,
then executes the requested laps and holds. Simulation and hardware share approach speed 0.8 rad/s,
minimum approach duration 0.35 s, speed ramp 1 s, and 60 Hz updates. Reset retains 0.5 rad/s.
An optional `execution_profile` in the hashed plan can reduce `entry_speed_rad_s`
and increase `ramp_seconds`. It cannot accelerate the shared profile.

Locally, the measured remote pose is used to check collisions along the circle and approach segment before sending the result.
The remote host checks calibrated endpoints, model/trajectory/control hashes, and entry pose.
A pose change over 0.003 rad requires a new check. Remote playback does not depend on MuJoCo.
Tracking error and action completion are assessed separately; retired error thresholds do not stop execution.
Motor faults, lost torque, connection loss, or heartbeat loss still trigger hold.
Read-only model/unit checks do not cause motion; simulation objects cannot enable hardware control.

See [Control performance](control-performance.md) for the latest physical experiment errors and camera records.
Physical playback requires separate validation. See [Code layout](code-layout.md) for shared parameters and deployment.

## Reviewed horizontal plans

The programmatic hardware connection accepts `Link(..., hardware_control=True,
circle=True, circle_plan=workspace_plan_path)`. The same workspace-relative plan
must be deployed to the remote host. This loads the plan without starting motion;
an explicit `circle_start` command is still required. This direct path approaches
the measured current pose without invoking Reset.

The diagnostic runner uses the same path and saves all observed states:

```sh
PYTHONPATH=src python -m lerobot_arm_control.diagnostics.circle_tracking \
  --execute --from-current --laps 1 --plan data/trajectories/table-circle-8cm.json
```

This command requires the matching reviewed plan and calibration on both hosts,
an empty gripper and an observed, cleared scene. It does not generate a new plan
or waive any identity, collision, native limit or fault check.

An XY plan declares `plane: "XY"`, `table_z_m` in base coordinates and the exact
`joint_calibration_sha256`. Wrist roll and gripper remain constant, including
during approach. Local validation checks the Cartesian radius and plane, the
first five CAD joint ranges, self collisions and table collisions. Gripper native
limits remain authoritative when its measured angle exceeds the generic CAD
hinge range. All six calibrated command limits are checked remotely before enable.
`execution_ready: false` prevents a candidate from starting on hardware.

`migrate_sts3215_reference` is an explicit software-only reference conversion,
using the installed LeRobot Feetech identity
`Present_Position = Actual_Position - Homing_Offset`. It requires confirmation of
unchanged assembly and rejects device, phase, mode or model changes and encoder
wrap. It preserves the measured scale and independent third-point error, records
the old fingerprint and writes no motor registers. The ordinary calibration loader
still rejects any fingerprint mismatch. Reviewed command envelopes and plans must
be rebound to the resulting file hash; old rest poses and kinematic corrections
remain invalid until separately reviewed or retaught.
