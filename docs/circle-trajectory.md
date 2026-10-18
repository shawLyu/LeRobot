# Shared circle controller

The six-joint circle, manual control, following and manipulation tasks use the same
`PositionFeedback` implementation. Parameters live in `configs/control.json`.
The default `position_feedback` retains the validated contact/following settings.
`circle_position_feedback` contains overrides for the explicitly selected
`smooth-circle-v1` profile: bounded PI plus velocity feedforward, reference shaping,
and reduced native acceleration. High-speed free-space tuning is not silently
applied to grasping or pushing.

## Horizontal 8 cm circle at 60 degrees/second

The reviewed scene is `data/trajectories/table-circle-8cm.json`: an XY circle with
radius 0.08 m and center `[0.25, -0.017, 0.34] m` in the follower base frame.
This is circle phase speed: **60 degrees/second, a six-second steady lap**, and
8.38 cm/s TCP linear speed. Individual joint velocities are different.

With the matching calibration, empty gripper, inspected clear workspace and
operator present:

```sh
./scripts/lerobot-arm verify tracking --execute --from-current --laps 3 \
  --plan data/trajectories/table-circle-8cm.json
```

The current plan approaches at no more than 0.1 rad/s, then uses a six-second
quintic speed ramp at each end. Three complete circles take about **24 seconds**
after approach: 18 seconds of equivalent full-speed travel plus six seconds of
ramp overhead. The requested 60 degrees/second applies to the middle portion;
acceleration and braking deliberately run slower. Native acceleration is 16
while this profile runs; native PID, origins, EEPROM and protection are unchanged.
The original native acceleration and goal velocity are restored on close, with
torque retained for support. Ordinary completion holds at measured position.

The [physical report](measurements/table-circle60deg-s-2026-10-18/README.md)
records the smoothness/lag tradeoff, including the remaining speed ripple and
Cartesian tracking errors. This scene requires its own measured robot reference;
it is not a portable absolute motion command for a differently assembled robot.

## 90 degrees/second at different heights

The experimental plans `data/trajectories/table-circle90-h34cm.json`,
`table-circle90-h36cm.json` and `table-circle90-h38cm.json` use TCP heights of
34/36/38 cm above the table, the same XY center/radius and fixed tool inclination.
The `smooth-circle90-v1` profile inherits the circle gains but uses native
acceleration 32 instead of 16: the 90-degree path requires higher joint acceleration.
The original 60-degree plan and its acceleration remain unchanged.

```sh
./scripts/lerobot-arm verify tracking --execute --from-current --laps 3 \
  --plan data/trajectories/table-circle90-h34cm.json
```

Use the corresponding `h36cm` or `h38cm` plan after checking that height's workspace.
The steady lap takes four seconds. Each end retains its six-second speed ramp,
so three circles take about 18 seconds after approach. Finite jobs must be long
enough that the ramps do not overlap: this six-second ramp requires at least two
laps; three laps provide a useful steady segment for scoring. A plan without an
explicit speed retains the original one-period ramp limit.

These are measured experiments, not precision guarantees. See the
[multi-height 90-degree report](measurements/table-circle90-height-2026-10-18/README.md)
for completed trials, remaining errors and the rejected low-acceleration baseline.

## Remote execution and saved responses

`--from-current` reads a fresh remote pose without enabling motion. The local
computer checks the entire approach, circle geometry, table/self collisions and
sampled speed/acceleration, then uploads a proof bound to the exact model,
calibration, plan, configuration and initial pose. The remote computer checks the
proof, rechecks native limits and timing, and runs **one finite job of 1–5 laps**.
It generates references, reads encoders and writes motors on its own 60 Hz clock;
MuJoCo and the simulator are not in the physical control loop.

SSH output runs separately from the control loop. Observer backpressure or input
EOF does not stall the accepted finite job or its native feedback watchdog. It
can finish its bounded plan and hold without incoming client pings. A process
termination signal (including SSH SIGHUP) instead triggers hold and cleanup.
Local display interruption therefore does not guarantee an immediate stop. The connected client can send `hold`; no additional motion target
or reset is accepted during this standalone job. Native feedback gaps over the
watchdog limit, faults, torque loss, motion stalls and the finite execution deadline
still stop motion. The interactive/manual worker retains its client heartbeat.

After the hardware process closes, the local runner copies its full-rate trace
and writes:

- `logs/tracking-trial-<stamp>.json`: completion, observed states and report paths.
- `logs/tracking-trace-<stamp>.json`: every recorded control frame, actual RAM
  goals, encoder feedback, controller profile and restored native parameters.
- `logs/circle-response-<stamp>.json/.png`: measured phase speed, TCP errors,
  vibration residual and trajectory/speed/error plots.
- `logs/circle-response-<stamp>-joints.png`: six target/encoder curves.

Plotting and scoring happen after motion, locally. TCP values use encoders and
the calibrated CAD model; they are not independent optical measurements.

## Plan parameters and checks

An optional hashed `execution_profile` can lower `entry_speed_rad_s`, increase
`ramp_seconds`, explicitly select `controller_profile: "smooth-circle-v1"` or `"smooth-circle90-v1"`, and
request `angular_speed_deg_s`. An explicit speed requires an XY plan whose
`period_seconds` equals `360/angular_speed_deg_s`. The shared experimental maximum
is 90 degrees/second; the default reviewed 8 cm plan remains at 60 degrees/second. Planned joint speed and sampled entry/ramp acceleration
must also fit the budgets in `configs/control.json`; these checks run on both
hosts before enable. This budget is not a guarantee of physical tracking accuracy.

An XY plan declares `plane: "XY"`, `table_z_m` in base coordinates and the exact
`joint_calibration_sha256`. Wrist roll and gripper remain constant during approach
and playback. Local validation checks the first five CAD joint ranges, self and
table collisions. Native gripper limits remain authoritative when its measured
angle exceeds the generic CAD hinge range. `execution_ready: false` prevents a
candidate from starting. Any model, calibration, plan or control hash change
requires a new check; entry drift over 0.003 rad also invalidates the proof.

`Link(..., hardware_control=True, circle=True, circle_plan=workspace_plan_path)`
only loads a plan. Its interactive `circle_start` command requires local collision
validation. The diagnostic runner additionally supplies `circle_validation` and
`circle_laps` to launch the remotely owned finite job.

## Original small circle

`data/trajectories/circle.json` retains the 15 mm XZ circle and 16-second default
lap. Plans without an explicit angular speed keep the slower of their saved period
and the shared default. Generic simulation and interactive hardware entries:

```sh
./scripts/lerobot-arm view arm --backend mujoco --circle --circle-laps 3
./scripts/lerobot-arm view arm --backend hardware --hardware-control --circle --circle-laps 3
```

The interactive entry uses Reset first; the diagnostic `--from-current` path above
does not. The stored follower Reset reference predates the current calibration
and must be retaught before use. Do not bypass its identity rejection.

`migrate_sts3215_reference` is an explicit software-only reference conversion using
`Present_Position = Actual_Position - Homing_Offset`. It requires confirmed unchanged
assembly and rejects device, phase, mode or model changes and encoder wrap. It
preserves measured scale/third-point error, retains the old fingerprint and writes
no motor registers. Command envelopes and plans must be rebound to the new hash;
old rest poses and kinematic corrections remain invalid until separately reviewed.
