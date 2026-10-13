# SO-101 remote leader/follower control

White `arm1` is the passive acquisition handle/trigger; black `follower` carries the gripper.
Both independent six-joint calibrations are required. The measured white trigger range
39.08–140.54° maps to black gripper −11.31–109.25°; the 0/15/30° reference marks are calibration points. The upper endpoint was sampled at raw=3002
on 2026-10-13 while the user held the white trigger at maximum; 140.54° is the
existing encoder-calibration conversion, not a new independent mechanical angle measurement.

The minimum pair was captured with both arms stationary: white raw=1940 maps
to black raw=1479. They are endpoint positions, distinct from angle-calibration zero.
The black maximum is its calibrated encoder upper limit raw=2982 (109.25°),
not the former fixed 90° target. Physical following uses the verified physical
encoder range for the jaw; arm joints retain their existing CAD/encoder limit intersection.
The canonical CAD model is unchanged; its synthetic actuator range still ends at
100°. [Endpoint capture and calibration identities](measurements/trigger-range-2026-10-13.json).

## Start real following

From the local checkout:

```sh
./scripts/lerobot-arm demo follow
# Optional manual start, then type e and Enter in the controlling terminal:
./scripts/lerobot-arm demo follow --start-held
```

This opens an SSH terminal running the controller on the robot computer. No simulator
is started or used. Encoder acquisition, angle calibration, trigger mapping, startup
alignment, shared 60 Hz PID, motor writes and fault handling all run on that computer.
For operation directly on the robot computer, use:

```sh
./scripts/lerobot-arm serve follow --hardware-control --standalone
```

The owning terminal accepts **h + Enter** to hold, **e + Enter** to initialize/resume,
and **q + Enter** to hold and exit. Ctrl-C, terminal EOF and SSH session termination
also close the controller through its hold/cleanup path. To keep ownership on the
remote computer independently of a local SSH session, run that command in its own
remote terminal. An optional `--duration SECONDS` bounds a run.

Startup checks both calibration fingerprints and servo faults, confirms a fresh
current-position goal, enables the follower and performs bounded model-range recovery.
Normal boundary recovery is limited to 2°; a closed jaw may recover up to 5°, with a
0.5° interior margin. It then aligns at 0.6 rad/s. Once nominal targets are within
2° of the mapped leader, each cycle follows the latest input without an accumulating
startup speed limit. The white arm remains passive.

The remote acquisition loop checks its own freshness; it does not require simulator
heartbeats. Stale acquisition or a motor fault holds motion. Recovery requires an
explicit **e + Enter** after the fault clears. A confirmed white status error is retained
when the SDK raises while rereading it; the monitoring process remains available.
No automatic fault restart or protection-register changes are introduced.

## Optional read-only simulator

Start this in a second local terminal while the remote controller is running:

```sh
./scripts/lerobot-arm view follow
# Finite observation without a window:
./scripts/lerobot-arm view follow --headless --duration 10 --log logs/follow-observation.json
```

The simulator subscribes to the remote `logs/follow.sock` telemetry channel via a
read-only relay. It shows the saved tabletop, calibrated bases, white handle and black
encoder pose. It sends no motor targets, hold/resume commands or heartbeat. Closing,
minimizing, pausing or losing focus in this window does not stop remote following.
A stale display is labelled as such; it cannot establish the current physical state.
Use the owning remote terminal to control motion. Concurrent motor tools remain excluded
by the serial ownership lock; multiple telemetry viewers are allowed.

```sh
# Optional virtual response to targets from the already-running remote controller:
./scripts/lerobot-arm view follow --simulate
# Fully offline input and virtual follower, without SSH or hardware:
./scripts/lerobot-arm view follow --source synthetic --auto
```

`--simulate` affects only the display; an already-running real controller continues
operating. For an offline demonstration use `--source synthetic`. Synthetic keyboard
input uses 1–6 and Up/Down; E resumes and Space holds only that local virtual demo.

## Shared controller and deployment

Every normal motion demo uses `configs/control.json`; `configs/follow.json` holds
roles, mapping and initialization only. Independent PID/frequency overrides are rejected.
See [controller design](lerobot-control.md) and [current response evidence](measurements/remote-follow-2026-10-12/README.md).

```sh
./scripts/deploy_code.sh
```

Restart the controller after deployment. The viewer checks the remote control hash,
follow parameters; calibration identity is validated by the remote controller. Its connection is observational; a mismatch
rejects the viewer without stopping the controller. Hardware configuration and saved
calibrations are preserved by deployment.

## Smoothness-first default (2026-10-13)

All six axes now use the shared two-pole 25 ms reference shaper, zero predictive
velocity lead, and motion-weighted integral learning. Expect approximately 50 ms
of low-frequency reference delay plus the servo response. Smoothness takes
priority over the former <100 ms target; actual latency must still be measured.
The latency tool retains its strict <100 ms threshold and can report failure
for an otherwise usable smooth capture. See the [design](lerobot-control.md) and
[current evidence](measurements/follow-smoothing-2026-10-13/README.md).

## Record and measure

```sh
./scripts/lerobot-arm demo follow --start-held --latency-test
# Or a labelled subset; this cannot pass unmeasured axes:
./scripts/lerobot-arm demo follow --start-held --latency-test --latency-joints elbow_flex
```

Type **e + Enter** and follow the joint name/countdown in the controlling terminal.
Move only that joint gently back and forth for its 12-second stage. All sampling and
stage timing occur remotely. Holds pause stage progress; an interrupted capture is not
reported as a completed pass. On normal exit the launcher retrieves the remote trace,
writes `logs/follow-remote-trace.json`, `.metrics.json` and `.png`, and exits with code 2
if acceptance fails. Use `--log` to choose a different local destination.

`--control-log /absolute/remote/path.json` records a normal run without guided motion.
The finite buffer retains up to 36,000 frames (about ten minutes at 60 Hz).
The [latency measurement definition](measurements/follow-latency-2026-10-12/README.md#measurement-definition)
requires reversals and sufficient movement; read-to-command timing alone is not motor
response. See the [latest retuning report](measurements/follow-retune-2026-10-13/README.md).
The current profile does not yet have an uninterrupted six-axis <100 ms pass.

To separate the agreed normal operation envelope from overspeed stress tests:

```sh
./scripts/lerobot-arm verify follow-latency logs/follow-remote-trace.json \
  --normal-speed-limits 2 2 2 2 2 1.2 \
  --output logs/follow-scoped.json --plot logs/follow-scoped.png
```

Limits are peak **mapped leader** radians/second in motor order, including 0.5 s
of preceding motion. The gripper limit applies after the 118° → 90° mapping.
Every candidate window is retained; overspeed failures stay in the report. At least
two valid normal windows per requested joint are required. Any recorded leader
alarm or unfinished guided capture rejects the overall capture. These limits
classify measurements; they do not add an input rate limiter to the controller.

## Online warnings and recovery

A confirmed leader motor alarm displays `WARNING`, holds the follower, and keeps
the process, readings and read-only observer online. It records the original
driver error and post-hold voltage, temperature and protection thresholds in
`metadata.leader_fault_events`. When current readings become healthy, the display
says the alarm cleared; **e + Enter** in the owning terminal explicitly resumes.
An active alarm still rejects resume. There is no automatic torque release or
voltage-threshold change. `--duration` ends a finite test even if a warning is
being displayed; ordinary `demo follow` has no such time limit.

The observed white-gripper voltage limits were already 4–12 V. A valid 5.3 V
readback alongside a device voltage-error packet does not establish that the
transient fault is harmless; widening that range is not part of this update.

## Upstream model and mapping

Joint-space leader input follows upstream LeRobot's
[teleoperator](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/so_leader/so_leader.py)
and [SO follower](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py)
interfaces, with this setup's measured encoder-to-angle mappings. White display geometry
comes from the official SO-101 acquisition handle; the calibration-bound black model
and both measured angle mappings are preserved.
