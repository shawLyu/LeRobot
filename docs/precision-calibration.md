# Dual-arm simulation and measured control precision

Updated on 2026-10-07. Both SO101 models run locally. The remote host captures
camera observations and executes the shared follower controller without MuJoCo.

## View both arms

```sh
./scripts/view_tabletop_realtime.sh --fps 5

# Camera and mounting poses only: does not open either motor bus.
# This mode can run alongside the isolated gripper benchmark below.
./scripts/view_tabletop_realtime.sh --camera-only --fps 5
```

The black follower uses live calibrated encoders. The white `arm1` has its own
six-joint CAD chain at its measured base, with a translucent, **unmeasured preview**
joint pose. Its joint calibration and control are not enabled. Both chains share
mesh assets, but have independent bodies, joints, sites and materials. Recorded
two-arm scenes contain 12 joints; reset their `recorded_pose` keyframe when loading
them independently. The viewer calls `mj_forward`; it is a kinematic mirror with
no hardware-driving actuators. Base separation is 289.127 mm. See
[Two-arm mounting calibration](two-arm-calibration.md).

## What the current markers measure

IDs 7 and 6 are rigidly attached to the bases. They locate the bases and cannot
independently measure a moving wrist or fingertip. Encoder-derived forward
kinematics is not an independent measurement of Cartesian error.

For optical measurements, rigidly mount a third marker on the follower wrist or
fixed gripper body. Do not mount it on the moving jaw. Keep it and the primary
table anchor visible, and record the measured black-border side and its rigid
transform to the chosen tool reference. The marker center can be measured without
that transform, but fingertip accuracy requires it.

[Printable ID 8 marker](../assets/calibration/tool-marker-id8-42mm.svg) uses
`DICT_4X4_50`, with a 42 mm black square. Print at 100%, disable fit-to-page, and
measure the printed square before using that size. Add the following under
`robot` in `configs/tabletop.json` **after installing and measuring the marker**:

```json
"tool_marker": {
  "enabled": true,
  "dictionary": "DICT_4X4_50",
  "id": 8,
  "marker_size_m": 0.042,
  "max_reprojection_error_px": 1.0
}
```

```sh
./scripts/deploy_code.sh --config
./scripts/view_tabletop_realtime.sh --camera-only --fps 5
```

`states.jsonl` publishes `tool_marker`: camera-measured marker-center position
and orientation in the table frame, capture timestamp, reprojection residual,
and separate `position_valid` / `orientation_valid` flags. Measurement uses native
image corners, calibrated intrinsics and table extrinsics; it does not use robot
angles or forward kinematics. Lost markers/table reference and ambiguous square
poses are flagged. Missing measurements are not replaced by predictions. Synthetic
projection tests verify coordinate conversion, not the physical camera accuracy.
Estimate stationary optical noise before evaluating motion, then compare repeated
approaches from both directions at several reaches using independent validation
poses. The camera's current capture rate is approximately 5 Hz; it cannot validate
every transient in the 60 Hz motor loop.

## Public reference and practical target

The [official STS3215-C001 datasheet](https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf)
specifies a 12-bit encoder (4096 counts, approximately 0.088 degrees per count)
and backlash up to 0.5 degrees. Encoder resolution is not whole-arm accuracy.
The [Robo9 STS3215 experiment](https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/)
reports about ±0.3 mm repeatability at a 100 mm lever, approximately 0.17 degrees
or two encoder counts. That experiment used a different 12 V motor variant and
fixture; it is a comparison target, not a guaranteed SO101 Cartesian limit.

The isolated benchmark uses a strict criterion: every encoder sample in every
final 0.8-second dwell window must be within two counts of its nominal reference.
Report startup, direction, load and settling time rather than retaining only the
best visits. Full-arm absolute accuracy and gearbox output backlash remain
unmeasured until an independent moving marker is installed and validated.

## Supervised isolated gripper benchmark

Support the arm with all motors initially unpowered. Close ordinary motor viewers,
calibration tools and controllers. `--camera-only` can remain open. Each robot bus
is reserved before connection so cooperating workers cannot open it concurrently.

```sh
# Executes a finite, small gripper-only trial, using the shared V14 profile.
./scripts/check_gripper_precision.sh --execute --amplitude-ticks 16 --cycles 3

# Larger repeated trial, still only about 2.6 degrees of gripper rotation.
./scripts/check_gripper_precision.sh --execute --amplitude-ticks 32 --cycles 5

# Use an explicit nearby encoder center for comparable trials.
./scripts/check_gripper_precision.sh --execute --amplitude-ticks 32 --cycles 5 \
  --reference-raw 1583 --dwell 2
```

Defaults use the shared quintic entry curve. `--step` selects abrupt references for
comparison. A fixed center must be within one trial amplitude of the initial raw
position, and all targets require limit margin. Stale feedback, faults,
disconnects or movement of an uncommanded arm joint stop the trial. The worker
uses the shared watchdog and controller. Cleanup holds, releases only the gripper,
and restores temporary RAM motion settings. Check `completed`, `cleanup_error`,
and acknowledgement evidence in each generated `logs/gripper-precision-*/result.json`.
Full-rate remote traces are copied into that directory as `control.json`.

## Measurements from this installation

All completed trials below kept the other five arm motors unpowered, with no
observed encoder movement in those joints. Only the gripper was actuated. RMS and
maximum errors refer to the final 0.8 seconds of each 2-second dwell, including
initial visits. Repeated-visit range is the span of median readings at each target.

| Trial | Profile / reference | Center / open (raw counts) | Cycles | RMS / max error (counts) | Median range, center / open (counts) | All windows within 2 counts |
| --- | --- | --- | ---: | --- | --- | --- |
| Isolated small trial | V14 / step | 1583 / 1599 | 3 | 0.58 / 1 | 0 / 0 | Yes |
| Larger confirmation | V14 / step | 1583 / 1615 | 5 | 5.30 / 9 | 5 / 6 | No |
| Smooth trial | V14 / quintic | 1580 / 1612 | 5 | 1.26 / 2 | 2 / 0 | Yes |
| Matched-center smooth recheck | V14 / quintic | 1583 / 1615 | 5 | 1.94 / 5 | 5 / 4 | No |
| Software PI candidate | Temporary V15 / quintic | 1583 / 1615 | 5 | 3.16 / 5 | 7 / 2 | No |

The matched-center V14 recheck had up to five-count error in its first two visits;
later visits were within one count. The candidate changed only gripper software
feedback settings and did not improve the overall result. **The original V14
configuration was restored on both hosts.** Native PID and EEPROM/protection
registers were not changed. Native clockwise/counterclockwise dead zones already
read one count. This evidence does not support a universal two-count claim.

Trial directories are `logs/gripper-precision-isolated-20261007/`,
`logs/gripper-precision-confirmation-20261007/`,
`logs/gripper-precision-smooth-20261007/`,
`logs/gripper-precision-matched-smooth-20261007/`, and
`logs/gripper-precision-v15-candidate-20261007/`.
The earlier baseline attempt failed before its target cycle due to overlapping
motor access; its trace is retained, and exclusive pre-open ownership plus
acknowledged cleanup were added. It is not counted as a completed trial.

## Before whole-arm motion and calibration

The present resting elbow encoder maps to approximately -117.9 degrees, outside
the canonical CAD elbow limits of ±96.8 degrees. Resolve the physically measured
joint zero/direction and valid range before executing whole-arm trajectories.
Do not clip measured feedback or widen CAD limits to hide the discrepancy.
Previous height corrections were archived when the base installation changed;
they have not been reapplied. Whole-arm Reset, circle and physical Push-T have
not been revalidated for this mounting setup.

The next calibration needs the moving marker's actual side and rigid mounting
transform, a physically checked encoder-to-model mapping, and independent
validation poses. Only then can measured optical residuals justify a new shared
controller or kinematic correction. See [Joint angle calibration](angle-calibration.md)
and [Height calibration](height-calibration.md).
