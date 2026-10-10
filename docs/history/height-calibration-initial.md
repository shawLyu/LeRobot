# Historical record: before the 2026-10-07 cleanup

This document preserves capture stages and measurements at the time.
Use [README](../../README.md) and [Current height workflow](../height-calibration.md) for operation.

## Applied native pitch scale correction

Following the user's request to apply the correction directly, fixed_jaw_height_native_pitch_scale was enabled.
The installed LeRobot STS3215 driver was checked at 4096 ticks/revolution.
Shoulder lift, elbow, and wrist pitch use its native scale; the other three axes retain their mapping.
Only seven known 0 mm table-contact poses were used: original samples 4 and 9 for independent validation,
with the other five for fitting. Three unsupported ruler readings remain recorded but were excluded from this fit by measurement method.
The previous rejected candidate was backed up.

Zero offsets were -2.1239, +3.8400, and +1.2085 degrees.
Fit RMSE was 0.562 mm; independent validation RMSE 0.881 mm; maximum absolute error 1.137 mm.
Raw manual-angle file bytes were unchanged. The correction was saved on both hosts as follower_mujoco_kinematic.json and loads after restart.
This validates those table-contact measurements, not a full physical Push-T rollout or absolute accuracy at other heights.
The mode still fits three zeros with scale fixed by hardware constants, requiring at least five fitting and two independent validation poses.
Numerical/observability thresholds were retained. Recompute with:

```sh
./scripts/lerobot-arm calibrate height fit --native-pitch-scale --apply
```

The following sections preserve the first capture instructions and first-round failure.
They do not mean more samples are currently required to enable the correction above.

## Initial measurements and capture

The user confirmed two poses where the fixed fingertip's lowest point touched the table.
Stable encoders predicted heights 5.73 mm and 9.19 mm; records were saved locally/remotely in data/calibration/height_samples.json.
At that stage at least eight further distinct poses were needed for the first automatic fit and independent validation.

Close Push-T/circle windows to release the exclusive serial port. Support the arm and release torque before manually moving it.
The sampler only reads registers; it neither releases/enables torque nor commands motion.
Do not press the gripper against the table and mislabel deformation as a zero-offset error.

Sampling runs remotely; fitting runs locally; accepted results are returned for remote validation/application.
The height command synchronizes raw calibration, base, and samples. --apply sends only locally accepted fits for another remote check.
Remote sampling has no MuJoCo dependency. Run locally:

```sh
./scripts/lerobot-arm calibrate height collect
```

After placing each pose, enter actual fixed-finger minimum height in millimeters and press Enter for about 1.2 s of read-only sampling.
Empty Enter means 0 mm; q saves and exits. Vary reach, wrist pitch, and wrist roll;
shoulder pan alone cannot identify these three corrections.
Touch the table or measured flat spacers, such as 10 or 20 mm, gently.
The reference is the fixed finger's lowest surface, not the moving finger, wrist origin, or opening midpoint.
The existing two samples counted toward the target; duplicates, motion, and faults were rejected.

Of ten distinct poses, eight fit and two validate, with every fifth sample reserved for validation.
Validation should cover different reach/orientation. Use collect --count 15 for more samples.
After capture, local fit saves a candidate and reports offsets, fit error, independent error, and rejection/acceptance reasons.

## Model and acceptance

Local MuJoCo forward kinematics and the fixed-finger CAD minimum surface minimize predicted/measured height residuals.
The fitter uses three-parameter Levenberg–Marquardt with Huber loss, fitting only shoulder_lift, elbow_flex, and wrist_flex zeros,
each within +/-5 degrees. In ordinary mode, encoder scale, base, link dimensions, and gripper mounting remain unchanged; scipy is unnecessary.

Candidates must pass observability, fit RMSE <=1.5 mm, validation RMSE <=2 mm, and maximum validation error <=3 mm.
If original validation error exceeds 1 mm, at least 25% improvement is also required.
Near-bound offsets or unidentifiable parameters are rejected. Thresholds are acceptance criteria, not guaranteed physical accuracy.
Continued failure requires investigating scales, gripper mounting, deformation, or adding full position/orientation measurements of a rigid tool marker.

## First-round failure and resampling

The first actual fit failed: fit RMSE 5.73 mm, validation RMSE 2.64 mm; no candidate was enabled.
The user confirmed samples 3/5/7 at 40/100/70 mm came from unsupported ruler readings.
These were selected for remeasurement against known-height supports while retaining other records.
Model residuals alone did not prove ruler readings wrong; new poses were allowed without exactly restoring old joint angles.

```sh
./scripts/lerobot-arm calibrate height resample --poses 3 5 7 --apply
```

Enter 0 for table contact or the measured spacer height. Previous complete samples remain under superseded.
Resampling triggers fit/independent validation; rejection retains original calibration.
If reliable supported measurements still fail, investigate scale/geometry rather than relaxing acceptance criteria.

Review an accepted candidate and apply:

```sh
./scripts/lerobot-arm calibrate height apply
```

Or capture, fit, validate, and apply in one command:

```sh
./scripts/lerobot-arm calibrate height collect --apply
```

--apply saves accepted corrections only and sends no physical motion.
Raw follower_mujoco.json remains; correction is separate in follower_mujoco_kinematic.json.
Reapplication backs up the previous correction. Changes to raw joint calibration, CAD, or fixed-base calibration invalidate old corrections.
Changed measurements require refitting; fit supports recomputation separately.

## Mapping, recovery, and checks

`q_corrected = q_original + offset`; inverse conversion subtracts the offset,
so planned corrected angles map consistently to physical motor targets.
After restart, Push-T, circles, encoder display, and limits share the mapping.
Offline Push-T encoder ranges use the capture session's effective mapping.
Native PID, V14 compensation, RAM velocity/acceleration are unchanged.
An invalidated old Reset pose must be taught again after mapping changes.

To restore raw mapping, close windows, archive follower_mujoco_kinematic.json on both hosts, and restart.
Corrections/samples synchronize locally; all fitting/MuJoCo computation stays local and remote only samples/applies accepted results.
The dataset validates fixed-finger height, not horizontal position, orientation, or all six-axis absolute accuracy.

At that stage 166 local and 190 remote related regression tests passed:
noisy simulated calibration, independent-validation failure, unobservable parameters, read-only sampling,
forward/inverse correction, offline limit consistency, and native pitch scale on actual table-contact data.
Effective scales/zeros matched both hosts. No physical targets were sent; running processes do not hot-load mappings.
