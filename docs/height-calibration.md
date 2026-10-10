# Fixed fingertip height calibration

After angle mapping and base registration, measured heights refine shoulder lift, elbow, and wrist pitch zero offsets.
Sampling is read-only on the remote host. MuJoCo fitting and independent validation run locally;
accepted corrections are applied to both hosts.

## Workflow

Close other arm, Push-T, and Reset tools to release the serial port.
Support and manually position the robot. Sampling does not change torque or move motors.
Enter the lowest fixed finger surface's measured height above the table in millimeters;
empty Enter means 0 mm, and q saves and exits.

```bash
./scripts/lerobot-arm calibrate height collect --count 10
./scripts/lerobot-arm calibrate height fit
./scripts/lerobot-arm calibrate height apply
```

Vary reach and wrist orientation; shoulder pan alone cannot identify all three offsets.
Gently touch the table or a rigid, flat spacer of known height, without pressing against the surface.
The default dataset needs at least 10 distinct poses, with every fifth pose reserved for independent validation.
Duplicate poses, moving readings, and motor faults are rejected.

Replace specific samples using 1-based indices, then fit and validate:

```bash
./scripts/lerobot-arm calibrate height resample --poses 3 5 7 --apply
```

Old samples are retained under `superseded`. `collect --apply` also performs fitting/validation/application after capture.
The existing STS3215 setup has verified 4096 ticks per revolution. To refit with its native pitch scale:

```bash
./scripts/lerobot-arm calibrate height fit --native-pitch-scale --apply
```

This mode uses the hardware angle scale and fits three zeros, requiring at least five fitting and two independent validation poses.
It is not a bypass for raw calibration on arbitrary hardware.
Override connections with `--host` and `--remote-root`.

## Files and acceptance

Before fitting, the height command copies remote raw angle, base, and sample records:
`follower_mujoco.json`, `follower_table_base.json`, and `height_samples.json`.
It saves `follower_height_candidate.json`, then applies accepted results to `follower_mujoco_kinematic.json`.
Raw calibration is retained; previous corrections are backed up. Application writes data only, with no motor motion.

The ordinary mode retains encoder scale and fits three zero offsets, each bounded to +/-5 degrees.
Acceptance checks observability, fit RMSE <=1.5 mm, independent validation RMSE <=2 mm, and maximum error <=3 mm.
When original validation error exceeds 1 mm, at least 25% improvement is also required.
Rejected candidates cannot be applied. Changed samples require refitting;
changed raw calibration, base, or model identity invalidates old corrections.

The correction is `q_corrected = q_original + offset`; inverse conversion subtracts the same offset.
Restart processes for encoder display, Push-T, and circles to load the effective mapping;
control settings use the shared `configs/control.json` configuration. Teach Reset again after mapping changes.
To restore the original mapping, close relevant windows, archive `follower_mujoco_kinematic.json` on both hosts, and restart.

This procedure validates fixed fingertip height, not horizontal position, orientation, or absolute precision at every height.

## Existing device measurements

On 2026-10-07, `fixed_jaw_height_native_pitch_scale` was applied using five table-contact fitting poses and two independent validation poses.
Fit RMSE was 0.562 mm; validation RMSE 0.881 mm; maximum error 1.137 mm.
These are historical results for the earlier base installation. After the two-arm base recalibration
on 2026-10-07, that correction, candidate and sample set were archived under
`.cleanup/20261007-two-arm-registration/calibration/` on both hosts.
The active angle mapping remains the original measured mapping without this height correction.
Collect and validate new height samples against the current follower base before relying on fingertip height.
The historical results do not establish complete physical Push-T success.
See [Height calibration history](history/height-calibration-initial.md) for the failed first round, resampling rationale, offsets, and historical tests.
