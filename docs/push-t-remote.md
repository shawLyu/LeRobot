# Visual closed-loop Push-T: local planning, remote execution

```sh
./scripts/deploy_code.sh
./scripts/lerobot-arm view tabletop --push-t --fps 5
```

Camera capture, calibration/undistortion, table/T localization, and robot feedback run remotely.
The local host receives images/poses, extracts CAD-masked materials with local MuJoCo, predicts contacts,
solves IK, and plans short pushes. The remote host has no MuJoCo; it receives validated joint segments or preview results.
Local outputs: `data/tabletop/<session>/push_t/`.
Remote received records: `data/push_t/remote-<session>/plans.jsonl` and local-results.jsonl.
Default local display is 5 fps. Push-T requests a 10 Hz observation stream (or the higher display rate), independently of rendering; actual delivery is bounded by perception throughput. Motor control targets 60 Hz with exclusive serial access.
Push-T omits native JPEG transmission while retaining native-resolution marker detection remotely; ordinary tabletop/calibration captures still retain native images.
Local material extraction runs in its own thread without blocking feedback reception.

Display transmission uses a separate thread and one latest-state slot, so slow rendering/backlogs do not block perception.
The remote motor owner holds on camera/motor age over 500 ms, local heartbeat gap over 750 ms, or SSH loss.
During an active trajectory, local display-frame age does not override fresh remote control observations.
Local stream loss still cancels execution and stops heartbeats; planning/submission still require fresh observations.
Photographic material extraction is optional for planning/execution. If the
primary view cannot see a surface but another accepted view supplies its measured
pose, planning uses the measured geometry and ordinary CAD materials. Texture
extraction never replaces the live observation, calibration or collision checks.
Remote `data/tabletop/static_map.json` retains the auxiliary marker's measured relative pose,
allowing the same frame when the primary marker is occluded. Reobserve both after moving markers.

## Hardware execution

```sh
./scripts/lerobot-arm view tabletop --push-t --hardware-control --fps 5
```

For each segment the remote executor checks shared configuration, model, limits, velocity, continuity,
reference freshness, and scene consistency. Execution continues using current remote observations and encoders.
The window shows the live scene and next locally planned push.
P starts or replans from hold; Space stops progression and holds; Esc closes and ends remote execution.
--hardware-control permits execution but launch itself does not move motors.
Without it, feedback/planning remain read-only and P cannot initiate physical motion.

Hardware startup requires verified motor identity, model hash, and an available executor.
Serial open failure, missing motor packets, or feedback initialization failure are returned with the actual error.
The local client stops before constructing an execution proxy/planner and reports
`Remote SO101 is not ready for hardware control`.
Check power, USB/serial, and exclusive ownership before restarting.
Read-only preview can still show the camera, with motor errors displayed/logged;
it does not present old calibration as live feedback.

The current experimental `robot.push_height_compensation_m = 0.005` raises each physical fingertip target by 5 mm
along tabletop Z, on top of kinematic calibration. It resolves IK and checks collisions/limits before generating trajectories at unchanged speed.
XY is unchanged; contact planning still uses nominal height.
This empirical correction requires physical observation and does not establish contact with the 6 mm wood.
preview.json/state record height_compensation_m. Occlusion recovery uses the measured pose without this offset.
Set it to 0 to disable, then restart.

## Fixed base registration

Physical execution requires current tabletop/T observations, six-joint calibration, and trusted base registration.
The left black follower uses ArUco `DICT_4X4_50` / ID 7; the right white arm1 uses ID 6.
Each arm independently saves `calibrated=true`, `fixed_to_table=true`, `status=fixed`,
`pose_source=fixed_marker_calibration`. Runtime reads saved mounting poses while
follower joints update from encoders. Tabletop and T observations remain live.
The white arm registration does not enable joint control.

Both markers have a 42 mm black border, lie parallel to the table, and are centered laterally.
The follower marker center is 70 mm above the table; arm1's configured height is 69 mm.
Both centers are 19.5 mm forward of the base rear edge.
CAD rear edge X=-22.365 mm and table-contact Z=-2.400 mm give marker centers
`[-2.865, 0, 67.600]` mm for the follower and `[-2.865, 0, 66.600]` mm for arm1.
Decoded corner order accounts for each printed marker's rotation.
Known corner height and a flat-base constraint solve planar position/yaw while retaining raw joint calibration.
Old MIP ID 4/5/6 installation geometry has been removed from the active configuration.
See [Two-arm calibration](two-arm-calibration.md) for measurements and saved poses.

Initial registration requires visible primary ID 1 and 20 valid observations with distinct capture times spanning at least 1 s.
Position deviations from the median must be below 2 mm and yaw deviations below 0.025 rad.
Results are atomically saved remotely as `data/calibration/follower_table_base.json` and
`data/calibration/arm1_table_base.json`, then copied locally. Each arm requires its own 20 accepted observations.
Restart restores it, even if the base marker is subsequently hidden.
The cache validates marker mount geometry, table reference, and CAD signature; changed signatures require registration again.
Before replacing the follower base, archive its base-bound height correction, candidate and samples,
then collect and validate new height measurements after registration.
After moving the base or primary anchor, archive/recollect with:

```sh
./scripts/lerobot-arm view tabletop --recalibrate-base --fps 5
```

In the measured scene, follower ID 7's dark surrounding base causes an oversized contour around the white margin.
The standard ID 6 detector also selects a border inside the white arm's actual black square.
A supplementary detector constrains search using approximate native-image corners, refits actual edge lines,
and decodes original pixels; it accepts only the exact expected ID (7 or 6) without synthesizing occluded pixels.
The observed border is enforced after every detection path for these two markers.
This branch stops after calibration is saved.
Changed viewpoints require clear standard decoding or an updated border_seed_native search region.
Fixed-base registration does not need fresh visual timestamps; live camera/T/table/joint checks remain effective.
Registration establishes mounting pose, not completed physical pushing.
Deployment preserves remote calibration; explicit `./scripts/deploy_code.sh --config` replaces tabletop.json.

## Segment execution

Push segments are at most 4 mm, reference speed 4 mm/s, contact-stage joint speed at most 0.15 rad/s.
Free approach shares 0.8 rad/s and minimum segment duration 0.35 s with circle control.
Both stages use quintic interpolation, with peak slope 1.875 accounted for in velocity bounds.
MPC contact prediction uses the same interpolation/timing, original gripper geometry, and multiple friction coefficients.
Starts/targets must lie within encoder/CAD limits. Object, base, and actual joints are rechecked before action;
changes force replanning. After encoder arrival, the next segment waits for a camera frame captured after action completion.
A previous simulated prediction is never substituted for the next actual T observation.
Success requires center error <2 mm and yaw error <3 degrees in three distinct camera frames, with at most 100 pushes.

Enabling first writes current actual positions as hold targets, then enables torque/RAM acceleration streaming without EEPROM writes.
Joint tracking error over 0.12 rad, faults, or lost torque cancel subsequent targets.
Stopping/disconnection retains loaded-joint support and restores prior RAM acceleration/velocity.

## Shared circle controller

Physical Push-T uses the shared ManualControl and `configs/robot.json`: 60 Hz,
RAM acceleration 254, velocity 3000 ticks/s, reference shaping and bounded PI-D.
Predictive velocity lead is disabled. Native P/I/D, EEPROM protections and measured
angle calibration remain unchanged. There are no task-specific PID/native-gain
profiles. See [the controller](lerobot-control.md) and
[current response evidence](control-performance.md). Physical contact behavior
and independent 1 mm Cartesian validation remain pending.
Compensated push commands are constrained to encoder/CAD range intersections and
the shared device-bound command envelopes.

The loop shares two packed RAM reads, a 60 Hz grid, and skipped stale slots instead of queued catch-up.
Automatic cyclic garbage collection is disabled during control and restored after hold/controller closure,
reducing previously investigated periodic stalls.
Approach timing comes from control.json; quintic interpolation uses parking.smooth_fraction.
Circle period, ramp, and automatic Reset belong to the circle task; Push-T follows push/recovery paths.

Deployment follows the remote manifest and sole control configuration while preserving device/tabletop/base/joint calibration.
Robot control no longer accepts an independent control-parameter path from tabletop.json.
Session control_config/control_config_sha256 record parameter provenance.
Each motor sample's control records reference targets, compensation, actual sent targets, and RAM stream settings.
Changes do not hot-reload. Exit and restart hardware mode, then press P; physical contact effects still need hardware validation.

Offline tests cover shared control consistency, RAM restoration, calibrated limits,
60 Hz scheduling, fault holds and read-only startup. Offline passes are not
physical contact validation.

## Active occlusion recovery

Normal chunks end low. When T/reference observations disappear, pause pushing and check a recovery path
using trusted T/table/base geometry from the last second plus latest encoders.
Prefer raising the original gripper reference by 25 mm. If infeasible, retreat outward 15 mm slowly, then raise 25 mm.
Check full original gripper geometry, table, known T contour, and limits. No feasible path means no blind retreat.

Recovery temporarily tolerates missing T/reference markers using recent trusted geometry,
with planned duration at most 7 s and execution at most 8 s.
The camera must still deliver new frames, and encoder/tracking checks remain active.
After recovery, wait to reacquire T and resume from its observed pose. Recovery does not count as a push.
Camera loss, faults, or no recent trusted geometry keep the robot stopped.
Reachable paths and serial state transitions were tested with hardware substitutes and measured-scene simulation;
physical Push-T recovery remains unverified.

## Recordings and historical checks

Local `data/tabletop/<session>/push_t/` saves initialization, CAD/textures, observations.jsonl, plans.jsonl,
preview.json, recoveries.jsonl, and result.json on success. Images/motor samples live in the outer session.
The remote host saves only received plans.jsonl and local-results.jsonl.

The following records precede the responsibility split. References to remote planning/model saving describe historical deployment;
those computations now run locally. Full historical per-frame outputs may have been removed; see [Data retention](../data/README.md).

Marker localization succeeded, both Ts were tracked, contact actions were planned, and the local stream was about 5 fps.
A segment at 0.15 rad/s joint limit and 4 mm/s reference speed produced original moving-finger contact in MuJoCo.
These were previews, simulated motors, and historical remote simulation, not successful physical Push-T.

A 65 s ID 4 read-only preview at `data/push_t/id4-closed-loop-preview/` saved 299 displayed frames:
298 tracked/held base frames and one lost frame. A 4 mm low push was preplanned; initial planning took about 29 s.
Historical remote MuJoCo replay at execution timing produced 3140 original-gripper contact samples.
Center error decreased from 66.79 to 63.75 mm; yaw error changed from 34.00 to 34.32 degrees.
This verifies one simulated segment, not a complete physical rollout.

A 15 s ID 6 read-only capture at `data/tabletop/id6-fixed-calibration/` saved 55 local display frames.
Twenty valid remote observations locked the base; maximum position deviation was about 1.55 mm and yaw deviation 0.74 degrees.
These measure observation stability, not externally measured absolute accuracy.
A restart restored the fixed base on the first frame and continued live joint feedback at `data/tabletop/id6-fixed-realtime/`.
No physical motor targets were written.

## Current hardware strategy status

The policy uses actual camera estimates of both Ts, fixed follower ID 7 registration, and six-joint calibration.
Original gripper CAD, IK, limits, and collision checks filter contacts.
Friction values 0.22, 0.4, and 0.65 provide worst-case scoring; each physical push is at most 4 mm.
Afterward, a frame captured after completion drives replanning. Normal transfers stay low;
fixed-base registration can support bounded occlusion retreat and reacquisition.

Encoder starts outside CAD limits enter waiting for valid joint pose, displaying offending joints/angles while receiving feedback.
Observed poses are not clipped, and the planner thread does not terminate. Valid new feedback restores planning.
Forty-eight related remote tests covered fixed-base recovery, no action from invalid starts, and resumed planning.

The 2026-10-06 read-only precheck measured shoulder -100.68 degrees (CAD lower bound -100)
and gripper -10.43 degrees (CAD lower bound -10), so physical push generation was disallowed.
The feedback/execution policy is implemented and tested in simulation/with hardware substitutes;
a full physical Push-T rollout remains unverified.
Recorded policy preview: `data/push_t/id6-policy-preview-verified-20261006/`.

If a preview still matches current observations, P reuses the candidate after rereading joints,
bounding interpolation speed, and completing motor prechecks.
Object/joint changes invalidate the cache; each real action clears it so the next segment requires a fresh post-action observation.
The UI displays elapsed planning time and Start requested; computation no longer leaves stale Executable: true.
Motor checks accept measured live base registration and valid fixed_marker_calibration;
invalid calibration is rejected before writes. Compatibility/preview reuse fixes used substitute-hardware tests,
without automatically restarting live physical control.

## Torque telemetry and prediction performance fixes

In recorded motor samples 4764–4768, control was enabled but cached telemetry still reported all six Torque_Enable=0.
Sample 4769 reported all 1; Status stayed all 0. The former 100 ms telemetry refresh caused a false lost-torque stop
on the first cycle after enable. Hardware control now reads Torque_Enable/Status each cycle and independently confirms
abnormal bulk reads before rejecting motion. Actual torque loss/faults still stop; the UI preserves the stop reason.
Regression tests cover stale telemetry after enable, real fault stops, and per-cycle critical-register refresh.

Profiling found about 350,000 mj_step calls per full candidate plan, mostly in free approach before contact.
Prediction now advances only collision-free approach kinematically, sampling the complete joint path at roughly <=0.015 rad
and checking original gripper/robot collisions. Any contact restores initial data and falls back to full dynamics.
Only a verified contact-free approach advances directly to the contact simulation.
Contact prediction and actual execution retain physics, speed, and interpolation; actual simulation execution never fast-forwards.

Before circle-controller synchronization, a frozen-observation historical remote low-priority comparison measured
21.637 s full planning versus 9.655 s optimized planning.
For the chosen 4 mm action across three friction settings, final prediction differences were floating-point rounding only.
This covers that scene; live runtime still depends on geometry/load.
Evidence: `data/push_t/planning-performance-20261006/prediction-profile.json` and profile-state.json.
Forty-eight related remote tests passed; running hardware processes do not hot-reload changes.

## Measured start pose and model joint references

`robot.simulation.joint_reference_degrees` in `configs/tabletop.json` specifies
physical encoder angle minus CAD angle. It changes generated MJCF `ref` and ranges;
encoder feedback, joint-angle calibration, EEPROM and hardware command limits stay unchanged.
The mapping is bound to `joint_reference_calibration_sha256`; recalibration requires review.
The current follower jaw maps the saved closed command endpoint (raw 464, −92.7273°)
to CAD closed jaw −0.16 rad, with a −83.55995° model reference.
Push IK, live visualization and the remote trajectory validator use the same coordinates.

`robot.push_t.entry_tolerance_degrees` is currently 5° (maximum allowed).
It accepts small measured starting-pose deviations, including a manually closed jaw
just beyond the normal command envelope. Feedback is never clipped or relabeled.
Only the first segment's measured start gets this allowance: every destination remains
inside the intersection of model and calibrated encoder limits; the hardware owner
also checks its saved command envelope before enabling torque. Collision, freshness,
base-registration, speed and alarm checks still apply. The accepted pose is a planning
seed, not a declaration that arbitrary subsequent motion is collision-free.

2026-10-19 validation: replaying the rejected observation (gripper raw 445 / −94.2513°)
passed the revised seed check and produced 11 strictly bounded segments. A subsequent
35-second live read-only session (`logs/valid-joint-pose-live/`, 160 camera states)
reached `preview ready` with a 4 mm next-push plan; both arms reported live independent
encoders and follower torque stayed off on all six joints. This validates observation
and planning; no physical push was executed in this check.

2026-10-19 execution interruption fix: hardware sessions at 16:35 and 16:38 reached
`running`, then local display-age checks cancelled their slow entry within about 0.5 s.
The worker now uses remote execution status while display packets continue arriving;
remote camera/encoder watchdogs remain at 500 ms. Stop commands preserve their actual
reason rather than replacing camera faults or rejected plans with `operator stop`.

The subsequent 25-second read-only validation (`logs/push-t-stream-validation/`)
received 136 states, with mean capture age at transmission about 184 ms. After texture
initialization it planned once and retained `preview ready`, without camera-age holds
or repeated planning. Native images were not transmitted. Physical execution was
not requested during this validation.
