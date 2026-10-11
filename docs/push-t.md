# Offline SO101 Push-T

Run closed-loop pushing from the included measured scene:

```sh
./scripts/lerobot-arm task push-t
```

Initialize a complete rollout from a fresh observation:

```sh
./scripts/lerobot-arm task push-t --capture
./scripts/lerobot-arm task push-t --capture --headless --output data/push_t/new-rollout
```

`--capture` opens the existing remote fixed camera, reuses camera/six-joint calibration, and reads motors only.
It freezes the black target T, yellow wood T, fixed-base registration, six joint angles, and session textures/lighting.
Both Ts require current visible full/partial observations, and camera/joint data younger than 500 ms at selection.
Static caches, held poses, and lost localization cannot initialize a new rollout.
The black follower base uses one-time ArUco 4x4 ID 7 registration against tabletop AprilTag 36h10 ID 1;
the white arm uses its own ID 6 registration. A fixed base does not expire with visual timestamps,
while camera, Ts, and joints still require fresh data. Old snapshots retain their original base-estimation labels.
The remote connection closes before the entirely local contact rollout; physical motors do not execute simulated actions.
Failure to obtain a valid observation stops with a reason rather than reverting to an old snapshot.
`--capture-timeout` defaults to 90 s, `--host` to lerobot, and `--remote-root` to `/home/xy/Research/lerobot`.
Without --output, each capture creates a timestamped directory.

If capture waits for robot base registration, its message includes the registration status or rejection reason.
`markers_lost` requires a clear, fully visible ID 7. `pose_rejected` means the marker was detected but its
corners do not fit the configured mounting geometry; the message includes the pixel error and limit.
Measure the black border excluding the white margin (currently 42 mm), and its plane height above the table
(70 mm). Check the annotated corners against the actual outer black border: a detector can decode ID 7
while choosing a smaller quad inside that border. `border_seed_native` and `prefer_observed_border` constrain
the existing edge-line recovery to the observed border and require exact decoding of the original pixels.
These search regions describe the current 1920x1080 viewpoint and need checking after moving the camera or base.
Keep the 1.5 px geometry limit; increasing the capture timeout cannot fix rejected geometry.

On 2026-10-11, replaying 210 saved frames with the current border search regions accepted 196 follower
observations (median error 0.55 px). A read-only remote capture then saved 20-sample fixed follower
registration with 0.59 px error and 0.36 mm maximum position spread. A second capture restored that
registration and selected a fresh initialization on frame 4. This verifies capture startup, not task completion.
The later October 11 current-scene check found a different visible desktop installation:
right ID 1 and left ID 0. It replaced the ID 3 reference, verified the new layout, and
recollected both bases; current parameters are in [the saved scene](current-scene.md).
When a snapshot includes another robot, offline Push-T preserves that robot's recorded
posture as static geometry. It keeps the six follower joints and wood free joint as
the dynamic state; additional unactuated joints cannot reset to zero or fall under gravity.

Each observation is saved under `observations/<timestamp>/` with state.json, initialization.json,
calibration.json, joint_calibration.json, frame.jpg, native_frame.jpg, annotated.jpg, raw observations, materials, and CAD.
state.json always represents the selected frame; later latest.json updates do not replace it.
Replay initialization with `--snapshot <observation-directory>` without another remote connection.
`trajectory.jsonl` records actually executed joint positions/velocities/targets and object poses at about 62.5 Hz;
candidate predictions are excluded. `episode` distinguishes window resets; result.json records the endpoint and initialization source.
A single frame supplies pose only, so initial velocities are zero. Startup simulates 160 ms of settling,
while preserving raw observed poses in state.json and the measured_start keyframe.

A historical fresh-observation check at `data/push_t/fresh-observation-rollout/` selected camera data about 123 ms old
and motor data about 44 ms old. The original gripper completed 19 simulated pushes in 33.314 s,
ending at 1.368 mm position error and 1.749 degrees yaw error.

Physics, SO101 IK, candidate contact prediction, closed-loop planning, rendering, and saving run locally on Mac.
Remote motor communication, control, watchdogs, camera capture, and perception remain remote responsibilities.
This offline entry does not control the physical robot; see [Live hardware Push-T](push-t-remote.md).
Single-arm/circle simulation also runs locally; see [Realtime operation](realtime.md).

GUI playback follows simulation time by default; --realtime is explicit, --fast computes as quickly as possible.
Headless verification always runs fast. The window targets about 60 Hz, caching panel layout/static text.
A background worker predicts the endpoint and plans the next push during current execution.
Precomputed plans are accepted only when the full actual dynamics state matches; changed states discard them.
Startup still requires initial planning. The fixed observation camera is side-facing; chunk boundaries no longer lift the arm.

Prefer continuous contact or collision-checked low transfers. Lift to switch sides only if low paths are infeasible or make no progress.
Lifting avoids geometry rather than acquiring camera views. At chunk end the reference stays near 3.5 mm pushing height.
Space pauses/resumes after the current push; R restores the measured start; N randomizes the task-region start.
Arrows adjust initial XY by 15 mm and pause; Q/E adjust initial yaw by about 17 degrees.
Mouse controls the view; Esc exits. Object pose is set only during reset; executed pushes use MuJoCo contact dynamics.

## Measured scene

`data/push_t/measured/` is an independent snapshot of the last valid observation from session `20261006-133226-786103`.
It contains state, images, materials, CAD, camera calibration, and motor angle calibration.

| Object | X (m) | Y (m) | Yaw (rad) |
| --- | ---: | ---: | ---: |
| Black target T | -0.38970576 | 0.23385742 | 3.10668607 |
| Yellow wood T | -0.48547801 | 0.32775155 | 2.65290046 |

Both contours are 48x60 mm with 12 mm cap depth/stem width; wood thickness is 6 mm.
The SO101 base retains that frame's visual_estimate position `[-0.40323522, -0.04597901, 0.0024]` and orientation;
this historical mounting transform is not precise calibration. Textures/lighting come from observed reconstruction.
Source observations and original MJCF remain unchanged.
Default offline execution opens no camera, SSH, serial, or motor connection; explicit --capture adds read-only remote initialization only.

## Contact and control

The yellow T consists of two nonoverlapping boxes, a free joint, gravity, and table friction.
The printed target has no collision thickness. Table workspace follows measured localization bounds, not actual table edges.
SO101 retains full six-joint CAD; six simulated position actuators are limited to 2.94 Nm.
Joint limits intersect CAD ranges with existing encoder calibration ranges.

The pusher is the **original SO101 gripper**, held closed at original gripper angle -0.16 rad.
Original fixed/moving finger boxes, fingertips, and CAD collision meshes remain; there is no extra probe, sphere, or extension.
`push_contact` is an invisible reference at the fixed-finger capsule center, with no mass/contact force.
Actual contacts use original finger geometry. Wood density, friction, and actuator gains remain simulation assumptions;
physical T mass/friction have not been measured. Source CAD, measured target/start, base estimate, and calibration are retained.

After each push the controller reads the complete simulated T pose and replans:

1. Sample contact points on eight T edges, generating normal/oblique pushes with error-dependent lengths.
2. Rank using contact torque estimates, then predict translation/yaw with the full SO101 model.
3. Prefer contact continuation and low transfers. Solve IK and check limits, table penetration, and wood collisions using full pose/original gripper geometry.
   Existing compliant contact may be exited, but penetration cannot deepen or end inside wood.
   If needed, retract horizontally 8–16 mm before a low transfer. Normal compliant wood/table contact is not treated as a robot transfer collision.
   Generate lifted transfers only when low candidates cannot progress; unreachable candidates are rejected.
4. Recompute feasible candidates at several friction values and choose by worst predicted error.
5. Simulated position actuators execute transfer/push with the closed gripper, briefly hold low, observe, and replan.
   Near the goal, submillimeter lengths avoid a 2 mm minimum obstructing final corrections.

Prediction and execution share the six-joint SO101, original gripper geometry, and actuators.
Prediction copies full current dynamics state and simulates transfer, pushing, and low hold.
Necessary lifting/descent effects on the wood are included rather than approximated by a narrow point pusher.
State is simulation ground truth; visual localization noise is not modeled.
Table collisions are checked; robot self-collision is outside this verification scope.

Success requires center error below 1.5 mm and yaw error below 3 degrees.
The gripper remains at pushing height while wood settles and the final pose is checked.
Arbitrary starts mean reachable, unobstructed poses within the table workspace with a feasible push path;
there is no convergence proof for all table positions, obstacles, or initial states.

## Replay and verification

```sh
./scripts/lerobot-arm task push-t --headless --output data/push_t/check
./scripts/lerobot-arm task push-t --headless --benchmark 20 --seed 17 --output data/push_t/continuous-contact-validation-final
PYTHONPATH=src /Users/shawn/Research/RobotSimulator/URDFSim/.venv/bin/pytest -q tests/test_push_t.py
```

The last command preserves the historical workstation interpreter; use your local test environment as appropriate.
Outputs include scene.xml with relative textures, rollout.xml with the same prediction model, result.json or benchmark.json.
Results identify `pusher: original_so101_closed_gripper` and original geometry contact counts in jaw_contacts.
The measured_start keyframe preserves initial joints/wood; mj_resetDataKeyframe(model, data, 0) restores it.
Each actual push records contact, direction, length, pose, terminal finger height, and transition type.
`chunk_end_retraction: false` means no end-of-chunk lift; lift_transfers counts necessary next-chunk lifts,
and contact_continuations counts uninterrupted contact pushes.

Benchmarks include the measured start and random task-region positions/full +/-pi yaw,
friction 0.2–0.65, and density 450–950 kg/m^3. Failures remain in the reported success rate.
This is finite-sample simulation validation, not physical success rate. Unreachable cases display failure and stop.

## Recorded results

After removing mandatory lifts, 20 related local tests passed. All 20 seed-17 benchmark cases succeeded;
maximum center error 1.478 mm, maximum yaw error 2.937 degrees.
None of 337 chunks lifted at its end; transfers included 126 lifts and 65 contact continuations.
The measured start completed in 29 realtime chunks, with nine lifts/eight contact continuations,
terminal reference heights 2.979–3.321 mm, and final errors 1.486 mm / 1.631 degrees.
Mean rendering was 44.39 Hz; all 28 precomputed plans passed full integration-state checks.
Evidence paths: `data/push_t/continuous-contact-validation-final/benchmark.json`,
`data/push_t/continuous-contact-demo-final/result.json`, `logs/push-t-contact-transition-verification.json`,
and `logs/push-t-continuous-contact-final.png`.

Earlier extra-probe benchmarks do not validate the original gripper.
Original-gripper records include `data/push_t/gripper-validation/benchmark.json`,
`data/push_t/gripper-demo/`, and `logs/push-t-gripper-simulation.png`.
The following pre-low-transfer results do not replace validation of the current action strategy:

All 20 seed-17 original-gripper cases succeeded, with maximum center error 1.446 mm and yaw error 2.955 degrees.
Eighteen local tests verified nominal prediction/execution agreement without modifying current dynamics,
background full-state matching, and stale-plan rejection after state changes/resets.
The realtime window completed 14 pushes at mean 50.63 Hz; 13 later plans reused validated precomputation.
Final error was 0.301 mm / 2.497 degrees. Maximum frame gap was 132 ms, so brief scheduling/render delays remained.
Evidence: `data/push_t/smooth-demo/` and `logs/push-t-smooth-simulation.png`.
Historical per-frame outputs may have been removed; see [Data retention](../data/README.md).
