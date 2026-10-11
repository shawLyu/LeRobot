# SO-101 leader/follower demo

The current white `arm1` is the acquisition leader, with a fixed handle and moving
trigger. The black `follower` retains its gripper. Both use motor names 1–6;
channel `gripper` on the leader means **trigger**, not a physical jaw.

Run these commands on the local MuJoCo workstation. Close the tabletop viewer and
arm-calibration window first: each USB bus has one exclusive owner.

```sh
# Real white leader -> real black follower (default); E starts, Space holds.
./scripts/lerobot-arm demo follow

# Read-only real leader input -> simulated black follower.
./scripts/lerobot-arm demo follow --simulate

# Entirely offline, using the preserved desktop bases and dimensions.
./scripts/lerobot-arm demo follow --source synthetic --auto

# Offline manual leader: 1–6 selects a joint; Up/Down changes its angle.
./scripts/lerobot-arm demo follow --source synthetic
```

The default command selects the physical follower and requires independent
six-joint calibration of both arms and measured trigger travel. It does not
silently fall back to simulation when those measurements are missing. The
window starts held; press `E` to begin following. `--execute` remains an explicit
alias for the hardware mode. Synthetic input selects an offline simulated follower.

The `--simulate` preview is a kinematic visualization of commanded follower joint angles, not
a physical torque/contact simulation. The hardware preview reads both buses but
writes no motor registers. White angles are explicitly marked temporary/shared
until its own six-joint file exists. An incomplete or invalid white file is rejected.
The simulation uses `data/tabletop/current-scene/` when available and falls back
to the versioned `configs/scenes/tabletop-current.json` for a fresh checkout.

`E` starts/resumes following; `Space` holds. Targets begin at the current follower
pose and approach the live leader pose at at most 0.6 rad/s. Five arm joints mirror
angles in the existing canonical frame; trigger travel maps proportionally to
follower gripper opening. Values are clipped to the configured model/encoder limits.
Stale samples, motor faults, client heartbeat loss and connection close hold the
physical follower. Holding preserves support on gravity-loaded joints. Resume
requires `E`; it does not restart automatically. The leader bus remains read-only.

## Enable the physical follower after independent measurements

The real mode is implemented but **has not been run on this scene**. White-arm
independent alignment is still pending. Its temporary black-arm mapping cannot
drive the physical follower.

1. Complete white `arm1` alignment in `view arm --backend hardware --arm arm1`.
   Joints 1–5 use 0/30/60 degrees; trigger joint 6 uses 0/15/30 degrees.
   Copy the accepted `data/calibration/arm1_mujoco.json` to the remote calibration directory.
2. Measure the leader trigger's closed/open angles in that reference frame. Set
   `leader_trigger_degrees` and `trigger_range_measured: true` in `configs/follow.json`.
   The supplied 0–45° CAD estimate is provisional and blocks physical execution.
   Check the configured follower gripper opening `[0, 90]` degrees against the actual jaw.
3. Deploy the same follow parameters and code to the remote host:
   `./scripts/deploy_code.sh --config`. The already configured white leader role is
   in `configs/arms.toml`; ordinary deployment preserves that hardware-specific file.
4. With the workspace clear and the leader supported, launch:

```sh
./scripts/lerobot-arm demo follow
```

The window initially holds. Press `E` to enable the calibrated black follower.
Fresh current-position goals are confirmed before torque is enabled; the existing
controller restores its temporary RAM acceleration/velocity/P settings on close.
No EEPROM calibration, offsets, modes or travel limits are rewritten.
Runtime records go to `logs/follow-latest.json`; choose `--log` to keep another run.

## Open-source references and model provenance

The joint-space flow follows upstream LeRobot's
[SO leader `get_action`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/so_leader/so_leader.py)
and [teleoperation loop](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_teleoperate.py).
This implementation reuses our measured-radian controller and exclusive low-level
bus connections. Upstream high-level connections also configure/calibrate motors;
they are not used to alter this installation's EEPROM.

The handle, wrist and trigger are from
[The Robot Studio's official leader description](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101/so101_leader_new_calib.urdf),
pinned at `a758567c3978dfeefe282ede0500085a48fe8f78`, under Apache-2.0.
`so101_leader.xml` converts that head subtree into our canonical arm frame. The
canonical black follower XML is unchanged, preserving accepted encoder/base hashes.
The leader's inherited `gripperframe` is a compatibility frame, not a measured
leader grasp point. Upstream trigger zero, 0–45° limits and printed-part inertia
are provisional; see its [model README](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101/README.md).
