# SO-101 leader/follower demo

The current white `arm1` is the acquisition leader, with a fixed handle and moving
trigger. The black `follower` retains its gripper. Both use motor names 1–6;
channel `gripper` on the leader means **trigger**, not a physical jaw.

Run these commands on the local MuJoCo workstation. Close the tabletop viewer and
arm-calibration window first: each USB bus has one exclusive owner.

```sh
# Real white leader -> real black follower; automatically initializes/starts; Space holds.
./scripts/lerobot-arm demo follow

# Start held instead; press E when ready.
./scripts/lerobot-arm demo follow --start-held

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
client automatically initializes and begins following once its window is ready and focused
and fresh feedback is available. This happens once per launch. Set `--start-held`
or `initialization.auto_start: false` to require `E` at startup. `--execute` remains an explicit
alias for the hardware mode. Synthetic input selects an offline simulated follower.

The `--simulate` preview runs local MuJoCo dynamics with the shared 60 Hz joint PID.
It displays measured simulated motion rather than assigning follower target angles directly.
The hardware preview reads both buses but
writes no motor registers. The current white arm uses its own accepted six-joint
calibration; temporary sharing is disabled. An incomplete or invalid white file is rejected.
The simulation uses `data/tabletop/current-scene/` when available and falls back
to the versioned `configs/scenes/tabletop-current.json` for a fresh checkout.

Connection initializes the displayed poses from live encoder feedback. Automatic startup or `E` starts/resumes
the follower from a fresh current-position hold. Small model-limit violations are
corrected at the normal speed limit to 0.5 degrees inside the usable range; the
entire correction must be at most 2 degrees for arm joints and upper-limit recovery.
The calibrated physical gripper can close below the CAD lower stop (about −12.43°
versus −10°). Only this lower-stop case allows up to 5° of opening recovery,
at the same bounded speed and within calibrated encoder travel. It uses the same automatic startup or explicit `E` initialization.
Initialization does not move the other joints or remap the follower gripper. Once all
commanded targets are inside their limits and within 0.25 degrees of the initialization
target, following begins from that command, preserving the speed bound. This is a
command-ramp initialization, not precision settling: native encoder tracking error
is displayed and recorded without blocking teleoperation. Physical encoder travel,
mode, voltage, fault and torque checks still apply.
An initialization lasting over 5 seconds holds and requires `E` to retry. Larger
model-limit violations or positions outside calibrated encoder travel are rejected
before torque is enabled; reposition those manually. The passive white leader is read-only.

`Space` holds. Targets begin at the current follower
pose and approach the live leader pose at at most 0.6 rad/s. Five arm joints mirror
angles in the existing canonical frame; white trigger travel 0–118° maps proportionally
to black gripper opening 0–90° (59° gives 45°). Calibration references 0/15/30°
do not define the travel endpoints. Values are clipped to the configured model/encoder limits.
Stale samples, motor faults, client heartbeat loss and connection close hold the
physical follower. Holding preserves support on gravity-loaded joints. Resume
requires `E`; it does not restart automatically. The leader bus remains read-only.
Feedback delays above 0.25 seconds request hold and leave the window open. Fresh
feedback restores the display while motion remains held until another `E`.
If the remote worker exits, the window displays the cause and remains open;
close it and relaunch after resolving the connection or motor fault.

Automatic startup was verified on both real buses on 2026-10-12: the client entered
following without an E keypress, then held when the window lost focus. The six-second
session completed without command errors or stream-delay events. Post-checks confirmed
all six black motors held with torque on, all six white motors remained torque off,
no motor faults, and unchanged calibration hashes. The compact record is
[`automatic-initialization.json`](measurements/pid-2026-10-12/automatic-initialization.json).
Queued heartbeat timestamps cannot rewind the completed initialization timestamp;
command expiry is checked at processing time, including after motor preflight.

## Enable the physical follower after independent measurements

White-arm independent alignment is complete (6/6). The user corrected the released/fully
pressed trigger endpoints to 0/118 degrees on 2026-10-11, and the current
follow configuration uses this range. Both arms require their own valid mapping.
The latest read-only capture recorded follower `shoulder_lift` at -100.85 degrees,
just below its -100-degree model limit. This small violation is eligible for the
default bounded initialization at startup.

On 2026-10-11, real-bus validation confirmed enabling with all six storage locks
still at 0, native goal writes and actual multi-joint motion. A subsequent short
0.02 rad/s run completed held → initializing → following → held with no command
errors. After exit, black goals matched its measured positions and all six torques
remained on; the white arm remained torque-off. P=16, acceleration=254, velocity=0,
storage locks and calibration were unchanged. Production speed remains 0.6 rad/s.

1. Complete white `arm1` alignment in `view arm --backend hardware --arm arm1`.
   Joints 1–5 use 0/30/60 degrees; trigger joint 6 uses 0/15/30 degrees.
   Copy the accepted `data/calibration/arm1_mujoco.json` to the remote calibration directory.
2. Measure the leader trigger's closed/open angles in that reference frame. Set
   `leader_trigger_degrees` and `trigger_range_measured: true` in `configs/follow.json`.
   This installation now uses the confirmed 0–118° endpoints; repeat measurement
   after replacing or rebuilding the leader handle.
   Check the configured follower gripper opening `[0, 90]` degrees against the actual jaw.
3. Deploy the same follow parameters and code to the remote host:
   `./scripts/deploy_code.sh --config`. The already configured white leader role is
   in `configs/arms.toml`; ordinary deployment preserves that hardware-specific file.
4. With the workspace clear and the leader supported, launch:

```sh
./scripts/lerobot-arm demo follow
```

The window initially holds. Press `E` to initialize and enable the calibrated black follower.
Fresh current-position goals are confirmed before torque is enabled. The follower uses
`shared-joint-pid-v1` from `configs/control.json`, exactly as the other task controllers.
Follow frequency is derived from that file (60 Hz); `follow.json` cannot override it.
The PID sends bounded compensated position targets. RAM acceleration 40 and velocity
600 ticks/s are restored on exit; existing native P/I/D remain untouched, including Lock=0.
See [the controller and block diagram](lerobot-control.md) and
[response measurements](control-performance.md).
No EEPROM calibration, offsets, modes or travel limits are rewritten.
Runtime records go to `logs/follow-latest.json`; choose `--log` to keep another run.
They include initialization/hold/follow stages, feedback age, command errors,
controller status, target angles and the exit reason. A rejected `E` retains its
specific error in subsequent held-state packets instead of replacing it with a
generic pause message.

## Open-source references and model provenance

The joint-space flow follows upstream LeRobot's
[SO leader `get_action`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/so_leader/so_leader.py)
and [teleoperation loop](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_teleoperate.py),
with synchronized `Goal_Position` writes as in the
[SO follower `send_action`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py).
This implementation reuses our measured-radian controller and exclusive low-level
bus connections. Upstream high-level connections also configure/calibrate motors;
they are not used to alter this installation's EEPROM.

The handle, wrist and trigger are from
[The Robot Studio's official leader description](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101/so101_leader_new_calib.urdf),
pinned at `a758567c3978dfeefe282ede0500085a48fe8f78`, under Apache-2.0.
`so101_leader.xml` converts that head subtree into our canonical arm frame. The
canonical black follower XML is unchanged, preserving accepted encoder/base hashes.
The leader's inherited `gripperframe` is a compatibility frame, not a measured
leader grasp point. Our joint/actuator limits use the user-confirmed 0–118° travel.
Upstream printed-part inertia remains provisional; see its
[model README](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101/README.md).
