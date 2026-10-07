# SO-101 Local Simulation and Remote Robot Control

This project provides a live SO-101 mirror, joint angle calibration, tabletop perception, circle trajectories, and Push-T demos.
**Maintain the complete codebase locally. All MuJoCo simulation, collision checks, inverse kinematics, and planning run locally.**
The remote Linux host handles USB cameras, perception, motor execution, and encoder feedback. It receives locally generated trajectories or results and does not need MuJoCo.

The default configuration belongs to the existing experimental setup. Offline demos can use the included snapshot. For another robot or camera, configure the devices and perform the relevant calibrations.
Unless a section explicitly says **remote**, run its commands from the local project root.

## 1. Install the local environment

Use macOS or Linux. Python 3.12 is recommended; the existing Python 3.11 workstation environment has also been tested.
Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then run:

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements-local.txt
.venv/bin/python -c 'import mujoco, numpy, cv2, PIL; print("local runtime ready")'
```

The local environment uses [requirements-local.txt](requirements-local.txt) for simulation and display dependencies.
`pyproject.toml` and `uv.lock` describe the remote **Linux x86_64** environment: LeRobot, Feetech, and CPU PyTorch.
Do not use `scripts/bootstrap.sh` or `uv sync` to install the local environment on macOS.
The launch scripts set `PYTHONPATH=src`; installing the full LeRobot package locally is unnecessary.

To reuse an existing environment:

```bash
export LEROBOT_MUJOCO_ENV=/path/to/your/.venv
```

The search order is this variable, the project's `.venv`, then the legacy macOS workstation environment at `../Materials/UWLab/.venv`.
New users should use the project's `.venv`. The viewer requires a graphical desktop; use `--headless` for checks without a window.
See the [uv environment documentation](https://docs.astral.sh/uv/pip/environments/) and [MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html).

## 2. Start with demos that require no hardware

### Single arm and circle trajectory

```bash
# Local SO-101 window: no SSH, camera, or motor connection
./scripts/view_arm_realtime.sh --backend mujoco

# Run three circle laps automatically in local simulation
./scripts/view_arm_realtime.sh --backend mujoco --circle --circle-laps 3

# Headless check with an explicit log destination
./scripts/view_arm_realtime.sh --backend mujoco --headless --duration 3 \
  --log logs/simulation-smoke.json
```

In the arm window, use `1`–`6` to select a joint, Up/Down to change its simulated target, `Space` to hold, and `Esc` to exit.
The default backend is `hardware`; offline demos must explicitly use `--backend mujoco`.

### Offline Push-T

```bash
# Use the included independent observation snapshot, played at simulation speed
./scripts/simulate_push_t.sh

# Headless, fast computation, with a separate output directory
./scripts/simulate_push_t.sh --headless --fast --limit 10 \
  --output data/push_t/my-demo
```

The default input is `data/push_t/measured/`; the default output is `data/push_t/run/`.
Repeated runs reuse the default output location. Set a different `--output` to retain separate experiments.
`--limit` caps the number of pushes. Reaching that limit does not mean the task succeeded; inspect the result record.
Window controls: `Space` pauses/resumes, `R` restores the measured start, `N` randomizes the start, arrows adjust XY, `Q/E` adjust orientation, and `Esc` exits.
See [Push-T](docs/push-t.md) for the algorithm, assumptions, and batch experiments.

## 3. Configure and deploy the remote host

The remote runtime uses Linux x86_64, Python 3.12, and LeRobot 0.6.1 with Feetech.
Define a working alias in your local `~/.ssh/config`; replace the address and account below:

```sshconfig
Host lerobot
    HostName YOUR_ROBOT_HOST
    User YOUR_LINUX_USER
```

The defaults are SSH alias `lerobot` and project directory `/home/xy/Research/lerobot`.
Confirm that `ssh lerobot` works, then deploy:

```bash
# Preview changes when deploying to an existing directory
./scripts/deploy_code.sh --dry-run
./scripts/deploy_code.sh

# Override the deployment destination
LEROBOT_HOST=my-robot LEROBOT_REMOTE_ROOT=/home/robot/lerobot ./scripts/deploy_code.sh
```

For the first deployment, run `deploy_code.sh` directly. It creates the destination, which must be under `/home/`.
A dry-run before first deployment requires the destination to exist already.
The script sends the files in the [remote runtime manifest](configs/remote-runtime.txt), plus shared configuration, documentation, and dependency locks.
Before each actual update, it backs up existing code under remote `.cleanup/`. Incremental deployment preserves remote-only files.
`arms.toml`, `camera.toml`, and `tabletop.json` are seeded only when missing. Existing device configuration and calibration are preserved.
Use `--config` explicitly to replace remote `tabletop.json` after changing camera or marker installation settings:

```bash
./scripts/deploy_code.sh --config
```

Then install on the **remote host**:

```bash
ssh lerobot
cd /home/xy/Research/lerobot    # Replace with your deployment directory
./scripts/bootstrap.sh        # First installation: project-local uv and locked dependencies
# Later dependency updates: ./scripts/uv sync --locked
./scripts/uv run --locked arm-control doctor
```

The default remote installation excludes MuJoCo. Camera X11 preview also requires system Tk and SSH X11 forwarding.
On Ubuntu, install `python3-tk` and `xauth` if missing. Check `dialout` for serial access and `video` for camera access; log in again after group changes.
See [Environment and diagnostics](docs/setup.md) for initialization and upstream tools.

**Custom connection settings:** deployment and camera wrappers use `LEROBOT_HOST` and `LEROBOT_REMOTE_ROOT`.
Arm, tabletop, Push-T, Reset, and height calibration clients use explicit `--host` and `--remote-root` options:

```bash
./scripts/view_arm_realtime.sh --host my-robot --remote-root /home/robot/lerobot
```

## 4. Identify robots and cameras

Inspect devices on the **remote host**:

```bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/
ls -l /dev/v4l/by-id/
./scripts/uv run --locked arm-control doctor
./scripts/uv run --locked arm-control read --arm follower --samples 3
```

`doctor` and `read` only read hardware; they do not write targets or calibration registers.
Configure the remote `configs/arms.toml`: port, role, baud rate, and motor IDs. Prefer `by-id` paths with a stable USB identity.
`by-path` describes a physical connection path. Multiple aliases may point to the same device; they do not prove that multiple arms are connected.
`ttyACM0/1` numbering can change after reconnection and cannot identify the follower by itself.

The existing follower is an SO-101 with USB serial `5B8E114461`. Device `5B91028324` is configured as `arm1`, with an unconfirmed role.
The standard IDs are shoulder pan 1, shoulder lift 2, elbow 3, wrist pitch 4, wrist roll 5, and gripper 6. Verify them on each new device.
Calibration binds to USB identity and motor registers. Renaming another robot to follower does not make the original calibration valid.
Only one process should own a serial port at a time.

## 5. View cameras and set resolution

On macOS, install [XQuartz](https://www.xquartz.org/), then run locally:

```bash
# Default camera; remote camera.toml requests 640x480 by default
./scripts/view_camera_x11.sh

# Second camera: open another terminal for simultaneous viewing
./scripts/view_camera_x11.sh \
  --device /dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0

# Temporary capture resolution; verify video numbers on your host first
./scripts/view_camera_x11.sh --device /dev/video0 --width 1920 --height 1080
./scripts/view_camera_x11.sh --device /dev/video2 --width 1280 --height 720
```

In the existing setup, USB Camera exposes video0/video1 and icspring exposes video2/video3.
Each pair belongs to one USB device; the verified image capture nodes are video0 and video2. Prefer `by-id` paths ending in `video-index0`.
The current first camera supports up to 1920x1080, the second up to 1280x720. The program reports requested and actual dimensions; arbitrary sizes may not be supported.
Press `Esc` or close the window to exit. Change **remote** `configs/camera.toml` fields `width` and `height` for persistent preview defaults.
Independent preview resolution does not change the Push-T camera intrinsics.

On Linux, use `ssh -X` to forward to your own X11 desktop. To view directly, run in a terminal on the **remote graphical desktop**:

```bash
cd /home/xy/Research/lerobot
.venv/bin/python scripts/camera_x11.py --device /dev/video0 --width 640 --height 480

# Save one image without a graphical desktop; the image stays on the remote host
.venv/bin/python scripts/camera_x11.py --device /dev/video2 --snapshot logs/camera-test.png
```

Exit other capture programs before opening an already occupied camera.
See [Camera preview](docs/camera.md) for device and X11 troubleshooting.

## 6. Calibration workflow

Joint angles, camera intrinsics, tabletop/base registration, fingertip height, and the resting pose solve different problems.
The included records belong to the existing experimental devices and are not universal calibrations.
Repeat the affected stages after replacing devices, changing EEPROM, or moving the base or markers.
Archive affected old files before recalibration. An existing kinematic correction blocks raw angle recapture and must be archived first.
Keep valid calibration consistent on both hosts after verifying device identity; ordinary deployment does not overwrite it.

### 6.1 Joint angles: encoders to model angles

For a new device or fresh capture, close the window and archive local raw angle calibration and its correction first.
Skip this step when retaining valid calibration for the same device:

```bash
calibration_backup="data/calibration/backups/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$calibration_backup"
for calibration_file in follower_mujoco.json follower_mujoco_kinematic.json; do
  if [ -f "data/calibration/$calibration_file" ]; then
    mv "data/calibration/$calibration_file" "$calibration_backup/"
  fi
done
```

```bash
./scripts/view_arm_realtime.sh --backend hardware
```

This entry reads feedback and captures manually placed poses. It does not execute motion or modify EEPROM.
Support the arm and turn off torque on all motors before capture:

1. Press `B` to enter calibration and `1`–`6` to select a joint.
2. Align to the 0-degree reference, hold steady for at least half a second, then press `C`.
3. Press `R` for +30 degrees, measure the angle physically, hold steady, then press `C`.
4. Press `R` for +60 degrees and capture it. This independent check allows 2 degrees of error.
5. Repeat for each joint until `Angle alignment 6/6`. Verify physical zero, positive direction, and feedback at several poses.

The result is saved **locally** as `data/calibration/follower_mujoco.json`.
Close the window, confirm the same device identity, and copy it to the remote executor:

```bash
ssh lerobot 'mkdir -p /home/xy/Research/lerobot/data/calibration'
scp data/calibration/follower_mujoco.json \
  lerobot:/home/xy/Research/lerobot/data/calibration/
```

Do not edit the fingerprint to bypass identity checks. This software alignment is separate from upstream LeRobot EEPROM calibration.
Use the upstream process only when motor limits or offsets need rebuilding; see [Angle calibration](docs/angle-calibration.md).
After raw angle recapture, also archive the remote correction tied to the old calibration, then repeat height correction and Reset teaching.
Do not mix old `follower_mujoco_kinematic.json` or `follower_rest.json` with new raw calibration.

### 6.2 Camera intrinsics and tabletop geometry

Tabletop perception reads **remote** `configs/tabletop.json`. Key fields include `camera_matrix`, `dist_coeffs`,
`image_size`, `capture_size`, marker dictionaries/IDs/measured black border sizes, and T dimensions.
There is currently no automatic camera intrinsic capture wizard. For a new camera, capture a checkerboard with known square size from several angles,
follow the [OpenCV calibration procedure](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html),
check reprojection error, then populate these fields.
Intrinsics must match the processed image size. Scaling requires scaling focal lengths and principal points; cropping also changes the principal point. Changing width and height alone is insufficient.

The current tabletop anchor is AprilTag 36h10 / ID 1, with a 150 mm black border, plus an auxiliary marker.
For a new scene, verify dictionary, ID, dimensions, fixed layout, and actual object geometry. See [Tabletop perception](docs/tabletop.md).

### 6.3 Register the robot base to the tabletop

Measure the base marker corners and installation relationship in `configs/tabletop.json` for your actual setup.
The current left black follower uses ArUco 4x4 / ID 7; the right white arm (`arm1`) uses ID 6.
Both black marker borders are 42 mm square, parallel to the table, centered laterally,
69 mm above the table and 19.5 mm forward of the base rear edge.
Make the primary anchor and both base markers clearly visible, then run:

```bash
./scripts/view_tabletop_realtime.sh --recalibrate-base --fps 5
```

The program archives each previous base calibration, accumulates 20 valid observations per arm,
checks consistency, and saves remote `data/calibration/follower_table_base.json` and
`data/calibration/arm1_table_base.json`. Copy both completed files back locally.
The live stream exposes `base_registrations` and `base_distances` in one tabletop frame.
Only the follower has joint feedback/control; the white arm registration supplies its mounting pose.
The local viewer includes both six-joint models. White-arm joints use a translucent,
unmeasured preview until separate joint calibration and feedback are available.
For camera observation alongside a controller, use
`./scripts/view_tabletop_realtime.sh --camera-only --fps 5`; this does not open a motor bus.
Recalibrate after moving the base or primary anchor. After moving the camera, check intrinsics, localization quality, and configured marker search regions.
Before replacing the follower base file, archive any height correction and samples bound to it,
then collect and validate new height measurements. See [Two-arm calibration](docs/two-arm-calibration.md)
and [Base registration](docs/push-t-remote.md) for coordinates and acceptance conditions.

For moving-marker measurements, a printable wrist marker, supervised gripper
benchmark commands and actual precision results, see
[Dual-arm simulation and precision calibration](docs/precision-calibration.md).
Base markers alone cannot measure end-effector error. The current elbow zero/range
discrepancy must be resolved before whole-arm motion for this installation.

### 6.4 Fixed fingertip height correction

After joint and base calibration, exit other serial tools. Support and manually position the arm; sampling is read-only:

```bash
./scripts/calibrate_height.sh collect --count 10
./scripts/calibrate_height.sh fit
./scripts/calibrate_height.sh apply
```

Enter the measured height in millimeters from the lowest fixed finger surface to the table, then press Enter to sample.
An empty entry means 0 mm; `q` saves and exits. Vary reach and wrist orientation. Touch the table or a rigid spacer of measured height gently; do not press the gripper against the table.
Sampling runs remotely; fitting and independent validation run locally. Only accepted candidates can be applied to both hosts.
Corrections are saved separately as `follower_mujoco_kinematic.json`; raw angle calibration is retained.
The current STS3215 setup uses the native pitch angle scale. After collecting measurements
for the current base installation, fit with:

```bash
./scripts/calibrate_height.sh fit --native-pitch-scale --apply
```

`--apply` saves a correction; it does not send motion. Resample and validate after changing raw angle or base calibration.
See [Height calibration](docs/height-calibration.md) for thresholds, resampling, and recovery.

### 6.5 Resting pose for Reset

```bash
./scripts/control_arm_realtime.sh
```

Support and place the arm in a stable resting pose. With all six motors torque-off and feedback stable, press `P` to save remote
`data/calibration/follower_rest.json`. Pressing `R` drives the arm back to that pose.
Teach the resting pose again after changing the angle mapping or kinematic correction. Copy the saved file back locally for backup.

## 7. Live tabletop and hardware demos

### Read-only mirrors and local planning previews

```bash
./scripts/view_arm_realtime.sh --backend hardware
./scripts/view_tabletop_realtime.sh --fps 10
./scripts/push_t_realtime.sh --fps 10

# Read a fresh remote image/joint snapshot, disconnect, then simulate locally
./scripts/simulate_push_t.sh --capture
```

The first three entries continuously use remote feedback; run them sequentially for the same robot.
Without `--hardware-control`, Push-T performs perception and local planning previews only.
Tabletop sessions are written to `data/tabletop/<session>/`, including images, states, models, and summaries.

### Hardware manual control, circle trajectory, and Push-T

These entries allow physical robot motion. Verify six-joint calibration, device identity, and the actual workspace first.
Push-T also requires valid tabletop/object observations and base registration.

```bash
# Manual control: E/S enables motors; arrows move the selected joint
./scripts/control_arm_realtime.sh

# Automatically Reset, approach, and execute three laps: motion starts after launch
./scripts/control_arm_realtime.sh --circle --circle-laps 3

# Headless Reset: motion starts after launch
./scripts/reset_arm_remote.sh

# Allow real Push-T: P starts; launch itself does not send motion targets
./scripts/push_t_realtime.sh --hardware-control --fps 10
```

| Manual window key | Action |
| --- | --- |
| `1`–`6` | Select joint |
| `E` / `S` | Enable all / selected joint after aligning targets to measured positions |
| Up/Down | Adjust target by +/-0.03 rad |
| `Space` | Hold current pose |
| `X` | Release selected joint; support loaded joints first |
| `R` / `P` | Return to / save resting pose |

In Push-T, `P` starts or replans, `Space` stops progression and holds, and `Esc` ends the session.
Closing a control window preserves supporting torque. Communication faults trigger hold; exiting does not mean every motor is released.
See [Hardware control](docs/real-control.md), [Realtime operation](docs/realtime.md), and [Live Push-T](docs/push-t-remote.md).
Previous V14 motion tests do not establish successful full Push-T operation on a new robot or scene.

## 8. Configuration, layout, and data

The sole active motor control configuration is [configs/control.json](configs/control.json), currently V14:
60 Hz, RAM acceleration 20, velocity 600 ticks/s, Reset/approach peak speed 0.5 rad/s, and a 4 s circle period.
Manual control, Reset, circles, and Push-T share it. `--fps` sets display/observation stream rate, not motor control frequency.
After changing shared settings, stop old processes on both hosts, deploy, and restart. Hardware execution handshakes check the configuration hash.

| Path | Purpose / update method |
| --- | --- |
| `src/arm_control/` | Local clients, simulation/planning, shared logic, remote services |
| `scripts/` | User entry points; local runtime setup is centralized in `local_runtime.sh` |
| `configs/control.json` | Shared motor control configuration, automatically deployed |
| `configs/arms.toml` | Remote USB identities, roles, IDs; verify and edit remotely |
| `configs/camera.toml` | Remote independent camera preview settings |
| `configs/tabletop.json` | Intrinsics, tabletop/object geometry, base mounting; explicit `--config` deployment |
| `configs/remote-runtime.txt` | Remote file allowlist; add new remote modules here |
| `assets/so101/` | Local model, meshes, and original license; remote needs only XML metadata |
| `data/calibration/` | Device calibration, samples, resting pose; retain these |
| `data/trajectories/circle.json` | Circle input; retain it |
| `data/push_t/measured/` | Included local offline demo/test snapshot |
| `data/tabletop/static_map.json` | Persistent auxiliary marker registration; retain it |
| `data/push_t/`, `data/tabletop/`, `logs/` | Generated runtime outputs, ignored by default |
| `.cleanup/` | Historical recovery snapshots, unused at runtime |

Do not delete the entire data directory: calibration, trajectories, and offline snapshots are inputs.
Remove old per-frame sessions only after stopping their processes and saving required results elsewhere.
See [Data retention](data/README.md).

### Git management

The complete source repository is [shawLyu/LeRobot](https://github.com/shawLyu/LeRobot), on branch `main`:

```bash
git clone git@github.com:shawLyu/LeRobot.git
cd LeRobot
```

Commit source, shared configuration, documentation, accepted calibration and required demo inputs.
`.gitignore` excludes environments, caches, logs, raw measurement sessions, backups,
local secrets and generated scenes. Canonical model meshes are tracked once;
offline demos generate their own copies. Device configurations and accepted
calibrations describe the existing installation and must be checked before hardware use.
Deploy the remote runtime from the same source version.
Treat the remote host as a runtime copy. Bring remote edits back into the main local repository before deployment to avoid maintaining divergent codebases.
If the remote host also needs Git metadata, clone the same repository, optionally use sparse-checkout, and pin the same commit.
Do not run `reset --hard` or `clean` against the existing runtime directory without preserving its calibration and outputs.
The current deployment script uses rsync and does not create a remote Git checkout.

## 9. Development and verification

Tests use simulated feedback or offline inputs. The pytest commands below do not send real motor targets.
The lightweight local environment lacks LeRobot drivers, so driver safety tests run remotely:

```bash
# Local: install development tools into the selected environment first
uv pip install --python .venv/bin/python 'pytest>=8.1,<10' 'ruff>=0.14,<1'
PYTHONPATH=src .venv/bin/python -m pytest -q --ignore=tests/test_safety.py
.venv/bin/ruff check src tests scripts

# Remote: run all tests in the deployed runtime manifest
ssh lerobot 'cd /home/xy/Research/lerobot && ./scripts/uv run --locked pytest -q'
```

Verify both hosts when changing shared code. Update `pyproject.toml` and `uv.lock` for remote dependencies,
and `requirements-local.txt` for lightweight local dependencies.
`tests/test_runtime_split.py` checks that the remote manifest does not depend on MuJoCo.
See [Code layout and shared control](docs/code-layout.md).

## 10. Troubleshooting

| Symptom | Check / remedy |
| --- | --- |
| Missing Python, mujoco, or cv2 | Install section 1; check the environment selected by `LEROBOT_MUJOCO_ENV` |
| SSH, path, or remote module error | Check the alias and `--remote-root`; bootstrap/sync after deployment |
| Serial path missing | Check power, USB, and by-id; do not substitute a different device identity |
| Serial port busy | Exit other arm, calibration, or Reset tools |
| Fingerprint / calibration mismatch | Check device, EEPROM, model, and calibration provenance; archive invalid records and recalibrate |
| `Remote SO101 is not ready for hardware control` | Read the attached serial/feedback error; restore device feedback before hardware mode |
| Bare `'fingerprint'` KeyError in old code | Deploy current code and restart; it reports the actual initialization error |
| Control configuration hash mismatch | Deploy `configs/control.json` and code; restart old processes |
| Camera blank / Permission denied | Check capture node, video group, and ownership; on Mac check XQuartz / DISPLAY |
| Resolution unchanged | Inspect actual capture dimensions, choose a supported size, and restart |
| Tabletop or T not localized | Check intrinsics, marker dimensions/dictionaries, lighting, color thresholds, visibility, and configuration |
| Height fit rejected | Collect varied poses and reliable height measurements; do not apply rejected candidates |
| Invalid Reset pose | Check calibration identity/mapping; teach a new rest pose after correction |

This README and operation guides describe current usage. `verification.md`, `control-performance.md`, and history documents
preserve experiments and their results at the time; full per-frame data from historical sessions may already have been removed.
