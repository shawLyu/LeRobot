# LeRobot Arm Control

SO-101 joint control, calibration, leader/follower teleoperation, camera/tabletop
perception, local MuJoCo simulation, circle trajectories, Push-T and T pick/place.
The project runs directly from source; no package installation or frontend build
is required.

Keep the complete checkout on the local computer. Simulation, inverse kinematics,
collision checking and planning run locally. A Linux robot host owns the serial
buses, cameras, feedback and physical control loops. Leader/follower control runs
entirely on that host; an optional simulator subscribes to its telemetry.

All account names, host addresses and paths below are **examples to replace with
your own settings**. Hardware identities and calibrations belong to individual
installations. The included measured scene is an offline demo input, not a
calibration for a newly connected robot.

## 1. Get the source and install locally

Copy the repository URL from your Git hosting service:

```bash
git clone YOUR_REPOSITORY_URL LeRobot
cd LeRobot
```

Use macOS or Linux with Python 3.12 and [uv](https://docs.astral.sh/uv/):

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements-local.txt
.venv/bin/python -c 'import mujoco, numpy, cv2, PIL; print("local runtime ready")'
./scripts/lerobot-arm --help
```

`requirements-local.txt` provides the local simulation and display dependencies.
`pyproject.toml` and `uv.lock` describe the separate Linux x86_64 hardware runtime.
Install that runtime on the robot host using section 4.

The launcher prefers `LEROBOT_MUJOCO_ENV`, followed by the checkout's `.venv`.
To use an existing environment, set its path in your own terminal:

```bash
export LEROBOT_MUJOCO_ENV=/absolute/path/to/your/.venv
```

An optional macOS fallback can be set privately as `local.fallback_environment`
in section 3; the public configuration has no personal environment path.

## 2. Check local simulation first

These commands require no robot, SSH connection or camera:

```bash
./scripts/lerobot-arm view arm --backend mujoco
./scripts/lerobot-arm view arm --backend mujoco --headless --duration 3
./scripts/lerobot-arm task circle --headless --duration 3
./scripts/lerobot-arm task push-t --headless --fast --limit 1 --output data/push_t/my-demo
./scripts/lerobot-arm view follow --source synthetic --auto
```

In the arm viewer, `1`–`6` select a joint, Up/Down adjust its simulated target,
`Space` holds and `Esc` exits. Specify `--backend mujoco` for offline arm viewing;
the arm viewer's default backend is hardware.

Push-T uses the included `data/push_t/measured/` observation snapshot. Use a new
`--output` directory to retain a separate run. Reaching `--limit` is a stopping
condition, not proof of task success. See [Push-T](docs/push-t.md).

## 3. Configure your own SSH account and project path

Create a host entry in your local `~/.ssh/config`, replacing both placeholders:

```sshconfig
Host robot-host
    HostName YOUR_ROBOT_HOSTNAME_OR_IP
    User YOUR_LINUX_ACCOUNT
```

Keep SSH keys and credentials in your SSH configuration and agent. Confirm that
`ssh robot-host` reaches the intended account before deploying.

`configs/runtime.json` contains generic shared defaults. On each computer, create
an untracked override and edit it for that computer:

```bash
# First-time setup only; preserve an existing private override.
cp configs/runtime.json configs/runtime.local.json
chmod 600 configs/runtime.local.json
```

Example `configs/runtime.local.json`:

```json
{
  "version": 1,
  "remote": {
    "host": "robot-host",
    "root": "/home/YOUR_LINUX_ACCOUNT/lerobot"
  },
  "local": {
    "fallback_environment": null
  }
}
```

Set `root` to the **absolute Linux project path**, not a local computer's path.
The deployment script currently accepts normalized destinations under `/home/`.
The example remote commands below assume that directory is `~/lerobot`.

Connection setting priority is:

**Explicit `--host` / `--remote-root` → environment variables →
`configs/runtime.local.json` → `configs/runtime.json`.**

Environment overrides also work for deployment and camera wrappers:

```bash
export LEROBOT_HOST=robot-host
export LEROBOT_REMOTE_ROOT=/home/YOUR_LINUX_ACCOUNT/lerobot
```

The private override is loaded only from the selected robot workspace. Git ignores
`configs/*.local.*`, and deployment excludes them. Each account maintains its own
copy; deployment does not copy one account's private connection settings to another.

## 4. Deploy and install on the robot host

The hardware runtime uses Linux x86_64, Python 3.12 and the locked LeRobot/Feetech
dependencies. From the local checkout:

```bash
./scripts/deploy_code.sh
# For later deployments, preview changes first if desired:
./scripts/deploy_code.sh --dry-run
```

The first deployment creates the destination. A dry-run on a fresh host requires
that destination to exist already. The script copies the files listed in
[configs/remote-runtime.txt](configs/remote-runtime.txt), shared configuration,
dependency locks and documentation. It preserves existing device settings,
calibration, logs and remote-only files.

Then run on the **remote host**, using your own project directory:

```bash
ssh robot-host
cd ~/lerobot
./scripts/bootstrap.sh
./scripts/uv pip check
```

For subsequent dependency updates, use
`./scripts/uv sync --locked --extra hardware` from that remote directory.
Remote simulation dependencies are unnecessary. See [Setup](docs/setup.md).

## 5. Configure and read your devices

On the remote host, enumerate the actual devices:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-id/
```

A fresh deployment seeds an unassigned arm template. Edit remote
`configs/arms.toml` using [configs/examples/arms.toml](configs/examples/arms.toml):
assign each arm's verified USB path, role, model, baud rate and motor IDs.
Use `follower` for the controlled arm and, for a two-arm installation, `arm1`
with role `leader` for the acquisition arm. Copy the same verified arm configuration
to the local workspace used by viewers. Match channel names and IDs to your wiring;
the template's ID order is only an example.

After configuration, these commands inspect devices without sending motion targets:

```bash
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control hardware doctor
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control hardware read --arm follower --samples 3
```

Only one process may own a motor bus at a time. Check serial and camera group
permissions on Linux; after changing group membership, log in again.

For a local browser interface:

```bash
./scripts/lerobot-arm view motors
```

Open **http://127.0.0.1:8765**. Select an arm, inspect up to six motors, edit native
servo parameters with readback, set a software motion range, send position targets
and record step responses. The charts include horizontal/vertical A/B cursors,
`Δt`/`Δy`, sample snapping and CSV/JSON/PNG export. Connecting does not enable torque.
See [Servo workbench](docs/servo-workbench.md) for write modes and measurement units.

For cameras, edit remote `configs/camera.toml` and use a verified capture node:

```bash
# Local terminal; macOS X11 preview needs XQuartz.
./scripts/view_camera_x11.sh
./scripts/view_camera_x11.sh --device /dev/video0 --width 640 --height 480
```

Device node numbers and supported resolutions vary. Check the reported actual
resolution. Preview settings and tabletop camera intrinsics are separate.
See [Cameras](docs/camera.md) for snapshots and camera-only capture.

## 6. Calibrate before enabling physical motion

Measure each robot's encoder-to-model angle mapping, zero, direction and usable
travel. Calibration is bound to the device identity and relevant motor registers;
renaming an arm does not make another device's calibration valid.

Use the guides appropriate to your installation:

- [Joint angle calibration](docs/angle-calibration.md) and [two-arm calibration](docs/two-arm-calibration.md).
- [Single-servo zero and native PID tests](docs/native-pid.md).
- [Camera intrinsics, table and base registration](docs/tabletop.md).
- [Height correction](docs/height-calibration.md) and [precision validation](docs/precision-calibration.md).

Teach a new resting pose for your own calibrated arm before using Reset.
Measurements for an earlier scene or motor are not transferable performance guarantees.

After calibration and checking the physical workspace, common operations are:

```bash
# Feedback-only arm viewer
./scripts/lerobot-arm view arm --backend hardware

# Enable the manual-control interface; motion uses its explicit controls
./scripts/lerobot-arm view arm --backend hardware --hardware-control

# Remote leader/follower controller; initialization and alignment can move the arm
./scripts/lerobot-arm demo follow

# Optional read-only follower simulator in another terminal
./scripts/lerobot-arm view follow
```

The follower terminal accepts `h` + Enter to hold, `e` + Enter to resume and
`q` + Enter to hold and quit. Use `--start-held` to start without automatic following.
See [Follow](docs/follow.md) and [Hardware control](docs/real-control.md).

Tabletop planning uses locally reconstructed camera observations:

```bash
./scripts/lerobot-arm view tabletop --camera-only --fps 5
./scripts/lerobot-arm view tabletop --push-t --fps 10
./scripts/lerobot-arm task pick-t
```

Physical task execution requires the documented hardware flags and accepted
calibrations. See [Live Push-T](docs/push-t-remote.md) and [T pick/place](docs/pick-t.md).

## 7. Configuration and source layout

| Path | Purpose |
| --- | --- |
| `configs/runtime.json` | Generic connection defaults; no personal account or environment |
| `configs/runtime.local.json` | Per-account connection/environment override; ignored and never deployed |
| `configs/arms.toml` | Device roles, ports and motor IDs for the selected workspace |
| `configs/control.json` | Shared outer controller, reference shaping and protection parameters |
| `configs/follow.json` | Leader/follower mapping and initialization settings |
| `configs/camera.toml`, `camera_intrinsics.json`, `tabletop.json` | Capture, intrinsics and scene geometry |
| `data/calibration/` | Device-bound measurements and resting poses |
| `data/push_t/measured/` | Included offline demo snapshot |
| `src/lerobot_arm_control/` | Control, calibration, planning, runtime, transport and workbench |
| `configs/remote-runtime.txt` | Files required by the robot host |
| `logs/` | Generated recordings and diagnostic outputs |

The source checkout is the default robot workspace. To use a separate private
workspace, set `LEROBOT_ARM_WORKSPACE` to an existing directory containing its own
complete `configs/`, `data/` and required `assets/`. Missing resources fail instead
of falling back to another robot's configuration.

Normal motion demos share one outer controller. The native servo workbench and
native PID benchmark measure the motor's built-in position loop directly.
See [Control design](docs/lerobot-control.md), [Code layout](docs/code-layout.md)
and [System operation](docs/system.md) for details.

## 8. Privacy and sharing

Keep account names, host/IP addresses, private environment paths, credentials and
keys out of shared documentation and public defaults. Use private runtime overrides,
SSH configuration or environment variables for account-specific values.

Review device configuration, calibration fingerprints, exported JSON/CSV, images
and logs before publishing: they can contain USB identifiers, absolute paths and
scene metadata. Some reference measurements are versioned in this repository;
`.gitignore` does not remove already tracked files or older Git commits. Removing
information from this README or a current configuration does not erase Git history.

Commit source, generic configuration, documentation and deliberately selected demo
inputs. Keep newly captured installation data in a private workspace unless it has
been reviewed for sharing. Deploy from the same source revision on both computers;
bring deliberate remote code changes back into the source checkout first.

## 9. Development checks

```bash
uv pip install --python .venv/bin/python 'pytest>=8.1,<10' 'ruff>=0.14,<1'
PYTHONPATH=src .venv/bin/python -m pytest -q
.venv/bin/ruff check src tests scripts
```

Tests use synthetic feedback or offline inputs. Driver-specific tests can skip
when the local environment lacks hardware dependencies; run the deployed test suite
on the robot host as well. Optional browser tests and prerequisites are documented
in [Servo workbench](docs/servo-workbench.md); hardware acceptance uses a separate
explicit flag.

For connection failures, check the selected interpreter, `ssh robot-host`, your
private runtime override, the actual device paths and process ownership. For
calibration or control-hash mismatches, inspect the stated identity/configuration
error and synchronize the intended source and measurements before resuming.
