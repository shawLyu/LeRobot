# Historical record: before the 2026-10-07 cleanup

This document preserves installation and intermediate device states at the time.
Use [README](../../README.md) and [Current setup](../setup.md) for current operation.
The historical local-environment guidance below predates the separate lightweight Mac installation.

## Environment and directories

The project was located on remote host lerobot at `/home/xy/Research/lerobot`, operated as xy.
It uses a Python src layout: project lerobot-arm-control, module arm_control,
without shadowing upstream lerobot.

```text
lerobot/
├── pyproject.toml            # Project, dependencies, CPU PyTorch source, development tools
├── uv.lock                   # Exact dependency versions and integrity metadata
├── .python-version           # 3.12.3; remote system interpreter at the time
├── .uv-version               # uv 0.12.23
├── .venv/                    # Isolated uv environment; no system site-packages
├── .tools/bin/               # Project-local uv/uvx
├── .cache/                   # Installers and uv download/build cache
├── configs/arms.toml         # Stable USB paths, roles, motor IDs
├── src/arm_control/          # LeRobot adapter, diagnostics, bounded motion CLI
├── tests/                    # Hardware-free safety checks
├── scripts/                  # Installation, activation, read-only hardware checks
├── docs/                     # Configuration, operation, experiment records
├── data/calibration/         # Confirmed inputs tracked; candidates/backups ignored
└── logs/                    # Diagnostic and motion JSON
```

The initial installation contained required LeRobot core, Feetech, pyserial, deepdiff, and pytest/ruff.
CPU PyTorch was 2.11.0 with torchvision 0.26.0; CUDA training, dataset recording, and camera operation were not configured then.
Ubuntu's USB serial driver worked without additional kernel drivers, udev changes, or sudo.

## Initialization and routine use

```bash
ssh lerobot
cd /home/xy/Research/lerobot

# Already configured host: synchronize from the lock
./scripts/uv sync --locked

# Rebuild from source: install project-local uv, then synchronize
./scripts/bootstrap.sh

# Optional activation for the current shell; no .zshrc edits
source scripts/activate.sh
python -c 'import sys, lerobot, scservo_sdk; print(sys.executable); print(lerobot.__file__)'
uv run arm-control doctor
```

scripts/uv places caches, Python downloads, and virtual environments inside the project.
.python-version pins 3.12.3; .venv used `/usr/bin/python3.12`, with Python packages managed entirely by uv.
System Python and other accounts' environments were not changed.
If no matching system interpreter is available, uv can download one under `.tools/python/`.
Update dependencies with `./scripts/uv add PACKAGE`, or edit pyproject and run
`./scripts/uv lock && ./scripts/uv sync --locked`; retain uv.lock.
Temporary uv pip installs are not persistent configuration because later sync follows pyproject/lock.

## Ports and joints at the time

| Name | Serial node then | USB serial | Role |
| --- | --- | --- | --- |
| follower | ttyACM0 | 5B8E114461 | User-confirmed follower |
| arm1 | ttyACM1 | 5B91028324 | Unconfirmed |

Configuration uses `/dev/serial/by-id/`, independent of ttyACM numbering after reconnection.
Both buses detected IDs 1–6, model 777 (STS3215), baud 1,000,000.
Names follow upstream SO100/SO101: shoulder_pan=1, shoulder_lift=2, elbow_flex=3,
wrist_flex=4, wrist_roll=5, gripper=6.
At this stage the user had confirmed follower ID 6 as gripper; other physical mappings and precise SO100/SO101 model still required confirmation.

## Testing and bounded gripper motion

Results were recorded in docs/verification.md.
After a successful round trip, additional restoration encountered an input-voltage error;
further motion required checking motor voltage rating and loaded supply capability.

```bash
# Unit checks without hardware access
./scripts/uv run --locked pytest -q
./scripts/uv run --locked ruff check src tests scripts
./scripts/uv pip check

# Read both buses three times each
./scripts/test_hardware.sh

# Repeated follower reads
./scripts/uv run --locked arm-control --output logs/follower-read.json \
    read --arm follower --samples 10

# Gripper preview: reads only, no motor register writes
./scripts/uv run --locked arm-control --output logs/gripper-preview.json \
    move --arm follower --delta 32
```

Diagnostics use FeetechMotorsBus and disconnect explicitly with disable_torque=False.
They do not call high-level robot connect/configuration, calibrate, set PID, or unlock EEPROM.
Positions are uncalibrated raw ticks, not joint angles or gripper percentages.

The diagnostic motion tool permits only follower ID 6, displacement 16–32 ticks (about 1.4–2.8 shaft degrees), default 32.
Eight ticks did not reliably start on that hardware; 16 moved forward but did not reliably return.
Arrival requires target error within 5 ticks, Moving=0, Status=0, and three consecutive qualifying reads.
The gripper had about 4 ticks of stationary offset; PID was not changed to remove it.
Actual displacement must have the correct sign and reach at least half the requested amount, preventing no-motion success.
Operation requires position mode, a stationary start, and current/target positions inside configured limits.
Before enabling previously disabled torque, the target is aligned to the measured position to avoid stale-target jumps.
Successful arrival returns to start and restores original torque; other five motors receive no writes.
With original torque off, target remains at the safe start; with torque on, only a checked original target near start is restored.
Goal_Position writes auto-enabled torque on this gripper, so cleanup writes torque last and confirms it by readback.
Timeout/Ctrl-C attempts gripper torque-off only; communication loss cannot guarantee delivery.
Only Goal_Position and Torque_Enable are changed, not EEPROM, speed, PID, or calibration.

**Run only after confirming ID 6 is the physical gripper, the area is clear, and someone is observing:**

```bash
./scripts/uv run --locked arm-control --output logs/gripper-motion.json \
    move --arm follower --delta 32 --execute --confirm-clear
```

--confirm-clear records the operator's physical checks. Preview does not need it.

## Upstream calibration

After confirming SO100/SO101 model, use upstream calibration if needed; it may require manually moving joints.
Initial installation did not automatically calibrate. Keep output in the project, for example:

```bash
source scripts/activate.sh
uv run lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E114461-if00 \
    --robot.id=follower \
    --robot.calibration_dir="$PWD/data/calibration"
```

For SO100, use so100_follower. Check --help for the installed version.
Do not rerun lerobot-setup-motors to assign IDs; the existing buses already communicated with IDs 1–6.

## Restricted networks

The initial remote downloads from PyPI and the official PyTorch index succeeded without local relay.
Retained `.cache/uv/` supports offline resynchronization:

```bash
./scripts/uv sync --locked --offline
```

If only the uv installer cannot download, transfer its Linux wheel from another machine.
install_uv.py records the official URL and SHA256. A macOS uv executable cannot substitute for the Linux binary.

```bash
# Download the Linux x86_64 wheel specified in install_uv.py on the local machine
scp /local/path/uv-0.12.23-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl \
    lerobot:/home/xy/Research/lerobot/.cache/downloads/

# Use the transferred wheel remotely without downloading again
python3 scripts/install_uv.py --wheel \
    .cache/downloads/uv-0.12.23-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

Other transferred wheels must target Linux x86_64 / CPython 3.12 and match uv.lock versions/hashes.
A Mac environment cannot be copied directly as remote .venv.
A complete backed-up remote `.cache/uv` can be restored before offline synchronization.

## Official references

- [uv projects](https://docs.astral.sh/uv/guides/projects/)
- [uv PyTorch integration](https://docs.astral.sh/uv/guides/integration/pytorch/)
- [LeRobot installation](https://huggingface.co/docs/lerobot/installation)
- [LeRobot SO101](https://huggingface.co/docs/lerobot/so101)

## Historical simulation/deployment note

The default lock served remote camera/motor operation without MuJoCo.
The earlier guide suggested the simulation extra (`uv sync --locked --extra simulation`) or an existing environment,
and Mac wrappers defaulted to `../Materials/UWLab/.venv` with LEROBOT_MUJOCO_ENV override.
That installation guidance has been superseded by requirements-local.txt and project-.venv-first selection in the current README.
Deployment uses configs/remote-runtime.txt; simulation, planning, and display entries are not sent remotely.
