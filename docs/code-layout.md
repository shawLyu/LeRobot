# Code layout and shared control parameters

Existing local source root: `/Users/shawn/Research/RobotSimulator/LeRobot`.
Remote runtime root: `lerobot:/home/xy/Research/lerobot`.
Shared control source, remote tests, and settings are deployed from the local copy.
Simulation, planning, and display source remain local.

The sole active configuration is `configs/control.json`, using V22 responsive after
two successful five-joint speed/error comparisons against V14:

| Section | Contents |
| --- | --- |
| `native_defaults` | Native LeRobot 0.6.1 settings, 60 Hz, two read retries |
| `position_feedback` | Per-joint PI and velocity/friction compensation; first-five error filter 80 ms and correction rate 0.24 rad/s; RAM acceleration 40, velocity 600 ticks/s; locked session P 24/32 restored on exit |
| `trajectory` | Approach peak 0.8 rad/s, minimum approach 0.35 s; Reset 0.5 rad/s; circle ramp 1 s, period 4 s |

`ManualControl`, `PushMotor`, headless Reset, window control, Push-T, and T pick/place load this file by default.
`joint_path.py` evaluates locally checked finite five-joint paths on the remote
control clock. `./scripts/lerobot-arm verify response` performs local collision checks and
records speed/tracking comparisons; this script and its MuJoCo analysis stay local.
Local MuJoCo circle playback shares nominal trajectory timing with remote hardware; simulation does not apply hardware feedback compensation.
Task contact speed, vision thresholds, and display frame rate remain task parameters, distinct from motor frequency.

Controllers reject separate feedback/trajectory parameter files, period overrides, and native-baseline overrides.
Edit `configs/control.json`, deploy, then restart both hosts. Historical configurations were moved to
`.cleanup/20261007-before-interface-removal/historical-configs/`; active code never reads this directory.
Hardware windows and Push-T compare the shared SHA-256 during connection. Mismatch prevents normal control.
Configuration loads at startup; no hot reload occurs during motion.

```sh
./scripts/deploy_code.sh --dry-run
./scripts/deploy_code.sh
```

`configs/remote-runtime.txt` lists motor/camera services, pure trajectory execution, read-only sampling, and their tests.
Deployment sends only listed source, scripts, tests, XML metadata, and circle input, plus docs, shared settings, and locks.
Simulation clients, MuJoCo meshes/materials, and planning modules are not sent.
Remote dependencies exclude MuJoCo; local installation uses `requirements-local.txt` or an existing environment.
Install the Linux x86_64 lock and Mac local environment separately; see [README](../README.md).

Incremental sync preserves remote-only files. Verified retired simulation source, tests, resources, and the MuJoCo installation
were archived remotely under `.cleanup/20261007-local-simulation/`.
Existing `arms.toml`, `camera.toml`, `tabletop.json`, calibration, and tabletop registration are preserved.
Experiment-output cleanup is separate. Every actual deployment snapshots existing remote code under `.cleanup/`.
First deployment creates directories and seeds missing device configuration; only explicit `--config` replaces an existing `tabletop.json`.
Use `LEROBOT_HOST` and `LEROBOT_REMOTE_ROOT` to override the deployment host and project path.

| Path | Purpose |
| --- | --- |
| `src/lerobot_arm_control/` | System modules grouped by hardware, control, calibration, perception, planning and runtime |
| `scripts/` | User entries, deployment, verification |
| `configs/control.json` | Sole active motor configuration |
| `assets/so101/` | Model/meshes with original license |
| `assets/tabletop/` | Generated reconstructed scene/appearance and mesh copies; only the marker test input is tracked |
| `data/calibration/` | Joint, resting-pose, base, and related measurements |
| `data/push_t/measured/` | Independent local offline/test observation |
| `data/tabletop/static_map.json` | Persistent auxiliary marker registration |
| `data/push_t/`, `data/tabletop/`, `logs/` | Generated experiments and evidence, ignored by default |
| `.cleanup/20261007-before/` | Pre-cleanup source/configuration snapshot |

The complete codebase is versioned at `git@github.com:shawLyu/LeRobot.git`.
The local checkout is the source of changes; commit code, shared control settings,
accepted calibration, tests, documentation and canonical models here. The remote
host receives the selected runtime through `scripts/deploy_code.sh`; it does not
need a full simulation checkout. Do not run `git clean -fdx`: ignored calibration
measurements, experiment data and recovery snapshots may still be needed locally.

```sh
git clone git@github.com:shawLyu/LeRobot.git
cd LeRobot
# Edit and validate locally, then record the change.
git add src scripts tests docs configs README.md
git commit -m "Describe the control or simulation change"
git push origin main
./scripts/deploy_code.sh
```

Generated scenes, copied meshes, logs, datasets, caches, local secrets and backup
snapshots are ignored. The offline Push-T sample retains its state, calibration
and textures; its mesh copies are generated from `assets/so101/assets/` when absent.
Caches and macOS `._*` metadata are reproducible and excluded from deployment.
On 2026-10-07, old per-frame images, traces, and duplicate session models were removed on both hosts.
Small JSON summaries and deletion manifests remain in each host's `logs/data-cleanup-20261007/`.
Required calibration, circle input, registration, and the local independent snapshot were retained.
See [Data retention](../data/README.md).

Camera intrinsics are versioned in `configs/camera_intrinsics.json`; the shared
vision configuration validates their hash and the exact processing-view mapping.
`table_reference_calibration.py` and `./scripts/lerobot-arm calibrate table` run
locally to fit the stationary two-marker layout from camera-only observations.
Only the accepted record and runtime map are synchronized; this fitter is not
included in the remote runtime manifest. Saved maps bind to the native intrinsics
hash, and fixed bases also bind to the table reference and runtime camera geometry.

`pick_t.py` performs local grasp IK, collision checks, and free-payload contact
simulation. `pick_t_client.py` sequences live stages and verifies placement from
fresh camera observations. `pick_t_protocol.py` is shared with the remote bounded
executor and has no physics dependency. See [T pick and place](pick-t.md).

## Source execution

Canonical implementations live in `src/lerobot_arm_control/`. The unified entry
is `./scripts/lerobot-arm`, or `PYTHONPATH=src python -m lerobot_arm_control`.
Domain imports load requested implementations on demand; basic configuration,
protocols and hardware help do not require a physics engine or open a serial port.

`core/paths.py` resolves the checkout and an optional explicit robot workspace.
Every resource is read from that workspace, with no fallback to another robot's
configuration. Settings load at startup; changing the workspace or control file
requires restarting the processes.

`pyproject.toml` manages the Linux dependency environment with
`[tool.uv] package = false`. Bootstrap selects the `hardware` dependencies;
local dependencies are in `requirements-local.txt`. No project installation or
build step is required. Local subprocesses receive an absolute source path.
SSH workers explicitly use the remote `src` and remote workspace.

Deployment archives existing remote code, synchronizes the selected canonical
runtime, and retires obsolete interface files after all transfers succeed.
Existing machine configuration and calibration are preserved.
See [System structure and operation](system.md).
