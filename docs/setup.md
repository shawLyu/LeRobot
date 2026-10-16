# Environment, devices, and diagnostics

See [README](../README.md) for the complete onboarding workflow.
Maintain all source locally; remote Linux runs hardware services.

## Two environments

| Location | Installation | Contents |
| --- | --- | --- |
| Local macOS / Linux | `uv venv --python 3.12 .venv`, then install `requirements-local.txt` | MuJoCo, NumPy, OpenCV, Pillow; LeRobot/Torch not required |
| Remote Linux x86_64 | `./scripts/bootstrap.sh` | Python 3.12, locked LeRobot 0.6.1 / Feetech, CPU Torch, development tools; no MuJoCo |

`scripts/lerobot-arm` selects the local interpreter and sets an absolute PYTHONPATH.
`LEROBOT_MUJOCO_ENV` takes precedence, followed by project `.venv`. An optional
Mac fallback belongs in ignored `configs/runtime.local.json`; public defaults
contain no personal environment path.
Do not use the Linux x86_64 `uv.lock` to install a Mac environment.

Remote `scripts/uv` keeps cache, Python downloads, and environment inside the project.
`scripts/install_uv.py` supports Linux x86_64 only and verifies the pinned uv installer SHA256.
`.python-version` specifies 3.12.3, preferring a matching system interpreter and downloading under project tools if necessary.

## Routine remote operation

```bash
ssh robot-host
cd ~/lerobot
./scripts/uv sync --locked --extra hardware
./scripts/uv pip check
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control hardware doctor
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control hardware read --arm follower --samples 3
```

`source scripts/activate.sh` activates only the current terminal without changing global shell configuration.
With a complete retained cache, use `./scripts/uv sync --locked --extra hardware --offline` on restricted networks.
Dependency updates require pyproject changes, lock/sync, and committing the lock; temporary pip installs are not persistent declarations.
Do not install the optional `simulation` extra remotely: simulation is a local responsibility.

## Device configuration

Remote `configs/arms.toml` stores roles, stable USB paths, IDs, baud rate, and model.
Assign a verified port and role for each installed arm; use the unassigned template
in `configs/examples/arms.toml` when configuring a new host.
Check actual `/dev/serial/by-id/` paths rather than historical ttyACM numbering.
The template uses IDs 1–6 and STS3215; verify mappings on each new installation.

`doctor`/`read` connect read-only through FeetechMotorsBus and disconnect with `disable_torque=False`.
They do not invoke high-level automatic robot configuration, change PID/EEPROM, or calibrate automatically.
Raw encoder ticks are not model radians or gripper opening percentages.

Check dialout for serial permissions and video for cameras; log in again after membership changes.
X11 camera preview needs remote system Tk, xauth, and sshd X11 forwarding; Mac uses XQuartz.

## Motion and calibration

Normally use the [Hardware window](real-control.md) after [Model angle alignment](angle-calibration.md).
Upstream `lerobot-calibrate` rebuilds EEPROM travel and does not replace model angle alignment.
Close other serial tools, verify the actual port, and follow [Official SO-101 instructions](https://huggingface.co/docs/lerobot/en/so101#calibrate).
Do not rerun motor-ID setup merely to connect a bus that already has IDs 1–6.

Low-level `python -m lerobot_arm_control hardware move` remains a bounded gripper diagnostic, with preview as default.
Only explicit `--execute --confirm-clear` executes the follower ID 6 test of 16–32 ticks.
It is not a full-arm demo. Current enable/hold behavior is documented in
[Hardware control](real-control.md).
