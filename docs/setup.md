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

Remote `configs/robot.json` stores roles, stable USB paths, IDs, baud rate, and model.
Configure each installed arm under `arms`, or use private `robot.local.json` USB
port overrides. The recorded hardware baseline is installation-specific; verify
motor identity and calibration before using it on different hardware.
Check actual `/dev/serial/by-id/` paths rather than historical ttyACM numbering.

`doctor` checks environment/device access. `read` uses the same checked connection
as motion functions: saved parameter drift is restored only while torque is off
and the arm is stationary. Disconnect always uses `disable_torque=False`.
No automatic motion targets or torque-enable commands are sent.
See [Robot configuration](robot-config.md) for the power-up guard and capture workflow.
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
