# System structure and operation

This SO-101 system runs from source. LeRobot supplies the Feetech motor bus;
this repository contains the control, calibration, perception and task logic.

## Structure

```text
src/lerobot_arm_control/
├── core/             device configuration, paths, process commands
├── robot/so101/      bus adapter and canonical model metadata
├── control/          manual control, feedback, joint paths and parking
├── calibration/      joint, camera, height, marker and table calibration
├── perception/       table registration, object detection and tracking
├── planning/         local MuJoCo planning and collision verification
├── tasks/            circle, Push-T and pick/place orchestration
├── transport/        shared protocols, acknowledgements and display stream
├── runtime/          local clients and remote workers
├── visualization/    views, reconstructed scenes and appearance
├── diagnostics/      motor diagnostics and precision checks
├── cli.py            unified command routing
└── __main__.py       python -m lerobot_arm_control
```

Local simulation and planners never run on the motor host. The selected remote
source is listed in `configs/remote-runtime.txt`; import closure and execution in
a separate deployment directory are covered by tests.

## Environments and commands

Install the local Python environment with `requirements-local.txt`, as described
in the [README](../README.md). The local launcher selects
`LEROBOT_MUJOCO_ENV`, then the checkout's `.venv`, then the existing Mac
workstation environment. It sets an absolute `PYTHONPATH` for this source tree.

```bash
./scripts/lerobot-arm --help
./scripts/lerobot-arm view arm --headless --duration 3
./scripts/lerobot-arm task circle --headless --duration 3
./scripts/lerobot-arm task push-t --headless --fast --limit 1 --output /tmp/push-demo
./scripts/lerobot-arm task pick-t --snapshot data/push_t/measured --output /tmp/pick-demo
./scripts/lerobot-arm calibrate camera --help
./scripts/lerobot-arm calibrate table --help
./scripts/lerobot-arm verify joints --help
```

The remote Linux x86_64 environment uses the pinned `uv.lock` and CPU PyTorch
index. `pyproject.toml` is an environment project with `package = false`.
The project is never installed into site-packages or built into a distribution.

```bash
# On the robot host:
./scripts/bootstrap.sh
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control hardware doctor
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control hardware read --arm follower --samples 3
```

Arguments go to the corresponding workflow parser. Hardware motion retains its
explicit execution flags and existing checks. `task pick-t` without a snapshot
uses the remote preview workflow; see [Pick-T](pick-t.md).

## Resources and worker startup

By default, configuration, model resources, calibration and generated data use
the checkout containing the source, independent of the current directory.
To select another prepared workspace, set `LEROBOT_ARM_WORKSPACE` before imports
or use `./scripts/lerobot-arm --workspace /absolute/path ...`.

The selected directory must already exist and provide every required resource.
Missing configuration or models raise an error with the exact missing path.
No resource is silently borrowed from a second workspace, and startup does not
copy physical calibration or create device assignments.

Controllers load configuration at startup; restart processes after selecting a
different workspace or changing control settings. Local subprocesses explicitly
receive the current source tree. SSH commands quote project paths and arguments,
set `PYTHONPATH=src`, and select the remote checkout as their own workspace.

## Source APIs

```python
from lerobot_arm_control import ArmConfig, JOINTS, load_arms, resource_path
from lerobot_arm_control.robot.so101 import connected, joint_ranges, read_state
from lerobot_arm_control.control import ManualControl, PositionFeedback, JointPath, Parking
from lerobot_arm_control.calibration import AngleCalibration
from lerobot_arm_control.perception import Perception, TTracker, BaseRegistration
from lerobot_arm_control.tasks.circle import Circle
from lerobot_arm_control.planning import PushSimulation, PushPlanner, PickPlanner
from lerobot_arm_control.runtime import Link, TableLink
```

Domain imports load implementations on demand. Importing the root namespace or
requesting hardware help does not load LeRobot, PyTorch, MuJoCo or OpenCV and
never opens a serial port. The bus adapter preserves serial ownership, checks
STS3215 identity and disconnects with `disable_torque=False`.

## Deployment and verification

`scripts/deploy_code.sh --dry-run` previews the runtime transfer. An actual
deployment first archives existing code under remote `.cleanup/`, transfers
canonical source and selected tests, then retires obsolete interface files.
Existing calibration, logs, environments and machine configuration are preserved.
A fresh deployment seeds an unassigned USB device template. Configure the actual
devices before motor operations.

```bash
# Add test tools to the local simulation environment.
uv pip install --python .venv/bin/python 'pytest>=8.1,<10' 'ruff>=0.14,<1'
PYTHONPATH=src .venv/bin/python -m pytest -q
./scripts/deploy_code.sh --dry-run
```

Tests exercise configuration isolation, dependency boundaries, worker routing,
remote import closure, protocol checks, control behavior and local collision
planning. Offline verification does not operate robot hardware.
