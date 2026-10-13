# Local simulation and remote realtime control

Existing local project: `/Users/shawn/Research/RobotSimulator/LeRobot`.
Remote runtime: `lerobot:/home/xy/Research/lerobot`.

All MuJoCo simulation, collision checks, IK, contact prediction, and Push-T planning run locally.
The remote host handles camera perception, physical motor control, and feedback, receiving locally checked trajectories/results without MuJoCo.

```sh
./scripts/lerobot-arm view arm --backend mujoco
./scripts/lerobot-arm view arm --backend mujoco --circle --circle-laps 3
./scripts/lerobot-arm view arm --backend hardware --hardware-control
./scripts/lerobot-arm view arm --backend hardware --hardware-control --circle --circle-laps 3
./scripts/lerobot-arm view tabletop
./scripts/lerobot-arm view tabletop --push-t --fps 5
```

Simulation uses a separate local physics process over stdio, with no SSH or motor connection.
Hardware commands/feedback use the existing remote `.venv` over SSH stdio, without additional service ports.
Normal entries share the [sole control configuration](code-layout.md): motor control 60 Hz, circle period 16 s,
Reset speed 0.5 rad/s and approach speed 0.8 rad/s, with a 0.35 s minimum approach.
Simulation shares nominal timing; hardware adds the shared feedback compensation.

Displayed hardware feedback is about 20 Hz, independent of the 60 Hz control loop.
Fixed deadlines include serial processing; missed slots are skipped without burst catch-up. Targets are sent synchronously each frame.
Client feedback older than 0.5 s prevents control; a remote heartbeat gap over 0.75 s triggers hold.
Closing the window retains supporting torque. Fresh simulation states can maintain heartbeat independently of rendering.

Use read-only feedback and angle capture with:

```sh
./scripts/lerobot-arm view arm --backend hardware --backend hardware
```

This entry executes no motion. `B` enters calibration, `R` changes the reference, and `C` captures.
See [Angle calibration](angle-calibration.md). Hardware motion requires valid calibration for all six joints.
See [Hardware control](real-control.md) for controls and Reset. Headless Reset uses `./scripts/lerobot-arm reset`,
sending one trigger plus heartbeats rather than an angle sequence.

Install local dependencies from `requirements-local.txt`; the launcher prefers the project's `.venv`.
`LEROBOT_MUJOCO_ENV` selects another environment; Mac retains the legacy UWLab fallback if the project environment is absent.
See [README](../README.md) for installation. The existing remote account has dialout/video access.
The serial port is exclusive; do not run hardware tools concurrently.

The local model and original license are under `assets/so101/`; remote `so101.xml` is metadata for limits/calibration identity.
Before a circle, the local host checks the entire path and approach from the measured pose.
The remote host checks model, trajectory, control hashes, and entry pose; changes over 0.003 rad reject execution.
