# Verification records

## Live device checks: 2026-10-11, Beijing time

| Check | Result |
| --- | --- |
| SSH host | `lerobot`, project `/home/xy/Research/lerobot`; remote LeRobot 0.6.1 |
| Two arm connections | Current local config loader and driver loaded in memory remotely, using remote arms.toml; both buses open together, six STS3215 motors each, three state-read rounds successful |
| Motor state | All Status=0, Moving=0, Torque_Enable=0; 24–26 C; no motion, configuration writes or torque changes |
| Camera inventory | One USB Camera, video0/video1; configured stable video-index0 resolves to video0; previous icspring device absent |
| Camera capture | Existing remote preview script saved `logs/camera-check-20261011.png`, actual 640x480; backend fps setting 30 |
| Camera ownership | PipeWire held video0 open; snapshot succeeded without stopping it |
| Deployment/docs | Remote canonical tabletop worker absent; remote README/camera doc hashes differ from local; raw snapshot works, new tabletop entry awaits deployment |

Arm report: local `logs/two-arm-connection-20261011T021656Z.json`.
The camera check did not exercise the X11 window, tabletop perception or loaded
motor motion. Current camera instructions are in [Camera](camera.md).

## Script and demo cleanup: 2026-10-11, Beijing time

| Check | Result |
| --- | --- |
| Full test suite | 466 passed, 1 skipped in 51.74 s; upstream LeRobot comparison skipped locally |
| Scripts | 32 reduced to 9; 21 removed, 2 verification programs moved into diagnostics |
| Shared logic | One command acknowledgement checker, byte fingerprint function, atomic JSON writer and canonical SO101 joint order |
| Demo/model cleanup | Duplicate scene removed; 18 identical cached meshes removed (17,409,712 bytes); generated run output archived under .cleanup with hashes |
| Static and shell checks | Ruff passed; all 7 remaining shell entries passed syntax checks |
| Local runtime | Circle task launched from /tmp, started the circle and completed at about 20 Hz |
| Remote source boundary | Selected runtime import and deployment tests passed |
| Robot execution | Verification used simulation and fake buses |

## Source cleanup: 2026-10-11, Beijing time

The source-only SO-101 system was checked locally after migration to
`lerobot_arm_control` and removal of compatibility and build interfaces.

| Check | Result |
| --- | --- |
| Full test suite | 461 passed, 1 skipped in 49.64 s |
| Skipped check | Upstream LeRobot default comparison; LeRobot is absent from the local simulation environment |
| Static checks | Ruff passed for source, tests and scripts |
| Shell syntax | 21 scripts passed |
| Dependency lock | Offline `uv lock --check` passed with Python 3.12.13 |
| Local runtime | Headless MuJoCo launched from /tmp, completed the worker handshake and received about 20 Hz |
| Remote source boundary | Manifest staged into a separate directory; all listed source imported without LeRobot, PyTorch, MuJoCo or serial |
| Deployment behavior | Mock SSH/rsync verified dry run, backup ordering, failure abort, normalized paths and retirement after successful transfer |
| Robot execution | No deployment or physical motor operation was performed in this verification |

Runtime tests use the existing Python 3.11.15 local simulation environment.
Hardware feedback tests use fake buses and now run without importing LeRobot.
The remote Linux dependency versions remain pinned; only the environment project
metadata changed to source execution.

## Hardware verification: 2026-10-05, Beijing time

This is a historical record. The remote project was `/home/xy/Research/lerobot`, with files and `.venv` owned by `xy`.
Current setup instructions are in [README](../README.md).

| Check | Result |
| --- | --- |
| uv | 0.12.23 at project-local `.tools/bin/uv` |
| Python | 3.12.3 at `.venv/bin/python` |
| LeRobot | 0.6.1, imported successfully |
| Feetech SDK | 1.0.0, imported successfully |
| PyTorch / torchvision | 2.11.0+cpu / 0.26.0+cpu, imported successfully |
| Dependency consistency | 52 packages; `uv pip check` passed |
| Lock / offline sync | `uv sync --locked --offline` passed |
| Shell activation | bash and zsh both selected project Python |
| Official tools | `lerobot-calibrate --help` succeeded; calibration not executed |
| Unit tests | 24 passed, including target-write torque auto-enable, no-displacement rejection, and fault detection |
| Read-only hardware | Two buses, six STS3215 motors each, three rounds each; all 36 status reads succeeded |

The user confirmed follower USB `5B8E114461` (ttyACM0 at the time) and ID 6 as the gripper.
The role and SO100/SO101 model of USB `5B91028324` (ttyACM1 at the time) were unconfirmed.

## Motion tests and limits

Tests ran after the user confirmed an observer was present and the gripper motion area was clear.
An 8-tick action did not reliably arrive; 16 ticks produced motion, but the small return command did not reliably arrive.
A 3-tick tolerance was also stricter than the observed 4–5-tick resting error.
The final range was 16–32 ticks, default 32 ticks (about 2.8 motor-shaft degrees), with 5-tick tolerance,
three consecutive stationary samples, no faults, and sufficient actual movement required.
Starting deadband, static friction, and power were possible causes; no single cause was confirmed, and PID/EEPROM were unchanged.

**The final 32-tick round trip succeeded:**

- Starting target and feedback: 1490.
- Outbound target: 1522; feedback: 1518; actual travel: 28 ticks.
- Return target: 1490; feedback: 1495; error: 5 ticks, within tolerance.
- Gripper torque was 0 after completion, confirmed by reread.

Writing Goal_Position was found to enable gripper torque automatically.
Cleanup was changed to write torque last and reread it. When original torque was off, the target remained at the safe start
rather than restoring a potentially stale or out-of-range historical target.

A subsequent attempt to return to the initial diagnostic position 1472 produced `Input voltage error`, so motion stopped.
**This additional restoration did not complete.** The user reported a separate follower power supply;
that did not exclude load-related power or wiring problems.
A final torque-disable command and direct response checks confirmed torque 0, position 1483, Moving 0,
Status 0, raw voltage 52, and temperature 25 C, consistent over three reads.

The software environment was ready and one limited round trip had passed, but load-related power stability remained unverified.
Before further motion at that stage, the motor rating, supply current, connectors, and motion-related voltage drop needed physical inspection.
Successful stationary communication alone did not establish adequate loaded motor power.

## Reports

Remote `logs/` contains:

- `verification.json`: summary and unresolved supply issue.
- `environment.json`, `read-test.json`, `final-read.json`: environment and read checks.
- `unit-tests.xml`, `unit-tests.txt`, `lint.txt`, `dependencies.txt`: software checks.
- `gripper-motion.json`: successful 32-tick round trip.
- `gripper-motion-initial-failure.json`, `gripper-motion-8ticks-failure.json`, `gripper-motion-16ticks-strict-failure.json`: initial failures.
- `torque-goal-diagnostics.json`: measured torque auto-enable on target write.
- `final-voltage-fault.json`: torque disable and final state after voltage fault.

Failure records were retained for investigation; PID, protections, and motor IDs were not altered to force a passing result.
