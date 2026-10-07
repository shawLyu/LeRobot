# Post-calibration gripper verification: 2026-10-06

The physical gripper responded to target writes, but neither trial met the specified 5-tick arrival criterion.
Neither was marked successful; PID, torque limits, and tolerance were unchanged.
One live feedback window remained open locally, with actual positions streamed during motion.
This document describes the state at the time; current operation is documented in [Hardware control](real-control.md).

## Results

| Trial | Start | Target | Result |
| --- | ---: | ---: | --- |
| First, near closed endpoint | 1478 | 1510, then return to 1478 | Returned to 1485, error 7 ticks; torque disabled after timeout |
| Second, after manual opening | 1962 | 1994 | Stopped at 1982, error 12 ticks; torque disabled after timeout; no return leg |

The second trial saved 55 feedback samples. Actual travel was 20 ticks, about 1.60 degrees using the completed gripper calibration;
requested travel was about 2.57 degrees. Every sampled fault status was 0; raw voltage readings were 52–52.
The earlier input-voltage fault did not recur in this small test, which does not establish supply behavior under larger loads.
Final reads confirmed Torque_Enable=0 for all six joints.

The first trial saved only the failure acknowledgement and final state, so its exact outbound peak cannot be reconstructed.
The second saved phase traces, targets, voltage, faults, torque, and final state.

## Software checks

- Only the confirmed follower ID 6 was commanded; other joints received no motion commands.
- Existing position mode, limits, and zero registers were retained.
- Fault status was checked; targets were incremented within 32 ticks, with stable feedback and sufficient actual displacement required.
- Failure or feedback loss disabled gripper torque without an unverified return leg.
- Calibrated endpoint checks were added before retry, rejecting starts near the closed endpoint.
- Each launch allowed one gripper execution request, then revoked permission.
- All 44 safety/protocol tests passed at that stage.

## Evidence and subsequent work

Local evidence: `logs/gripper-verification-attempt1-failed.json`,
`logs/gripper-verification-attempt2-failed.json`, and corresponding final-state files.
The summary is `logs/gripper-verification-summary.json`; remote logs also contain the complete second trace.
Angle backups remain in `data/calibration/backups`; calibration was not changed during these trials.

At that stage the arrow keys were read-only previews. Further investigation of following error and motor parameters was needed;
the cause was unresolved, so full-arm execution was not enabled at that time.
Do not relaunch `--verify-gripper` for unattended repeated tests.
