# V14 control performance and verification

The sole current configuration is `configs/control.json`; all hardware entries share V14.
The physical circle performs Reset before entering the saved absolute 15 mm circle: 4 s period, 60 Hz,
Reset/approach planned peak speed 0.5 rad/s, RAM acceleration 20, velocity 600 ticks/s.
Native PID, EEPROM endpoints, protections, and measured calibration are retained.

The circle measurements below were recorded on 2026-10-06. New supervised
gripper-only trials on 2026-10-07 are reported in
[Precision calibration](precision-calibration.md); whole-arm circles have not
been repeated for the new mounting calibration. V14 remains active after a
software PI candidate failed to improve the matched-center gripper trial.

| Trial | Period / laps | Shoulder RMS / max error (deg) | Elbow RMS / max error (deg) | Model fingertip RMS / max error (mm) |
| --- | --- | --- | --- | --- |
| V14 tuning validation | 4 s / 3 | 0.30 / 1.11 | 0.35 / 1.23 | 2.75 / 8.83 |
| Standard entry recheck | 4 s / 3 | 0.303 / — | 0.345 / — | 2.94 / 7.76 |

The standard entry completed Reset, approach, and three laps, then held with supporting torque retained.
No motor fault was confirmed. The 60 Hz trace contains 780 running frames; maximum circle interval was
18.21 ms, with none exceeding 25 ms. A transient gripper Status=1 returned 0 on independent reread and was recorded as a recovery warning.
Evidence: `logs/responsive-final-client.json`, `logs/tracking-trace-responsive-final.json`, and their analysis outputs.
V14 tuning evidence: `logs/tracking-trial-1791288975334155000.json` and its corresponding tracking trace.

Errors are computed from encoders and calibrated kinematics, not optical measurements.
The experiments cover the current robot, small circle, and Reset path. Peak residual errors remain;
these results do not demonstrate agreement with simulation throughout the workspace.
Previous tuning comparisons and configurations are retained in pre-cleanup source snapshots and historical `.cleanup/` directories.

```sh
./scripts/control_arm_realtime.sh
./scripts/control_arm_realtime.sh --circle --circle-laps 3
./scripts/view_arm_realtime.sh --backend mujoco --circle --circle-laps 3

# Explicit physical verification: executes motion. Close other serial tools first.
PYTHONPATH=src ../URDFSim/.venv/bin/python scripts/verify_circle_tracking.py --execute --laps 3
```

The last command records the historical workstation environment; substitute your local simulation interpreter as needed.
`verify_circle_tracking.py` uses current shared settings, retains full-rate trajectories and error evidence,
and rejects separate tuning files. Ordinary tracking error does not stop motion; faults, lost torque, and disconnection still trigger hold.
See [Code layout](code-layout.md) for deployment and parameter changes.
