# Control performance and verification

The sole current configuration is `configs/control.json`, named `measured-v22-responsive`.
All hardware entries load it on restart. Local simulation checks and analysis remain local;
the remote host executes the controller without MuJoCo.

## Five-joint response comparison (2026-10-09)

Supervised physical trials used the same raw waypoints and three-second dwells.
All five arm joints moved together through progressively larger excursions and returns;
maximum per-joint changes were approximately 22, 14, 23, 15 and 29 degrees.
The jaw was not enabled or commanded. Preparation to the common anchor is excluded
from the following metrics. Each plan passed 1,407 local self/floor collision samples
and remote physical-limit and fresh-start checks before execution.

| Configuration / trial | Planned moving time (s) | Moving model RMS / max (mm) | Settled model RMS / max (mm) |
| --- | --- | --- | --- |
| V14 baseline | 6.69 | 5.56 / 15.59 | 1.82 / 3.45 |
| V19: faster trajectory only | 4.26 | 10.51 / 21.02 | 1.60 / 3.04 |
| V21: acceleration and compensation | 4.26 | 8.39 / 19.65 | 1.51 / 2.29 |
| V22: locked session P, first trial | 4.26 | 3.92 / 10.23 | 0.95 / 2.64 |
| V22: independent repeat | 4.26 | 4.05 / 10.73 | 0.65 / 2.87 |

V22 reduced planned moving time by 36.4%. The largest excursion's slowest joint
reached 90% displacement in 0.886 seconds, versus 1.454 seconds for V14, a 39.1%
reduction measured from encoder feedback. Three-second dwells were unchanged;
the total trial wall time is not reduced by 36.4%. Faster trajectory timing alone
increased dynamic error, so V19 was not selected.

V22 uses 60 Hz execution, approach peak speed 0.8 rad/s, minimum duration 0.35 s,
RAM acceleration 40 and velocity 600 ticks/s. First-five-joint feedback filtering
is 80 ms and correction slew is 0.24 rad/s; velocity lead is 0.10 / 0.14 / 0.10 /
0.10 / 0.12 seconds. Session P coefficients are 24 / 24 / 32 / 24 / 24,
written with Lock=1 and restored on close/release. Native I=0, D=32, EEPROM
limits, homing offsets, phase, operating mode and protections are preserved.
Reset retains 0.5 rad/s; circle period and ramp retain 4 s and 1 s.
Postflight readback verified original P=16, Lock=1, restored RAM acceleration
and velocity, unchanged calibration fingerprint registers and zero servo faults.
Temperatures were 26–28 degrees C. Supporting torque remains on for the first five
joints; the jaw remains unpowered.

Errors compare nominal CAD marker positions computed from target and measured
encoder angles. Settled windows cover the last 0.8 seconds of each dwell.
These trials establish response/tracking improvement on this path, not absolute
optical accuracy or a workspace-wide 1 mm claim. Camera observations and latency
are recorded separately; image latency alone does not stop this encoder benchmark.
Motor freshness, connection, fault, tracking and heartbeat guards remain active.
Nominal self/floor checks do not cover unmodeled obstacles. Recorded wrist-roll
angles outside the canonical CAD range are evaluated without clamping; physical
encoder limits are unchanged. Base and tool registration remain unvalidated as
described in [Precision calibration](precision-calibration.md).

```sh
# Physical five-joint motion; close other serial tools and supervise the workspace.
./scripts/lerobot-arm verify response --execute
# Smaller simultaneous excursion only.
./scripts/lerobot-arm verify response --execute --amplitudes 64
```

The default path is relative to the current stable pose. `--anchor path/to/anchor.json`
accepts a saved six-joint raw pose for repeat comparisons, subject to fresh-start,
limit and local collision checks. Evidence is under `logs/response-tuning-20261009/`:
`baseline-v14-retry`, `trajectory-v19`, `feedforward-v21`, `locked-p-v22`,
`confirmed-v22`, `comparison.json` and `restore-proof.json`. Interrupted V20 captures
are excluded. Current control SHA256:
`5c5eb4e26ac58e2ab0c317dc80ef5efebd797b55ad0f52cda28ecfc716319591`.

## Historical circle measurements (2026-10-06)

The circle measurements below used V14: Reset/approach 0.5 rad/s and RAM acceleration 20.
New supervised
gripper-only trials on 2026-10-07 are reported in
[Precision calibration](precision-calibration.md); whole-arm circles have not
been repeated for the new mounting calibration. V14 was restored after the earlier
gripper-only candidate; subsequent five-joint experiments are reported separately.

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
./scripts/lerobot-arm view arm --backend hardware --hardware-control
./scripts/lerobot-arm view arm --backend hardware --hardware-control --circle --circle-laps 3
./scripts/lerobot-arm view arm --backend mujoco --circle --circle-laps 3

# Explicit physical verification: executes motion. Close other serial tools first.
./scripts/lerobot-arm verify tracking --execute --laps 3
```

The launcher selects the configured local simulation interpreter.
`diagnostics/circle_tracking.py` uses current shared settings, retains full-rate trajectories and error evidence,
and rejects separate tuning files. Ordinary tracking error does not stop motion; faults, lost torque, and disconnection still trigger hold.
See [Code layout](code-layout.md) for deployment and parameter changes.
