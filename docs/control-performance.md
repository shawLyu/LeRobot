# Control performance and verification

All normal demos use the shared configuration in `configs/control.json`.
The current controller favors smoothness and reduced overshoot, with no predictive
velocity lead. The remote computer owns real following; the simulator reads telemetry.
See [the controller and block diagram](lerobot-control.md).

## Current evidence and limits

The [smoothness report](measurements/follow-smoothing-2026-10-13/README.md) contains
recorded-input replay, simulated responses and physical six-motor response curves.
The final bounded 3° step test's worst overshoot was 8.26%; its narrow acceptance
also required tail peak-to-peak ≤0.25° and ±0.5° settling ≤1 s.

This evidence covers the exact configuration recorded in that report. The later
contact/loaded-stall guards and wrist/gripper command envelopes have passed offline
regression; physical clamp/contact and fault recovery checks remain pending.
Finite steps do not establish uninterrupted six-axis following or a universal
<100 ms latency bound. The user accepted moderate additional delay for smoothness.
Independent optical 1 mm Cartesian validation also remains pending; see
[Precision calibration](precision-calibration.md).

The latest loaded gripper can enter contact position hold. A confirmed motor
alarm latches motion while monitoring remains online; recovery requires healthy
readings and explicit resume. These protection changes are detailed in the
[controller guide](lerobot-control.md), not a new physical performance claim.

## Retained comparison reports

| Report | Purpose |
| --- | --- |
| [Initial PID](measurements/pid-2026-10-12/hardware-summary.json) | Original six-channel step baseline |
| [Latency](measurements/follow-latency-2026-10-12/README.md) | Earlier normal-speed latency captures and definition |
| [Remote following](measurements/remote-follow-2026-10-12/README.md) | Remote independence, reversal and overshoot comparison |
| [Wrist retuning](measurements/follow-retune-2026-10-13/README.md) | Normal-speed/stress separation and warning evidence |
| [Smoothness](measurements/follow-smoothing-2026-10-13/README.md) | Current reference shaping, comparison curves and pending checks |

These versioned records retain exact configurations, summaries and compressed
samples.

## Reproduce response measurement

```sh
./scripts/lerobot-arm verify motors --backend mujoco --output logs/new-simulation-response
./scripts/lerobot-arm verify motors --backend hardware --execute --output logs/new-hardware-response
./scripts/lerobot-arm verify follow-latency --help
```

Hardware execution requires an inspected pose, clear motion range and supervision.
Reports include per-motor reference, command and measured encoder curves, acceptance
metrics, actual sample gaps and native parameter restoration. A simulation pass
cannot replace hardware evidence; encoder performance cannot replace optical accuracy.
