# Control performance and verification

All normal demos use the shared configuration in `configs/robot.json`.
The default controller favors smoothness and reduced overshoot, with no predictive
velocity lead. The explicit `smooth-circle-v1` free-space profile adds velocity
feedforward and reduced acceleration; it uses the same controller implementation.
The remote computer owns real following; the simulator reads telemetry.
See [the controller and block diagram](lerobot-control.md).

## Current evidence and limits

The [0/15/30-degree plane-tilt comparison](measurements/circle-tilt-2026-10-18/README.md)
keeps the 38 cm center, 8 cm radius, 120°/s profile and eight-second ramps.
Mean speeds were 120.60/120.52/120.69°/s; temporal RMS was 13.42/10.83/8.16 mm.
The operator accepted the two inclined cases' motion and starts/stops. Remaining
speed ripple and the lack of independent optical precision are reported explicitly.
No PID change was made; geometric validation and response scoring now share the
validated 3D circle frame instead of treating a tilted path as an axis-aligned circle.

The [120°/s experiment at 38 cm](measurements/circle120-2026-10-18/README.md)
uses an explicit profile with native acceleration 16→48→16 and eight-second
ramps. The operator accepted start/stop smoothness after rejecting six-second
ramps. The final mean was 120.61°/s, temporal TCP RMS 13.49 mm and maximum error
35.69 mm. Instantaneous speed ripple remains substantial; neither this nor visual
acceptance establishes precision tracking. The original joint budgets remain;
34/36 cm paths exceed them at this circle speed and were not executed.

The [90°/s start/stop follow-up](measurements/circle90-transitions-2026-10-18/README.md)
records accepted start/stop smoothness at all three heights, 34/36/38 cm. The
`smooth-circle90-v2` profile schedules native RAM acceleration from 16 at low speed
to 32 in the middle and back to 16, using the same six-second speed ramps.
Mean speeds were 89.73/89.71/89.65°/s; temporal TCP RMS was 13.00/10.66/9.76 mm.
Substantial instantaneous speed ripple remains; average speed and visual acceptance
do not establish a tracking-accuracy pass. The [fixed-32 baseline](measurements/table-circle90-height-2026-10-18/README.md)
retains the operator's rejected start/stop feedback. The default 60°/s plan remains unchanged.

The [8 cm horizontal circle at 60°/s](measurements/table-circle60deg-s-2026-10-18/README.md)
records real three-lap trials and the operator's accepted start/stop smoothness.
The selected profile measured 60.06°/s mean phase speed, with remaining instantaneous
speed ripple, 10.36 mm temporal TCP tracking RMS and 5.53 mm geometric circle RMS.
These are encoder/CAD estimates. This does not establish optical precision or
white-to-black arm following performance; joints 5 and 6 were held constant.

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
| [Stationary follow jitter](measurements/static-follow-jitter-2026-10-19/README.md) | Recorded 8 Hz shoulder oscillation, outer-D gating, temporary native I=0 on shoulder/elbow, operator observation and bounded stop tests |
| [Initial PID](measurements/pid-2026-10-12/hardware-summary.json) | Original six-channel step baseline |
| [Latency](measurements/follow-latency-2026-10-12/README.md) | Earlier normal-speed latency captures and definition |
| [Remote following](measurements/remote-follow-2026-10-12/README.md) | Remote independence, reversal and overshoot comparison |
| [Wrist retuning](measurements/follow-retune-2026-10-13/README.md) | Normal-speed/stress separation and warning evidence |
| [Smoothness](measurements/follow-smoothing-2026-10-13/README.md) | Current reference shaping, comparison curves and pending checks |
| [Single-servo native PID](measurements/native-pid-single-joint-2026-10-16/README.md) | One installed ID 1, native-loop tuning, large steps and unmet strict precision/50 ms goals |

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
