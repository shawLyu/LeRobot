# Single-servo native PID benchmark

This entry operates one selected STS3215, even when the other five motors are absent.
The finite experiment runs on the remote motor computer without MuJoCo, camera,
SSH feedback round trips or the software outer PID. It is separate from the normal
six-joint demo controller.

Prepare the mechanical reference, load, cable clearance and permitted travel before
capturing a zero. The current experiment was authorized with a simple load and
clear travel within +/-90 degrees. The software origin is raw 2067; no homing offset
or EEPROM position limit was changed. This is an encoder reference, not an external
measurement of absolute mechanical angle.

On the robot computer, capture a new origin with torque already off:

```sh
cd /home/xy/Research/lerobot
PYTHONPATH=src .venv/bin/python -m lerobot_arm_control verify native-pid zero \
  --arm follower --joint shoulder_pan --allowed-degrees -90 90 --confirm-zero \
  --output data/calibration/new_single_zero.json
```

An existing zero file is never overwritten. Changing the homing offset, phase,
encoder scale, operating mode, device identity or EEPROM limits invalidates it.

Run a finite baseline after preparing the authorized installation:

```sh
PYTHONPATH=src .venv/bin/python -m lerobot_arm_control verify native-pid capture \
  --zero data/calibration/follower_shoulder_pan_zero.json \
  --p 16 --i 0 --d 32 --amplitudes 5 15 30 --repeats 2 --execute \
  --output logs/native-pid-baseline
```

`--amplitudes` specifies the full step: 90 degrees means -45 to +45 degrees and back.
Targets must fit both the saved authorization and actual EEPROM limits. Begin with
small angles before expanding the experiment. `--p`, `--i`, `--d`, `--deadzone`,
`--integral-limit`, `--startup-force` and `--acceleration` affect only this trial.
Integral-limit register 25 uses four native output counts per unit; the installed
LeRobot 0.6.1 table omits its name, so this entry adds a per-instance alias matching
the [manufacturer's STS table](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/).

Storage remains locked throughout; no EEPROM unlock is issued. Capture seeds the
initial goal with fresh measured position before enabling torque. It stops on an
alarm, lost torque, authorized-bound violation, excessive step overshoot, loaded
stall or temperature reaching 50 C. On exit it disables torque and restores the
original native gains, deadzones, integral limit, startup output and motion profile.
Alarm cleanup does not send a new goal that could clear a device protection latch.
Inspect `restoration_errors`, `final_identity` and `final_registers` in `result.json`.

The requested feedback rate is 200 Hz; results retain actual monotonic sample
and command-write timestamps, read duration and command acknowledgement duration.
Onset requires two counts of directed movement. The report separates onset,
10–90% rise, command-to-90%, overshoot, settling and the last 300 ms steady window.
Strict steady error below one integer tick requires every settled sample to match
the integer target. A small mean does not hide +/-1-tick oscillation. Both approach
directions are measured when returning to the saved zero.

`completed` means a finite acquisition finished, not that the performance goal passed.
`performance_passed` additionally requires strict encoder and zero precision, t90
within the requested 50 ms and acceptable sampling. Encoder agreement does not
establish sub-tick absolute mechanical accuracy. Large-step travel time includes
motor speed and acceleration limits; onset, rise and settling are distinct metrics.

Download reports and create curves in the local simulation environment:

```sh
rsync -az lerobot:/home/xy/Research/lerobot/logs/native-pid-baseline/ logs/native-pid-baseline/
./scripts/lerobot-arm verify native-pid report logs/native-pid-baseline/result.json \
  --output logs/native-pid-comparison
```

The report creates PNG/SVG/PDF response curves, a CSV of measured metrics and JSON
zero-return results. Comparing multiple reports requires matching zero identities.
Candidate settings measured with this simple load need verification after assembling
additional links or changing the load. Normal demos keep using `configs/control.json`.

## Measured results

The [single-joint hardware report](measurements/native-pid-single-joint-2026-10-16/README.md)
retains the zero, compressed samples, candidate settings, search metrics and response
and settled-error curves. The faster P64/I4/D128 candidate reduces 5° command-to-90%
from 254 ms to 89 ms, with zero settled error in those four windows. Its 30°/90°
windows still include one-tick errors, and 150° command-to-90% takes 703 ms. The
largest small-step overshoot is five ticks. The P48/I8/D96 candidate reduces
overshoot to two ticks with one-to-two-tick residuals. Neither passes the strict
precision and 50 ms large-angle requirements. All original settings were restored
and torque disabled; no candidate was persisted. Consult that report for the exact
deadzone, integral clamp and startup-force settings used with each PID.
