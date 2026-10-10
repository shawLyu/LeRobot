# SO-101 angle calibration in one window

The normal local window is `SO101 - live arm control`.
The main model shows follower feedback; the small lower-left image is a static calibration reference,
not another control window. Previews, raw positions, acknowledgements, and capture share one window.

```sh
./scripts/lerobot-arm view arm --backend hardware --log logs/realtime-arm-live.json
```

## Measurements

The existing follower is confirmed as SO-101. Support the arm physically and ensure all six motors are torque-off.
EEPROM already contains travel limits and Homing_Offset; this process preserves those registers
and saves only the software mapping from encoder ticks to MuJoCo radians.
Each joint uses three **physically measured** relative angles: 0, +30, and +60 degrees.
The first two determine zero/direction; the third is an independent check with 2-degree tolerance.
Positive direction is defined by the model reference, not viewpoint-dependent descriptions such as left or clockwise.

An arbitrary current pose is not zero. Align the supported arm with the reference zero pose,
including base and viewing directions. Measure child-link rotation relative to its parent with a protractor,
angle gauge, or another reliable method. Visual alignment alone gives approximate precision.
Gripper zero also follows the reference; fully closed is not automatically zero.

## Capture

1. Press B for calibration, then 1 for shoulder_pan.
2. Align to the 0-degree reference, support and hold for at least half a second, then press C.
3. Press R for +30 degrees; change only that joint, measure the angle, hold steady, then press C.
4. Press R for +60 degrees and capture similarly. Accepted calibration displays `check passed`.
5. Press R to return the reference to zero, and restore that joint to zero.
6. Repeat with 2–6: shoulder lift, elbow, wrist pitch, wrist roll, and gripper.

Capture causes no automatic motion. It requires nine consecutive feedback frames,
selected-joint variation no greater than 4 ticks, every motor torque-off, and no faults.
About 30 degrees should correspond to 341 ticks; deviations over 10% are rejected.
A failed 60-degree check does not overwrite valid data.
Crossing the 0/4095 encoder wrap boundary also triggers a span check; rearrange the zero/range first.
`Angle alignment N/6` shows progress. Accepted joints immediately use fitted angles;
others retain travel-range estimates until all six are calibrated.

The local file `data/calibration/follower_mujoco.json` is saved automatically and supports resuming later.
It binds stable USB identity, IDs, limits, offsets, mode, Phase, and MJCF checksum.
Device replacement or upstream EEPROM recalibration invalidates it; archive before recapture.
The included completed calibration belongs to the existing experimental device and cannot be reused for another robot.
After local capture, close the window, verify the same device identity, and copy the file to the same remote path;
ordinary deployment preserves remote calibration.
Archive an existing `follower_mujoco_kinematic.json` before raw angle recapture.

## Preview and hardware control

In normal mode, 1–6 selects a joint; arrows send a bounded preview to the remote host.
Uncalibrated previews use +/-20 raw ticks; calibrated ones correspond to approximately +/-0.03 rad.
The remote host validates against actual feedback and EEPROM limits, then returns the raw target.
The main model always shows feedback rather than displaying targets as completed motion.
This read-only entry does not execute targets. After 6/6 calibration, verify physical zero,
direction, and feedback over several poses, then use `./scripts/lerobot-arm view arm --backend hardware --hardware-control` for hardware control.
See [Hardware control](real-control.md). Capture and execution are launched separately; capture sends no physical targets.

## When upstream LeRobot calibration is needed

Use upstream calibration when EEPROM limits are wrong, the device has been rebuilt, or travel no longer matches.
It positions joints near midrange, records travel, then saves offsets/limits;
see [Official SO-101 calibration](https://huggingface.co/docs/lerobot/en/so101#calibrate).
That establishes LeRobot motor normalization and does not replace MJCF angle alignment.
Close the live window to release the serial port, support the arm, and use a separate terminal:

```sh
ssh -t lerobot
cd /home/xy/Research/lerobot
./scripts/uv run --locked --extra hardware lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E114461-if00 \
  --robot.id=xy_follower \
  --robot.calibration_dir=/home/xy/Research/lerobot/data/calibration/lerobot
```

This optional process writes EEPROM and runtime settings; use it only to rebuild motor travel calibration.
Follow the installed version's prompts. Do not sweep wrist_roll through a full revolution when recording limits;
the driver sets its range to 0–4095. Avoid forcing rotation or pulling cables.

The mapping is `q = (raw - zero_raw) * radians_per_tick`.
The file records zero, direction, and the independent check. It aligns joint angles, not link dimensions or tool coordinates.
