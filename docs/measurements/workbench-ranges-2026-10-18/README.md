# Follower working ranges — 2026-10-18

The black follower's six STS3215 motors were online and manually guided by the user
with torque off. The white leader's USB device was not connected, so this record
applies to the follower only. No mechanical zero or PID coefficients were changed.

## Applied intervals

All numbers are native encoder ticks, ordered numerically. Direction in physical
space can be reversed. A tick is approximately 0.0879°; this does not establish an
externally measured mechanical angle.

| ID / joint | Manually observed | EEPROM after update | Saved software range |
|---|---:|---:|---:|
| 1 / shoulder_pan | 770–3334 | 844–3440 | **860–3318** |
| 2 / shoulder_lift | 841–3225 | 809–3238 | **857–3209** |
| 3 / elbow_flex | 889–3095 | 773–3126 | **905–3079** |
| 4 / wrist_flex | 915–2884 | 862–3192 | **931–2868** |
| 5 / wrist_roll | 337–3940 | 0–4095 | **353–3924** |
| 6 / gripper | 450–1894; confirmed endpoints 452/1893 | **452–1893** | **464–1881** |

For IDs 1–5, the software interval is the intersection of the manually observed
range and existing EEPROM limits, reduced by 16 ticks (about 1.4°) at each end.
The software interval does not expand ID 1 beyond its existing hardware limit,
even though the user could move it farther by hand with torque off. ID 6 retains
12 ticks (about 1.05°) inside each confirmed endpoint. The 5th-axis retained samples
were checked for large jumps; no 0/4095 wrap was observed in those records.

ID 6 was initially at 590 ticks with EEPROM limits 1441–2982, so the workbench
could not create a valid software range around the actual pose. The user placed
it at minimum/maximum opening and stable readings of 452/1893 were recorded.
Its EEPROM limits were then changed to that confirmed interval while all six
motors had torque off. Independent readback confirmed the values and Lock=1.
The other five EEPROM position limits were preserved.

These software ranges persist on the robot host in ignored
`configs/workbench-ranges.local.json`. They are separate for each arm/channel and
bound to the bus context, ID, offset, phase, resolution, mode and EEPROM limits.
The private identity-bearing file is not published. Reconnection restored all six
ranges exactly, with torque still off until a separate enable command.

## Physical and software verification

The six motors were enabled at freshly read positions, with no reset-to-zero.
Each was then commanded sequentially to center +11 ticks, center −11 ticks and
back to center, at 3°/s. This is approximately ±0.97°, with the other five holding.
The test centers were 2185, 1061, 2888, 1586, 3204 and 571 ticks respectively.
All 18 moves completed, goals matched the requested values, encoder feedback
remained inside the software ranges and final goal errors were no greater than
5 ticks. The test's guard was ≤8 ticks, not a PID precision acceptance criterion.
Twelve targets, one tick below/above each interval, were rejected without starting
a move. PID coefficients, zero offsets, phase and scale matched pre-test readbacks.
All six final storage locks were 1, torque was on and Status was 0.

A final ten-second hold recorded peak-to-peak motion of 0/0/5/6/4/4 ticks and peak
errors of 2/0/3/3/3/2 ticks for IDs 1–6. These new-pose measurements do not repeat
the earlier PID tuning acceptance; this task did not retune gains. They also do
not establish full-range response performance.

The first manual recording stopped when status packets were lost; at one point
IDs 5 and 6 were offline. After the user checked cabling and power, all six returned.
A second manual recording completed. A 15-second independent read-only check
then completed with 621 snapshots. An early motion attempt stopped on stale UI
feedback and requested hold; no hold errors were reported. Its partial traces are
retained. The final complete six-axis motion run was performed after code tests
finished. The motion record ended with two immediate short-read retries; subsequent
monitoring reached eight, with all six online and holding without a device alarm.
Intermittent bus corruption remains observable; the interface shows the count.
The worker does
not fabricate samples or restart a closed connection automatically.

The reusable workbench now permits one immediate re-read only when a failed
snapshot took ≤50 ms. A second failure, a longer failure or a non-voltage device
fault still stops the worker. Both code tests and the real interface verified
that saved ranges are displayed separately from EEPROM limits. Range editing
checks actual torque, including after reconnect. Ordinary EEPROM position-limit
edits cannot exclude the current position.

## Scope and evidence

The broad intervals were inspected by manual movement. Automatic testing covered
only the small neighborhood above; the robot did not autonomously traverse every
boundary or every multi-axis combination. Per-joint intervals are not a Cartesian
workspace or proof against self-collision, table contact or cable strain in all
poses. Inspect a new pose before expanding use.

This record updates the native workbench's limits. Existing angle calibration,
scene data and leader/follower mapping have not been accepted for this changed
installation. The angle-calibration loader checks EEPROM identity, including
position limits, so old calibration records must be reviewed before normal
calibrated demos; a range edit is not a new angle calibration.

- [Machine-independent settings/readback summary](ranges.json)
- [Complete final motion/hold traces, rejected targets and interrupted attempt](verification.json.gz)
- [Workbench controls and persistence behavior](../../servo-workbench.md)

Verification: 79 focused local tests, 468 remote deployment tests, and three
isolated-browser suites passed. Browser regressions mock all hardware APIs;
physical verification is the separately recorded workbench run above.
