# SO101 hardware control

Default settings come from [Shared configuration](code-layout.md): V14, 60 Hz synchronous targets,
RAM acceleration 20, velocity 600 ticks/s, and bounded position compensation.
Native PID, protections, and measured calibration retain their existing values.

```sh
./scripts/control_arm_realtime.sh
./scripts/control_arm_realtime.sh --circle --circle-laps 3
./scripts/reset_arm_remote.sh
```

Window control, headless Reset, circles, and Push-T share one motor configuration.
The circle entry completes remote Reset, approaches the saved absolute trajectory, runs three laps, then holds with the window open.
Without a lap count it loops. `Space` cancels pending startup or holds the current pose.
Circle and Reset entries can cause motion after launch.

| Key | Action |
| --- | --- |
| 1–6 | Select shoulder pan, shoulder lift, elbow, wrist pitch, wrist roll, gripper |
| E | Precheck all motors, align targets to measured positions, then enable |
| S | Enable/resume selected joint |
| Up/Down | Change nominal target by 0.03 rad; key repeat is supported |
| Space | Clear pending commands and hold enabled joints |
| X | Release selected joint; physically support loaded joints first |
| R | Return to the saved resting pose |
| P | Save a supported stable pose with all six motors torque-off |

Reset uses remote `data/calibration/follower_rest.json`, projecting targets onto existing native endpoints.
Position calibration, identity, model, and offsets must match; teach the pose again after recalibration.
The remote monotonic clock executes quintic interpolation with planned peak speed 0.5 rad/s.
After reaching 5-tick tolerance, it freezes targets and observes two seconds of rest.
Automatic release occurs only when the target itself has been verified stable with all torque off; otherwise supporting torque remains.

Enable confirms the current target before torque, preventing activation of stale targets.
Motor faults, torque loss, SSH/heartbeat loss, and hardware-window focus loss trigger hold.
Ordinary tracking errors are displayed/recorded; retired custom error thresholds do not interrupt motion.
All active targets are sent synchronously each frame; released joints receive no automatic targets.
Exit/release restores prior RAM acceleration/velocity. Closing the window retains supporting torque.
Manual motion does not perform obstacle planning; the Reset path needs physical clearance.

Only one tool may own the serial port. Close hardware/Push-T windows before headless Reset.
See [Measured performance](control-performance.md) for historical evidence.
The code cleanup does not include new physical motion validation.
