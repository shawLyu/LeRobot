# Shared V14 control

Every hardware control entry reads `configs/control.json`. Controllers and CLI tools no longer expose
old native-baseline modes, separate parameter files, or independent period overrides. Deploy and restart after changing parameters.

```sh
./scripts/control_arm_realtime.sh
./scripts/control_arm_realtime.sh --circle --circle-laps 3
./scripts/reset_arm_remote.sh
./scripts/view_arm_realtime.sh --backend mujoco --circle --circle-laps 3
```

Shared settings: 60 Hz, two feedback retries, RAM acceleration 20, velocity 600 ticks/s,
Reset/approach planned peak speed 0.5 rad/s, circle period 4 s, and speed ramp 1 s.
Hardware joints use bounded V14 PI, 120 ms error filtering, velocity feedforward, and friction feedforward.
Local simulation shares nominal trajectory timing without hardware compensation.

Native position PID remains P=16/I=0/D=32; mode 0, return delay 0, maximum acceleration 254.
Gripper native torque/current/overload protections remain 500/250/25.
`native_defaults` records verified native settings; streaming parameters come from `position_feedback.native_stream`.
There is no fallback to the retired control mode. All active joint targets are sent synchronously each frame;
released joints receive no automatic target writes. Faults, lost torque, and connection loss trigger hold;
ordinary tracking errors are displayed and recorded.

Measured calibration and native endpoints are retained; EEPROM is not rewritten.
Exit/release restores the RAM settings present before enable. Both hosts compare the control SHA-256 at connection;
a mismatch prevents control.
See [Code layout](code-layout.md), [Hardware operation](real-control.md), and [Measured performance](control-performance.md).
