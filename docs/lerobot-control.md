# Shared control configuration

Every hardware control entry reads `configs/control.json`. Controllers and CLI tools no longer expose
old native-baseline modes, separate parameter files, or independent period overrides. Deploy and restart after changing parameters.
Precision trajectories use its V22 feedback profile. The leader/follower demo uses
`configs/follow.json` with `controller_profile: native_position`, preserving existing
servo gains, acceleration and velocity rather than applying temporary V22 tuning.
It retains the shared feedback retries and connection configuration hash checks.

```sh
./scripts/lerobot-arm view arm --backend hardware --hardware-control
./scripts/lerobot-arm view arm --backend hardware --hardware-control --circle --circle-laps 3
./scripts/lerobot-arm reset
./scripts/lerobot-arm view arm --backend mujoco --circle --circle-laps 3
```

Shared settings: V22 responsive, 60 Hz, two feedback retries, RAM acceleration 40,
velocity 600 ticks/s, approach peak 0.8 rad/s, minimum approach 0.35 s,
Reset speed 0.5 rad/s, circle period 4 s, and speed ramp 1 s.
The first five hardware joints use bounded PI, 80 ms error filtering and velocity
feedforward; shoulder/elbow also use friction feedforward. Two supervised response
trials improved speed and encoder-model error. Five-joint optical alignment is
still unvalidated; earlier V15–V18 recordings remain in the precision guide.
Local simulation shares nominal trajectory timing without hardware compensation.

Native baseline PID is P=16/I=0/D=32; mode 0, return delay 0, maximum acceleration 254.
V22's `locked_p_coefficients` requires Lock=1 and readback, and restores P on
close/release: pan/lift/wrists use 24, elbow uses 32, and gripper remains 16.
Native I/D and protections are retained.
Gripper native torque/current/overload protections remain 500/250/25.
`native_defaults` records verified native settings; streaming parameters come from `position_feedback.native_stream`.
There is no fallback to the retired control mode. All active joint targets are sent synchronously each frame;
released joints receive no automatic target writes. Faults, lost torque, and connection loss trigger hold;
ordinary tracking errors are displayed and recorded.

Measured calibration and native endpoints are retained; storage is never unlocked and no persistent gain change is written.
Exit/release restores the RAM settings present before enable. Both hosts compare the control SHA-256 at connection;
a mismatch prevents control.
See [Code layout](code-layout.md), [Hardware operation](real-control.md), and [Measured performance](control-performance.md).
