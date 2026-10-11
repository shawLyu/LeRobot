# Current two-arm tabletop scene — 2026-10-11

The observed desktop uses right AprilTag `DICT_APRILTAG_36h10` / ID 1 as origin,
with left `DICT_APRILTAG_16h5` / ID 0 as auxiliary. Both black borders are 150 mm.
The black follower base is ArUco 4x4 ID 7; the white arm1 base is ID 6.
The former ID 3 reference and old image search regions have been archived.

```sh
./scripts/lerobot-arm view tabletop --fps 5
```

Camera capture/perception and both arms' read-only encoder feedback run remotely;
the local MuJoCo view updates the table, Ts and independently registered bases.
The current view shows both arms with separate encoder streams and independent
six-joint angle mappings. White arm1 alignment was completed on 2026-10-11;
temporary sharing of the black follower mapping is disabled.
Each base keeps its own accepted marker registration. No motors were driven during this capture.
The table is the observed workspace patch; its full physical perimeter is unmeasured.

Run offline Push-T from this preserved desktop observation with:

```sh
./scripts/lerobot-arm task push-t --snapshot data/tabletop/current-scene
```

This uses a frozen observation. It does not mirror subsequent physical changes.
Only the black follower executes simulated pushes; the additional white model remains
fixed in its recorded posture. Both recorded joint sources are `live_encoders`,
using their own accepted mappings. Startup from this two-arm snapshot was checked;
that check does not establish completion of the pushing task.

## Preserved parameters

- `configs/scenes/tabletop-current.json`: complete versioned parameters, calibration
  hashes, independent measurement sources and both arm identities.
- `data/tabletop/current-scene/`: final `state.json`, `scene_parameters.json`,
  `scene.xml` recorded keyframe, config copies, camera/annotated/native images,
  textures and the shared CAD meshes. The same directory is saved remotely.
- `data/calibration/table_reference.json`: accepted current table layout.
- `data/calibration/follower_table_base.json` and `arm1_table_base.json`:
  independent fixed mounting records, each with 20 accepted observations.
- `data/calibration/two_arm_bases.json`: both records, hashes and separation.

Focus remains locked at 68 with autofocus off, native capture is 1920x1080,
processing is 640x360, and the selected October 8 camera calibration is preserved.
Marker sizes remain 42 mm; heights remain 70 mm for follower and 69 mm for arm1.
Control remains the shared V22 response profile at 60 Hz.

The accepted table fit uses 51 training and 12 independent validation frames.
For the stationary camera, training-frame median primary corners determine the shared
extrinsic pose; all primary observations must stay within 2 native pixels of it.
Auxiliary spread must remain below 2 mm / 0.025 rad and held-out joint reprojection
below 2 native pixels. Current maximum held-out RMS is 1.25 px; maximum center spread
is 0.20 mm. These measure image registration stability, not absolute physical accuracy.

| Arm | Base XYZ (mm) | Yaw (degrees) | Stored fit error (processing px) |
| --- | --- | ---: | ---: |
| follower | -469.318, -57.284, 2.400 | 90.165 | 0.170 |
| arm1 | -175.887, -59.913, 2.400 | 88.192 | 0.147 |

Base separation is 293.443 mm. Both 20-observation base records have maximum
position spread about 0.255 mm. Camera movement, tag movement or a changed robot
installation requires checking the marker search regions and new registration.

## Independent six-joint alignment interfaces

Close the tabletop viewer before opening a calibration window. Each arm has its
own `--arm` selector and output file; no mapping or EEPROM identity is overwritten.

```sh
./scripts/lerobot-arm view arm --backend hardware --arm follower \
  --log logs/follower-angle-calibration.json
./scripts/lerobot-arm view arm --backend hardware --arm arm1 \
  --log logs/arm1-angle-calibration.json
```

These commands are separate sessions, not simultaneous readers on the same serial bus.
Keep the existing follower result unless intentionally recalibrating it.

### White-arm six-joint alignment

The new entry selects the white USB identity ending `5B91028324-if00`, preserves
all EEPROM registers, and saves to its own `data/calibration/arm1_mujoco.json`.
The readiness check received about 20 samples/s, with all six torques off and no faults.
Read-only additional-arm calibration does not enable gripper tests or physical control.

```sh
./scripts/lerobot-arm view arm --backend hardware --arm arm1 \
  --log logs/arm1-angle-calibration.json
```

Support the white arm. Press **B**, select joints with **1–6**, and measure each
of joints 1–5 relative to its parent at **0°, +30°, +60°**. Joint 6 is the acquisition
trigger and uses **0°, +15°, +30°**, following the on-screen handle reference.
Press **C** after holding still for at least half a second; **R** advances the reference.
The current pose is not automatically zero. Nine consecutive stable samples are
required; the third reference must pass its independent check before a joint is accepted.
See [angle calibration](angle-calibration.md) for directions and the complete procedure.

After **6/6 aligned**, close the calibration window and copy its accepted software mapping:

```sh
rsync -az data/calibration/arm1_mujoco.json lerobot:/home/xy/Research/lerobot/data/calibration/
./scripts/lerobot-arm view tabletop --fps 5
```

The remote feedback worker verifies the white arm's identity and six fits, then the
viewer updates its prefixed joints independently of the black arm. A stale or invalid
white mapping cannot become live joint feedback. Ordinary deployment preserves this file.
The current snapshot was refreshed after alignment with 39 observed camera frames;
both robot feedback streams were live and independently calibrated. The accepted
white file has SHA-256 `87c7e6c2786678761aaa2e760887f80508b4999cbdfba1ae3c0b4eaf857a6963`.
All six third-reference checks are below 2 degrees; the largest is 1.91 degrees.
The previous snapshot is preserved under `.cleanup/scene-before-independent-arm1-*`.

## Independent mappings

`additional_robots[].joint_calibration` stays `data/calibration/arm1_mujoco.json`.
The former `joint_calibration_fallback` has been removed from the current configuration.
Both buses retain their own fingerprints and accepted files. Missing, invalid or
incomplete white calibration reports an error instead of borrowing black-arm fits.
The follow demo also has `simulation_shared_calibration: false`.

## Leader geometry and follow demo

White arm1 now has the official SO-101 acquisition handle/trigger geometry in
both calibration and tabletop views. Black follower retains its gripper. Existing
base and black joint calibration reference hashes are preserved. Model sources,
independent mappings and the user-confirmed 0–30-degree trigger travel are saved
with the scene. The leader joint and actuator limits use the same measured range.
Run `./scripts/lerobot-arm demo follow` for physical white-leader/black-follower control.
Use `--simulate` for a read-only real-leader preview, or `--source synthetic --auto`
for an offline demonstration. Missing independent calibration blocks the default
hardware mode rather than silently selecting simulation. See
[follow demo](follow.md) for physical execution prerequisites and controls.
