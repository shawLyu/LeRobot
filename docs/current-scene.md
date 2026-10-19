# Current two-arm tabletop scene — 2026-10-19

Both base markers have **50 mm black borders**, excluding white margins:
black follower is ArUco `DICT_4X4_50` / ID 7; white arm1 is ID 6.
Both base fronts point toward the T objects. The replacement prints have different
orientations from the previous stickers; `corners_base_m` now matches their observed
code axes. Marker centers and heights remain unchanged.

The table origin is right AprilTag `DICT_APRILTAG_36h10` / ID 1;
left `DICT_APRILTAG_16h5` / ID 0 is auxiliary. Both table borders remain 150 mm.
The selected camera intrinsics and fixed focus 68 are preserved.
Native capture is 1920×1080; processing is 640×360.

## Accepted base calibration

A 35-second camera-only session recorded 158 states. Both bases independently
passed 20-observation calibration, with the primary anchor visible.
No motor bus was opened and no torque, goal or EEPROM writes were sent.

| Arm | Base XYZ in table frame (mm) | Yaw | Stored fit error (processing px) | Maximum position spread |
| --- | --- | ---: | ---: | ---: |
| follower | −427.404, −57.621, 2.400 | 90.581° | 0.377 | 0.857 mm |
| arm1 | −175.939, −61.120, 2.400 | 88.130° | 0.266 | 0.811 mm |

Base separation is **251.489 mm**. Maximum yaw spreads are 0.156° and 0.369°.
The unchanged acceptance limits are 2 mm position spread, 0.025 rad yaw spread,
1.5 processing pixels of base reprojection error, and at least one second of sampling.
These describe observation consistency; absolute physical precision remains unvalidated.

At the end of base calibration, the camera position in the table frame was approximately
`[−315.523, −108.443, 634.114]` mm. The scene records its full rotation and provenance.
The table mesh is the observed workspace patch; its physical perimeter is unmeasured.

## Saved parameters

- `configs/tabletop.json`: 50 mm marker geometry and corrected decoded corner order.
- `data/calibration/follower_table_base.json` and `arm1_table_base.json`: accepted records,
  including current geometry signatures and camera identity.
- `data/calibration/two_arm_bases.json`: both records, hashes, separation and measurement scope.
- `configs/scenes/tabletop-current.json`: current camera/table/base parameters, configuration
  and calibration hashes, and the current controller fingerprint.
- `data/tabletop/current-scene/`: current recorded state, generated scene, source images,
  copied meshes, configuration and calibration records; also synchronized remotely.
- `logs/table-calibration-20261019-152215-679234/`: original local base-calibration capture.
- `logs/world-mirror-validation/`: subsequent 18-second live mirror validation, with 97 camera states,
  independent joint feedback, observed textures and a rendered screenshot.

The base-calibration session sampled base poses only. The current scene was subsequently
refreshed from live feedback: both arms now record `live_encoders`, `independent` calibration
and six measured joint angles. All 12 simulator joint positions matched their measured
angles, including the saved MJCF keyframe. Both arms had torque off and the mirror sent no
motion commands. Existing independent joint-angle files remain unchanged.

Run `./scripts/lerobot-arm view tabletop --fps 10` for ongoing synchronization;
the saved scene is one recorded observation. On stale or failed feedback the viewer marks
the affected arm as unavailable rather than treating a preview as live feedback.
The old 42 mm scene and intermediate camera-only joint previews are superseded by this capture.

## View and recalibrate

```sh
# Camera/table/base preview without opening motor buses.
./scripts/lerobot-arm view tabletop --camera-only --fps 5

# Include each arm's independent read-only encoder feedback.
./scripts/lerobot-arm view tabletop --fps 5

# Recollect bases after moving a base or replacing/rotating its marker.
./scripts/lerobot-arm calibrate table
```

Saved base records reload only when their geometry and camera signatures match.
Changing the printed orientation requires checking its decoded corner correspondence;
small fit residuals alone cannot establish the correct physical base heading.
See [the three calibration workflows](calibration.md) and [base geometry](two-arm-calibration.md).

## White-arm six-joint alignment

The white leader retains its independently measured acquisition handle/trigger geometry;
the black follower retains its gripper. Select each arm explicitly for angle calibration:

```sh
./scripts/lerobot-arm calibrate angles --arm follower
./scripts/lerobot-arm calibrate angles --arm arm1
```

Both mappings remain device-bound and are never copied between arms.
See [angle calibration](angle-calibration.md) for references and resource-hash updates.

Physical following runs entirely on the remote computer with
`./scripts/lerobot-arm demo follow`. The optional `view follow` observer reads telemetry;
it does not participate in the controller. See [follow operation](follow.md)
and [the shared robot configuration](robot-config.md) for the current parameters.
