# Tabletop T reconstruction and live MuJoCo view

The remote host `lerobot:/home/xy/Research/lerobot` performs perception: capture, distortion correction,
AprilTag localization, color segmentation, contour detection, known-model occlusion matching, and annotated images.
The local host receives/saves observations over SSH and displays them with MuJoCo; it does not run the perception pipeline.

```sh
./scripts/deploy_code.sh
./scripts/lerobot-arm view tabletop
```

The left panel shows the annotated image and observation status; the right shows complete T models.
Esc closes and releases the camera. Drag to rotate and scroll to zoom.
Keep markers fully visible with a white margin outside the black border. Run one capture process per camera.

## Localization and fixed dimensions

`configs/tabletop.json` contains the calibrated 640x360 processing intrinsics, five distortion coefficients, and geometry.
Both tabletop markers have **150 mm** outer black borders:

- AprilTag 36h11 / ID 3: fixed world anchor.
- AprilTag 16h5 / ID 0: auxiliary localization marker.

The world origin is ID 3's center; +X points right in its decoded orientation, +Y upward, and +Z away from the table.
Seeing both markers learns ID 0 relative to ID 3. Later localization can combine them or use registered ID 0 if ID 3 is occluded,
while retaining the ID 3 origin. An unregistered ID 0 alone at startup waits for ID 3.
When fusion error is excessive but ID 3 remains visible, the system falls back to `pose_source=anchor_fallback`.
Keep their relative layout fixed; restart and relearn after moving markers.

The confirmed T dimensions are **48 mm width, 60 mm length, 12 mm cap depth, 12 mm stem width**.
The wooden T is **6 mm** thick; the printed black target shares its XY contour and is displayed as a plane.
Geometry is fixed rather than shrinking to visible fragments. Wood contours are back-projected onto a top surface
6 mm above the table; two boxes preserve the concave T shape. This perception scene has no contact physics enabled.

## Partial occlusion

Observe each T sufficiently once to establish position/orientation.
Full contours relocalize unoccluded objects. For partial views, the remote tabletop ROI searches the complete known template
near the previous pose. Visible cap/stem edges, interior color support, and continuity estimate XY/yaw;
occluder edges do not redefine object size. Insufficient evidence or similarly plausible competing poses prevent updates.

Each object records `tracking_status`, `observed`, `observation_age_s`, and `pose_valid`:

| Status | Meaning | Display |
| --- | --- | --- |
| observed | Sufficient contour observation | Complete, opaque |
| partial | Visible edges support known-model pose | Complete, opaque |
| static_cached | Black target occluded; use its last observation | Complete, translucent |
| held | Insufficient wood observation; retain latest pose for up to 2 s | Complete, translucent |
| lost | No adequate wood observation beyond hold duration | Transparent ghost; invalid pose |

`complete_outline_xy` always contains the fixed model's eight vertices.
`edge_support` and `visible_fraction` measure template support, not independent accuracy.
The black target cache assumes it did not move during occlusion; a sufficient new observation updates it.
A monocular camera cannot recover a completely occluded moving wood block; removing the occluder enables reacquisition.

## Local models and recordings

Each launch creates `data/tabletop/<session>/` and saves remote outputs. Old recordings are not removed automatically:

- `frames/*.jpg`: JPEG camera frames.
- `annotated/*.jpg`: markers, complete T contours, tracking overlays.
- `states.jsonl`, `latest.json`: capture/receive times, corners, extrinsics, poses, contours, occlusion, quality.
- `calibration.json`: intrinsics, marker dimensions, anchor, T geometry, remote devices.
- `model.json`, `scene.xml`, `marker.png`: fixed geometry, initial pose, marker texture.
- `summary.json`: end-of-session statistics.

Latest models are also saved to `assets/tabletop/`. XML records the pose at model creation;
live poses are in `latest.json`/`states.jsonl`. Known geometry is saved even before a T is observed,
but unlocalized objects are not displayed on the table.

The modeled table is a workspace plane covering markers and objects, not a measured table boundary.
The robot base is registered through fixed markers; see [Base registration](push-t-remote.md).
Wood is assumed to rest on the table; its height cannot be estimated directly when lifted.
Invalid marker localization or SSH state older than 0.5 s invalidates the view.
GUI frame age measures time since local receipt; data separately records remote capture-to-send age.
No one-way delay is inferred from unsynchronized host clocks.
Early sessions used ID 0 as origin; current sessions use ID 1. Compare their calibration files before comparing records.

## Verification

```sh
ssh lerobot 'cd /home/xy/Research/lerobot && .venv/bin/pytest -q tests/test_tabletop_perception.py tests/test_tabletop_tracking.py'
./scripts/lerobot-arm view tabletop --headless --duration 15
```

Tests cover plane back-projection, two-marker recovery, fusion fallback, moving-T completion under partial occlusion,
rejection of occluders and stem-only ambiguity, wood hold timeout, and static target caching.
References: [OpenCV PnP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html),
[MuJoCo geometry](https://mujoco.readthedocs.io/en/stable/overview.html).

## Live SO101 mirror

`./scripts/lerobot-arm view tabletop` displays the table and SO101 together.
The remote follower bus is sampled read-only by stable USB identity, normally at 20 Hz.
Angles use the 0/30/60-degree fit in `data/calibration/follower_mujoco.json`.
Startup checks USB identity, motor IDs, limits, Homing Offset, Phase, Operating Mode, and original MJCF hash.
Any mismatch reports an error rather than falling back to uncalibrated angles.
Sampling writes no targets, changes no torque, and does not modify EEPROM.
The local host updates qpos and calls mj_forward; the scene has no control actuators.

Earlier ArUco MIP 36h12 base markers were vertical ID 4 and horizontal ID 5, both 20 mm;
the horizontal marker was 42 mm from the rear base edge. Their decoding at 640x480 was unstable,
so remote capture changed to 1920x1080 MJPG. Eight tabletop marker corners define an affine resolution mapping,
with maximum residual about 1.03 native pixels. Table processing images and small-marker corners map back to
calibrated 640x480 coordinates; original K/distortion are retained. Remeasure after changing camera crop, device, or mode.

The current system independently registers the left black follower (ArUco 4x4 ID 7)
and right white arm1 (ID 6). Both markers have 42 mm black borders. The follower marker center
is 70 mm above the table; arm1's configured height is 69 mm. Both centers are
19.5 mm forward of each base rear edge, on its centerline.
See [Two-arm calibration](two-arm-calibration.md) for the measured poses and separation.
`base_registrations` carries both mounting poses; `base_distances` carries their measured separation.
Only the follower has live joint feedback and hardware control.
The local scene includes a separate six-joint white-arm model at its measured
base. Its joints are a translucent, unmeasured preview. `--camera-only` skips the
motor bus entirely and can run alongside a controller. See
[Precision calibration](precision-calibration.md) for independent wrist-marker
measurement and the current physical validation limits.
Missing geometry returns `mount_calibration_missing`; such a preview cannot support hardware execution.
The old CAD silhouette fitting interface is retired; the remote host loads no mesh or forward kinematics.

For precisely measured marker installations, populate `corners_base_m` under robot markers in `configs/tabletop.json`.
List black-square corners in decoded top-left, top-right, bottom-right, bottom-left order,
as 3D meter coordinates in the original SO101 base frame.
Two noncoplanar markers enable PnP combined with tabletop camera extrinsics and output tracked/held/markers_lost.
Motor zero calibration alone does not determine this mounting transform.

Additional session files:

- `native/*.jpg`: native 1920x1080 frames; frames/annotated remain.
- `motors.jsonl`: six-joint encoders, radians, torque, voltage, temperature, timestamps.
- `states.jsonl` / `latest.json`: displayed angles, base source/status/cache age.
- `robot_assets/*.stl`, `so101_source.xml`: local CAD backup; scene.xml includes the table, both Ts and localized six-joint arm models for independent loading.

The original calibration model remains unchanged. A `recorded_pose` keyframe stores the initial joint pose.
After loading, call `mujoco.mj_resetDataKeyframe(model, data, 0)` then `mujoco.mj_forward(model, data)`.
Replay later poses/images/registration from JSONL recordings.
Motor feedback older than 0.5 s or connection failure makes the robot translucent with an error while camera capture continues.
Table reference and motor feedback loss are handled separately; the program does not compete for an occupied serial port.

## Observed materials and lighting

`appearance.enabled` in `configs/tabletop.json` enables observed appearance.
Once tabletop, both Ts, and base are valid, a local background thread extracts materials from received native images,
using local MuJoCo for CAD occlusion masks. The remote host performs localization/perception only.
`Materials: observed` means textures are loaded; image and motor streams continue.

The table is back-projected to its plane, excluding moving wood, black T, markers, robot, and visible foreground.
Table/paper color, paper regions, and fine details enter the texture; low-frequency brightness variation is removed
so moving-object ghosts and captured shadows are not drawn twice.
T cap/stem textures are extracted in each object's frame and move with its live pose;
unseen regions use visible material color. Full robot CAD is retained, printed parts use observed matte black,
and motor housings retain moderate highlights.

Table brightness variation estimates a main light direction, combined with ambient/fill light and MuJoCo dynamic shadows.
One ordinary image cannot separate reflectance, exposure, and illumination. This is an appearance approximation,
not photometrically calibrated lighting, unseen-surface texture, or a complete robot UV map.
The default viewpoint follows the actual camera direction; drag/scroll changes it.

Session and latest `assets/tabletop/appearance/` contain PNGs for table, auxiliary marker, and each T's cap/stem,
plus `appearance.json`. Metadata records source frame/time, coverage, colors, texture bounds, lighting, and approximation status.
`scene.xml` loads these textures; `model.json` retains appearance metadata.
Original robot models and motor calibration are unchanged. Restarting extracts appearance from new observations.

Remote `data/tabletop/static_map.json` retains only observed fixed black-target positions.
Restart validates dictionary, marker IDs/sizes, and T geometry before restoring them with their actual observation age.
Dynamic wood is not cached across sessions. Visible contours remain checked during occlusion;
insufficient evidence is shown as static_cached. Remove the cache and reobserve after moving target or reference markers.
The recorded restoration reused previous measured positions and revalidated current visible contour as partial.

Tests cover observed materials, six-joint/keyframe retention, moving-T exclusion, texture consistency after rotation,
no extraction without localization, and cross-session static-cache age/reference checks.
