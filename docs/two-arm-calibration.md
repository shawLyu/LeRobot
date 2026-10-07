# Two-arm tabletop base calibration

Measured on 2026-10-07 from the remote camera. Both mounting poses share the
primary AprilTag `DICT_APRILTAG_36h10` / ID 1 coordinate frame. Simulation and
MuJoCo remain local; the remote host only captures observations, estimates poses,
and provides hardware feedback/control for the configured follower.

## Physical identities and confirmed geometry

| Arm | Appearance in the current camera view | Base marker | Joint feedback/control |
| --- | --- | --- | --- |
| `follower` | Left black arm | ArUco `DICT_4X4_50` / ID 7 | Existing follower bus |
| `arm1` | Right white arm | ArUco `DICT_4X4_50` / ID 6 | Mounting pose only; role unassigned |

The user confirmed the same installation dimensions for both arms:

- Black marker border: **42 x 42 mm**, excluding the white margin.
- Marker plane: parallel to the tabletop.
- Marker center: **69 mm above the tabletop**, on the base lateral centerline.
- Marker center: **19.5 mm forward of the base rear edge**, toward arm extension.

The original SO101 CAD base frame has rear edge X = -22.365002 mm and table
contact Z = -2.400002 mm. Thus the marker center in that frame is
`[-2.865002, 0, 66.599998]` mm. Each corner is 21 mm from the center along
the base X/Y axes. `corners_base_m` follows decoded marker corner order,
accounting for the different printed rotations of IDs 7 and 6.

## Measured poses and separation

The origin is the primary tabletop marker center. +X points toward its decoded
right edge, +Y toward its decoded top edge, and +Z above the table. Positions
refer to the original SO101 CAD base origins, not the elevated marker centers.
Quaternions in the saved JSON use `[w, x, y, z]` order.

| Arm | X (mm) | Y (mm) | Z (mm) | Yaw (degrees) |
| --- | ---: | ---: | ---: | ---: |
| Black follower / ID 7 | -465.076 | -54.116 | 2.400 | 90.465 |
| White arm1 / ID 6 | -175.949 | -54.089 | 2.400 | 88.876 |

Base origin separation, projected onto the tabletop: **289.127 mm**.
The relative displacement from follower to arm1 is `[289.127, 0.027, 0]` mm.
Both bases have the same constrained height, so planar and 3D separation agree.
These figures describe this installation; sampling spread is not an absolute accuracy estimate.

| Quality check | Follower | Arm1 |
| --- | ---: | ---: |
| Accepted distinct camera observations | 20 | 20 |
| Maximum position deviation from median | 1.220 mm | 0.430 mm |
| Maximum yaw deviation from median | 0.513 degrees | 0.049 degrees |
| Stored reprojection error in calibrated 640x480 coordinates | 0.927 px | 0.529 px |

Acceptance retains the existing thresholds: at least 1 second of observations,
position deviations below 2 mm, yaw deviations below 0.025 rad, and marker
reprojection error below 1.5 px. No thresholds were relaxed for this capture.

## Runtime and saved files

`configs/tabletop.json` defines the follower under `robot` and the white mounting
pose under `additional_robots`. Each arm owns separate marker identities and a
separate `fixed_base.file`; duplicate names, markers or file paths are rejected.

- `data/calibration/follower_table_base.json`: follower fixed pose and mount signature.
- `data/calibration/arm1_table_base.json`: white arm fixed pose and mount signature.
- `data/calibration/two_arm_bases.json`: both measured poses, separation, quality and provenance.

All three records are synchronized between local and remote hosts.
The live stream contains `base_registrations`, indexed by arm name, and
`base_distances`, containing pairwise displacement and separation in meters.
The existing follower `robot.registration` field remains available.
Held or missing poses are excluded from distance calculations; `both_fixed`
indicates that both endpoints are saved calibrations.

```sh
# Local preview; SSH starts remote perception and read-only follower feedback.
./scripts/view_tabletop_realtime.sh --fps 5

# Camera observations without opening the local GUI.
./scripts/view_tabletop_realtime.sh --headless --duration 10 --fps 5
```

The local viewer shows both articulated SO101 models, base statuses and their
separation. The follower uses live encoder angles; arm1 joint angles remain an
explicit translucent, unmeasured preview because they have not been calibrated
or streamed. No motor targets were written during registration.
`configs/control.json` (V14), the original follower encoder calibration, and the
existing rest pose were preserved. The rest pose identity still matches the
active raw angle mapping.
See [Precision calibration](precision-calibration.md) for camera-only observation,
the moving-marker interface, supervised gripper measurements and the remaining
elbow zero/range discrepancy before whole-arm trajectories.

## Recalibrate after changing the installation

Archive any follower height correction, candidate and samples bound to the old
base before replacing it. Then make the primary table anchor and both markers
visible, deploy the installation configuration, and collect again:

```sh
./scripts/deploy_code.sh --config
./scripts/view_tabletop_realtime.sh --recalibrate-base --fps 5
```

`--recalibrate-base` archives both previous base files and collects independent
20-observation calibrations. Confirm both statuses become `fixed`, then copy
the two base files back locally and refresh the pair summary. Saved mounting
poses load on restart without requiring the robot markers to remain visible.
Moving a base or the primary tabletop marker requires new registration.

For the present camera view, approximate native-image search regions are stored
in each marker's `border_seed_native`. The black base can make the follower
detector include the white margin; the white arm detector can shrink inside the
black square. Observed line intersections therefore determine the actual black
border for both arms, with an exact expected-ID check after all detection paths.
This uses observed pixels without reconstructing hidden code cells. New viewpoints
require inspection and updated search regions. Keep the black border visible.

## Height calibration dependency

The previous height correction, candidate and samples were bound to the replaced
follower base and are preserved on both hosts under
`.cleanup/20261007-two-arm-registration/calibration/`, with original SHA-256
values in `manifest.json`. They are no longer active. Raw encoder-to-angle
calibration remains active; collect and validate new fingertip height measurements
before relying on corrected height for physical Push-T.
See [Height calibration](height-calibration.md).

Live capture evidence is in `logs/two-arm-registration-20261007-final/`;
`logs/two-arm-registration-20261007-restart/` records the restart check.
These generated sessions are ignored by Git. The three calibration records are
explicitly permitted by `.gitignore` for version control.
