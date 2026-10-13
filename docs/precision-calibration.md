# Dual-arm scene and independent precision checks

The current scene uses table ID 1 / ID 0, independent black/white base records
and independent six-joint angle mappings. Both arms can display calibrated live
encoder feedback. White arm1 is the leader with a collection handle; the black
follower has a gripper. See [the saved scene](current-scene.md) and
[two-arm calibration](two-arm-calibration.md) for mounting measurements.

The table fit used 51 training and 12 held-out frames, with maximum held-out RMS
1.25 native pixels. Both bases used 20 accepted observations; saved separation is
293.443 mm. These quantify image registration consistency. Absolute 1 mm Cartesian
accuracy, the complete workspace and continuous six-axis following remain unaccepted.
Current motor response evidence is indexed in [Control performance](control-performance.md).

## Viewing and reference changes

```sh
./scripts/lerobot-arm view tabletop --fps 5
./scripts/lerobot-arm view tabletop --camera-only --fps 5
```

Camera-only mode does not open the motor bus. Moving the camera, primary table
reference or a base invalidates associated registration. Recollect the affected
records using [table calibration](tabletop.md), [camera calibration](camera.md)
and [base calibration](two-arm-calibration.md). Do not change measured marker
sizes or mount heights to compensate for a projection error.

## Independent moving-marker measurements

Base markers 7 and 6 locate rigid bases. They cannot independently measure a
moving wrist or fingertip. Encoder-derived forward kinematics is not an optical
measurement of Cartesian error.

The black fixed finger's ID 8 uses `DICT_4X4_50`, a user-confirmed 20 mm black
square excluding white margin. `configs/tabletop.json.robot.tool_marker` stores
its detector settings and nominal CAD placement. The physical tool transform and
printed-code orientation have not passed validation. A fixed-finger marker cannot
independently validate moving-jaw opening; that requires a separate measurement.

The optional [42 mm ID 8 print](../assets/calibration/tool-marker-id8-42mm.svg)
is a different size from the installed marker. Measure the printed black border
and set the actual size in configuration before using it.

```sh
./scripts/lerobot-arm inspect tool-mount
./scripts/lerobot-arm verify joints --help
```

Camera state publishes marker position/orientation, timestamp, reprojection error
and separate validity flags. It uses observed native image corners, calibrated
intrinsics and table extrinsics, independently of robot angles. Missing or
ambiguous measurements are flagged, not replaced by predictions.
`precision_validated` remains false until independent physical validation.

First measure stationary optical noise. Then compare repeated approaches from
both directions at several reaches using independent validation poses. Keep the
moving marker and table reference visible. Camera sampling is slower than the
60 Hz motor loop and cannot establish every transient's performance.

For a supervised bounded shoulder-pan observation from a checked clear pose:

```sh
./scripts/lerobot-arm verify joints --joint shoulder_pan --observe-marker \
  --execute --amplitude-ticks 16 --cycles 3 --dwell 3
```

The command records fresh camera observations and motor feedback. It does not
certify fingertip accuracy unless the marker-to-tool transform is independently
validated. Height correction has its own [measurement workflow](height-calibration.md).

Old ID 3 layouts, superseded experimental instructions and unversioned scan
outputs were retired by the [cleanup](maintenance/README.md). Retained response
reports identify their exact configurations; their historical gains are not
current operating instructions.
