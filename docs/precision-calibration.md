# Dual-arm simulation and measured control precision

Updated on 2026-10-09. Both SO101 models run locally. The remote host captures
camera observations and executes the shared follower controller without MuJoCo.

## Current response configuration (2026-10-09)

After the optical verification recorded below, two supervised simultaneous
five-joint response trials selected the shared `measured-v22-responsive` profile.
The repeated trial reduced large-excursion 90% arrival time from 1.454 to 0.886
seconds and encoder/model moving RMS from 5.56 to 4.05 mm. This is a response
comparison, not acceptance of absolute 1 mm optical calibration. Table, base,
joint-angle and tool-mount acceptance status remains unchanged. Earlier V14
references below describe the configuration at the time of those measurements.
See [Control performance](control-performance.md) for settings, scope and evidence.

## ID 8 five-joint verification (2026-10-09)

All first five follower joints were driven through small, individual excursions
and returns. The fixed-jaw ID 8 retained its confirmed **20 mm black border**.
The jaw motor was not enabled. The elbow initially read 3122 against an upper
encoder limit of 3126; the first move reduced its target by 16 counts before the
remaining diagnostic poses. Each transition moved one joint by at most 16 counts.

The initial isolated shoulder-pan trial stopped after a missed marker frame and
high processing latency. A camera-only `--marker-only` mode now skips T-shape
fitting and base registration while preserving table and tool pose estimation.
It neither saves a replacement table map nor opens the motor bus. The complete
three-cycle retry recorded **351/351 valid ID 8 frames**, with median perception
time reduced from approximately **376 ms to 116 ms**. Settled encoder error peaked
at **5 counts**, and the lower target's repeated camera-position medians differed
by up to **6.565 mm**. These results do not meet a 1 mm accuracy claim.

The subsequent five-joint scan completed **11 poses and 902/902 valid marker
frames**. Without an accepted base transform, analysis fitted only a diagnostic
base yaw from the first pan excursion and compared the remaining nine poses'
relative displacements. Maximum held-out residuals were **4.772 mm** against
measured-encoder FK and **4.578 mm** against commanded-encoder FK. Translation
and absolute base registration remain unvalidated. Nominal CAD marker placement
also remains unvalidated, and recorded wrist-roll coordinates are outside the
canonical model range. FK used recorded angles without clamping; this calculation
cannot certify physical accuracy or activate an offset correction.

Separate live stationary captures compared the existing contour refinement with
subpixel refinement. All encoder spans were zero in both captures. The contour
method accepted **119/119** analyzed frames and gave X/Y/Z standard deviations of
**1.209 / 0.649 / 2.778 mm**; subpixel refinement gave
**5.496 / 2.847 / 12.623 mm**. The existing contour method and shared V14 control
configuration were retained. ID 8's shortest projected edge remains about 47
native pixels. Increasing its image size is the next acquisition prerequisite;
80 pixels is a capture target, not an accuracy guarantee. Camera movement requires
fresh table/base observations before any absolute calibration is accepted.

`lerobot_arm_control.calibration.marker_sampling --relative-only` explicitly records motion without
an accepted base. Its camera visibility guard uses the same 30-pixel threshold
as isolated repeatability checks; absolute acquisition retains its 48-pixel
threshold and requires the accepted base file. `marker_alignment` rejects
relative-only captures. Physical limits, tracking-error stops, freshness checks,
and the marker-loss watchdog remain active in both acquisition modes.

Evidence and analyses are under `logs/id8-verification-20261009/`, including
`pan-analysis.json`, `five-joint-analysis.json`, and `live-corner-comparison.json`.
Postflight feedback reported no servo faults. The first five joints remain
powered to support the arm; the gripper remains unpowered. No controller gains,
EEPROM settings, angle calibration, or accepted base transforms were changed.

## Current table reference (2026-10-09)

The user requested a new registration from the two complete markers on the left.
The lower `DICT_APRILTAG_36h11` / **ID 3** is now the origin; its decoded right,
up and outward directions define +X, +Y and +Z. The upper `DICT_APRILTAG_16h5` /
**ID 0** is the registered auxiliary. Both retain the user-confirmed **150 mm
black border side**. This replaces the original ID 1 world coordinates; old
ID 1 observations and poses must not be mixed with new ID 3 data.

A 30-second camera-only capture returned 124 distinct frames with both markers.
The local fit uses 100 training frames and 24 held-out frames. The auxiliary
center in the new frame is **(-345.494, -1.025, 0) mm**, with yaw **91.587 degrees**
and center separation **345.495 mm**. Training X/Y standard deviations are
0.221 / 0.180 mm; maximum center deviation is 0.870 mm. Held-out joint
reprojection RMS is 1.007–1.168 native pixels (mean 1.070). These are registration
and repeatability checks, not a certification of 1 mm physical accuracy.

The accepted record is [table_reference.json](../data/calibration/table_reference.json);
[static_map.json](../data/tabletop/static_map.json) contains its runtime auxiliary
layout. Local and remote copies use the same coordinates. A fresh restart
returned **49/49 valid table frames**, all fusing both tags and exactly restoring
the accepted layout; native RMS was 0.953–1.107 pixels. Camera focus readback was
68 and no motor bus was opened. Evidence is under
`logs/table-reference-id3-20261009/`.

To repeat after moving a table marker, first set the primary and auxiliary IDs
in `configs/tabletop.json` and both `fixed_base.reference` entries. Archive old
frame-dependent height corrections and pair summaries, then collect and fit:

```sh
./scripts/deploy_code.sh --config
./scripts/view_tabletop_realtime.sh --camera-only --recalibrate-table \
  --headless --duration 30 --fps 5 --output logs/new-table-capture
./scripts/calibrate_table_reference.sh --capture logs/new-table-capture \
  --output logs/new-table-capture/table-reference-candidate.json
```

The local fitter requires matching capture configuration, verified fixed focus,
distinct timestamps, stable observations and held-out native RMS below 2 pixels.
It reports an accepted registration candidate; it does not activate a new map.
Install its `reference`, `secondary_layouts` and `observed_unix_ns` into the runtime
map, clear old `black_t` data, and synchronize the accepted record/map to the
remote host before a restart check. A new table reference invalidates both saved
bases. Persistent maps now also bind to the selected native intrinsics hash.

Old table maps, both October 7 base records and the old pair summary were archived
on both hosts under `.cleanup/20261009-table-id3/`, with original hashes. The
follower encoder mapping, resting pose, canonical CAD and shared V14 controller
were preserved. Both base markers decode, but the confirmed 42 mm / 69 mm mount
constraints give approximately 8–9 native pixels of reprojection error, exceeding
the unchanged acceptance threshold. The user reconfirmed these original mounting
parameters; they were not altered to fit the image. No new base record was written.
ID 8 remained invisible in both captures. Base and tool metrology must be resolved
before proceeding with Cartesian control validation.

## Selected camera calibration (2026-10-09)

The user selected the **second** checkerboard dataset,
`/home/hkuit-001/camera_calibration/session_20261008_135548/calibration_optimized.json`.
Its exact original JSON is versioned as [camera_intrinsics.json](../configs/camera_intrinsics.json),
with SHA-256 `bf1aca2304736e8c099c7b4f2723fc389be39ad5239b15faae34e149127078ca`.
The native image size is 1920 x 1080; the board has 10 x 7 inner corners and
15 mm squares. The supplied fit uses 37 training views and three separate
validation views. Reported training RMS is 0.761380 native pixels; reported
validation RMS is 0.679753 native pixels. Redetecting all 70 corners on each
validation image and solving each pose with the supplied fixed intrinsics gave
0.724455, 0.631962 and 0.814780 pixels (mean 0.723732). These are pixel residuals,
not independent physical position or control error measurements.

The shared [tabletop configuration](../configs/tabletop.json) uses an exact 3:1
full-frame processing view (640 x 360). Its first two intrinsic-matrix rows are
the native rows divided by three; distortion is unchanged. The pixel mapping is
`[[3, 0, 0], [0, 3, 0]]`. Native 1920 x 1080 marker detection, the remote
perception stream and local texture projection use this same geometry. The
previous measured 640 x 480 sensor-crop mapping is no longer active. Loading
the configuration checks the original JSON hash and verifies the runtime matrix,
distortion and capture size against it before opening hardware.

Saved base signatures now include intrinsics, distortion and the selected source
hash. Both October 7 base records remain available as historical measurements,
but cannot load as current fixed calibrations. New registration and Cartesian
validation are required before hardware Push-T. The first camera check provisionally retained the old ID 1 auxiliary layouts;
the new ID 3 registration above supersedes them. October 8 marker-check outputs
used the earlier calibration and an ID 3 world origin; they must not be mixed
with encoder/FK measurements in the original ID 1 frame.

The supplied dataset does not record a camera serial or focus setting. The current
project still locks focus at 68 with hardware readback, but this does not establish
that the October 8 calibration used that lens position. No 1 mm precision claim
or new motor gains accompany this camera selection. Extracted original images,
marker checks, source hashes and the independent replay report are retained under
`logs/calibration-discovery-20261009/`, excluded from Git. Only the selected small
intrinsics record and shared runtime configuration are versioned. Previous shared
vision settings are retained under `.cleanup/20261009-second-camera-calibration/`.

A fresh camera-only check on 2026-10-09 saved 30 frames with this exact shared
configuration, native resolution and verified focus 68; no motor bus was opened.
IDs 0 and 3 decoded, but their saved October 7 layout produced 22.877 processing
pixels (68.631 native pixels) of joint reprojection error. Each individual tag
fit approximately 0.09 processing pixels, while their implied camera positions
differed by about 160 mm. The combined layout was correctly rejected rather than
silently switching coordinate origins. ID 8 was not detected in this capture.
That initial table reference was unavailable. The subsequent ID 3 registration
above restored the table; both fixed bases and tool metrology remain pending.

## View both arms

```sh
./scripts/view_tabletop_realtime.sh --fps 5

# Camera and mounting poses only: does not open either motor bus.
# This mode can run alongside the isolated gripper benchmark below.
./scripts/view_tabletop_realtime.sh --camera-only --fps 5
```

After valid base registration, the black follower uses live calibrated encoders.
The white `arm1` has its own six-joint CAD chain, with a translucent,
**unmeasured preview** joint pose. Its joint calibration and control are not enabled. Both chains share
mesh assets, but have independent bodies, joints, sites and materials. Recorded
two-arm scenes contain 12 joints; reset their `recorded_pose` keyframe when loading
them independently. The viewer calls `mj_forward`; it is a kinematic mirror with
no hardware-driving actuators. The historical October 7 separation was
289.127 mm; no new pair separation has been accepted in the ID 3 frame. See
[Two-arm mounting calibration](two-arm-calibration.md).

## What the current markers measure

IDs 7 and 6 are rigidly attached to the bases. They locate the bases and cannot
independently measure a moving wrist or fingertip. Encoder-derived forward
kinematics is not an independent measurement of Cartesian error.

The current follower has a **20 mm black-square side**, confirmed by the user,
`DICT_4X4_50` / ID 8 marker on the outer side of its fixed finger, close to the
finger's upper edge. The white margin is excluded from the 20 mm measurement.
The user also specifies the `View from X-` symmetry center and an 11 mm center
distance from the upper edge. These dimensions now define a nominal CAD mount;
the physical transform and printed-code alignment have not passed validation.
For this five-joint calibration, keep ID 8 on the fixed finger. A separate moving-jaw
marker is needed to independently validate gripper opening. Keep ID 8 and the primary
table anchor visible, and record the measured black-border side and its rigid
transform to the chosen tool reference. The marker center can be measured without
that transform, but fingertip accuracy requires it.

[Optional larger ID 8 marker](../assets/calibration/tool-marker-id8-42mm.svg) uses
`DICT_4X4_50`, with a 42 mm black square; it is not the installed 20 mm marker.
Print at 100%, disable fit-to-page, and
measure the printed square before using that size. Add the following under
`robot` in `configs/tabletop.json`; this matches the installed marker size:

```json
"tool_marker": {
  "enabled": true,
  "dictionary": "DICT_4X4_50",
  "id": 8,
  "marker_size_m": 0.020,
  "max_reprojection_error_px": 1.0,
  "adaptive_thresh_constant": 3.0,
  "corner_refinement": "contour"
}
```

```sh
./scripts/deploy_code.sh --config
./scripts/view_tabletop_realtime.sh --camera-only --fps 5
```

`states.jsonl` publishes `tool_marker`: camera-measured marker-center position
and orientation in the table frame, capture timestamp, reprojection residual,
and separate `position_valid` / `orientation_valid` flags. Measurement uses native
image corners, calibrated intrinsics and table extrinsics; it does not use robot
angles or forward kinematics. Lost markers/table reference and ambiguous square
poses are flagged. Missing measurements are not replaced by predictions. Synthetic
projection tests verify coordinate conversion, not the physical camera accuracy.
Estimate stationary optical noise before evaluating motion, then compare repeated
approaches from both directions at several reaches using independent validation
poses. The camera's current capture rate is approximately 5 Hz; it cannot validate
every transient in the 60 Hz motor loop.

The detector now requires an exact observed code without bit correction. The
stream includes source image dimensions, corners, shortest projected edge and
pixels per code cell. `position_valid` describes geometric acceptance, not verified
physical accuracy; `precision_validated` remains false. Detection settings belong
to the vision configuration and do not alter the shared motor controller.
Contour refinement uses the observed boundary points supported by
[OpenCV's detector parameters](https://docs.opencv.org/4.10.0/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html).

### Fixed-finger CAD reference

Inspect the original local model without opening a camera or motor bus:

```sh
./scripts/inspect_tool_mount.sh
```

The installed marker belongs to the fixed `gripper` body and the visual mesh
`wrist_roll_follower_so101_v1`, not the articulated `moving_jaw_so101_v1` body.
The script accounts for MuJoCo's compiled mesh translation and rotation before
extracting geometry. It runs locally; the remote host needs no MuJoCo installation.
Its output is recorded under `robot.tool_marker.cad_reference` in the shared vision
configuration, with XML and mesh hashes. These are CAD references, not an applied
marker-to-tool calibration.

All coordinates below are **millimeters in the `gripper` body frame**. The fixed
finger extends toward negative Z; these axes do not mean table height or image up.

| Reference | CAD coordinates or extent |
| --- | --- |
| Outer flat surface, identified from the CAD and photograph | X = -35.200; outward normal = (-1, 0, 0) |
| Flat surface triangle bounding box | Y = -12.650 to 12.214; Z = -43.158 to -7.976 |
| Upper rounded shoulder, toward the wrist (+Z) | Z = -5.000 |
| User-specified nominal marker center | (-35.200, -0.218, -16.000) |
| Fixed-finger contact reference, `fixed_jaw_box3` | (-9.000, 0.000, -103.000) |
| Tool site, `gripperframe` | (12.000, -0.218, -98.127) |
| Furthest negative-Z vertex of the fixed visual mesh | Z = -104.425 |

The contact reference is the center of a collision capsule, not the physical
contact surface. `gripperframe` is a different reference with its own orientation;
the two origins are 21.559 mm apart. The flat-surface bounds describe the mesh
triangles' envelope, not a solid rectangular sticker area.

The upper-edge distance is measured from the rounded part's shoulder at Z = -5 mm,
not from the flat patch's inset boundary near Z = -8 mm. The symmetry line is
Y = -0.218 mm. Subtracting the specified 11 mm gives marker-center Z = -16 mm.
The mount assumes negligible sticker thickness and a plane normal of body -X.
The decoded code axes inferred from the installation photograph are marker +X
toward body +Z, marker +Y toward body +Y, and marker +Z toward body -X. This code
alignment is a nominal installation assumption, not a validated angular measurement.

The contact-center offset in this nominal **marker frame** is
**(-87.000, 0.218, -26.200) mm**. This offset refers to `fixed_jaw_box3`; it must not
be reused for `gripperframe` or the outermost mesh vertex. `marker_pose_in_gripper`
records the specified mount; `tool_reference.offset_marker_m` supplies the camera
conversion without importing CAD or MuJoCo remotely.
The local scene renders the exact ID 8 cells as a 20 mm square at this nominal
mount, attached to the fixed `gripper` body. Opening the moving jaw does not move
the marker; wrist motion carries it with the fixed finger. This is a display of
the configured CAD mount, not optical validation of its physical placement.

For unambiguous observed marker poses, `tool_marker.tool_reference.nominal_position`
reports the resulting nominal contact-center position in the table frame. It uses
only the measured camera pose and the configured rigid offset, without encoder FK.
Ambiguous marker orientations produce no nominal contact position. The mount remains
`transform_validated: false` / `tool_transform_measured: false`, so the derived contact
position is not exposed as valid measured feedback and is not used for control.
Current physical precision remains unvalidated.

A 20 mm square with an 11 mm center-to-edge distance has only 1 mm black-frame
clearance at that end. The CAD shoulder has a 3 mm fillet; if the paper conforms to
it, the upper black border may curve. The nominal planar transform does not model
paper curvature or mounting thickness. Verify the black square is flat before
accepting the mount for fine 3D measurements.

After deploying this mount, a 10-second camera-only capture saved 45 frames with
the table reference present throughout. The arm was in a different, oblique pose:
ID 8 returned 7 positions, with a median projected short edge of only 12.1 pixels.
Four unambiguous poses produced nominal contact-center diagnostics; three
orientation-ambiguous poses withheld them. All derived contact positions remained
invalid as measured feedback because the mount is unvalidated. These sparse,
oblique observations do not validate control precision. The received configuration
matches the local file. Evidence: `logs/tool-marker-cad-mount-live-20261007/`.

### Installed-marker stationary checks

No motor target was sent in these captures. The first slanted view did not decode
with the original detector. Reprocessing its 55 saved frames with exact ID decoding
and threshold constant 0 returned 12 positions, with height standard deviation
39.1 mm. After the user manually repositioned the arm, 64 live frames using
subpixel refinement returned 35 positions, with X/Y/Z standard deviations
5.30 / 4.96 / 22.73 mm. The encoders did not move during that stationary capture.

A fresh independent capture using contour refinement and threshold constant 3
returned **21 positions from 69 frames**. X/Y/Z standard deviations were
**1.13 / 1.03 / 4.89 mm**; height span was 21.67 mm. Native-image marker-center
standard deviations were 0.131 / 0.083 pixels. Most decoded poses still had
ambiguous orientation. These statistics describe accepted samples only; they do
not conceal the 48 missing-marker frames or establish fingertip accuracy.
Evidence is retained in ignored `logs/tool-marker-id8-20mm-static-20261007/`,
`logs/tool-marker-id8-20mm-adjusted-20261007/`, and
`logs/tool-marker-id8-20mm-contour-20261007/`.

The marker is currently too small in this view for reliable fine 3D calibration.
The next acquisition target is a black-square short edge around 80 pixels or more,
with the primary table anchor fully visible, followed by another stationary noise
check. This pixel target is an acquisition heuristic, not an accuracy guarantee.
Keep the table anchor and bases stationary when adjusting the camera. V14 was not
retuned from these noisy observations, and no new kinematic correction was applied.

After the camera was moved closer, a further live capture returned **60 positions
from 64 frames (93.75%)**, with the table reference present in every frame. The
median black-square short edge was 37.2 native pixels. X/Y/Z standard deviations
were **1.26 / 1.13 / 4.44 mm**. Recognition improved substantially, while depth
noise still prevents a fine absolute 3D accuracy claim. Evidence:
`logs/tool-marker-id8-20mm-close-20261007/`.

### Recheck after the latest manual angle adjustment

A fresh 18-second stationary capture returned **77 positions from 78 frames**
(98.7%). All six encoders had zero span and all torques were off. The median
short edge was **38.9 native pixels**. X/Y/Z standard deviations were
**0.99 / 1.21 / 4.17 mm**, and all 77 accepted marker orientations remained
ambiguous. Better decoding does not imply fine depth accuracy.

A subsequent supervised wrist-flex experiment reused the unchanged shared V14
controller. It inspected the selected joint's raw/model limits, stepped by 32
counts (approximately 3 degrees), limited the total displacement to 128 counts
(approximately 12 degrees), required fresh decoded ID 8 observations, and stopped
on excessive tracking error or movement of an uncommanded joint. It returned to
the starting target after each complete scan.

| Wrist target offset, counts | Accepted / settled camera frames | Unambiguous poses | Z standard deviation, mm |
| --- | --- | --- | --- |
| 0, first visit | 31 / 32 | 0 | 2.41 |
| +32 | 31 / 31 | 0 | 3.15 |
| +64 | 31 / 32 | 8 | 3.95 |
| +96 | 32 / 32 | 7 | 6.28 |
| +128 | 31 / 32 | 1 | 5.55 |
| 0, return | 30 / 32 | 0 | 1.65 |

Four other encoders remained constant; shoulder-pan span was three counts. Settled
wrist tracking error reached three counts, so this experiment did not meet a
two-count criterion at every target. These observations do not validate Cartesian
accuracy or optical repeatability at an identical six-joint pose.

The reverse experiment's first startup failed before movement when its temporary
client queued heartbeats before receiving its first motor state. After correcting
that experiment script, the reverse motion stopped when wrist tracking error
exceeded eight counts for 400 ms. The wrist was held, then returned along the
inspected small displacement using a slower three-second transition. A final
read-only check confirmed all six torques and faults were zero, native velocity
and acceleration were restored to zero, EEPROM locks remained set, and PID stayed
16 / 0 / 32. The shared controller and joint calibration hashes were preserved.

An offline diagnostic also compared the observed fixed-finger silhouette apex
with the nominal CAD apex, without using encoder FK. It supported branch selection
in 64 of the 77 stationary frames. An exploratory five-point fit had 2.40 mm Z
standard deviation on that selected subset. Selection and nominal CAD assumptions
prevent interpreting this as independent absolute-accuracy validation; this
diagnostic has not replaced the runtime marker solver or activated the tool mount.

The camera's enumerated maximum native resolution is 1920 x 1080, already in use.
Increasing output dimensions digitally cannot add optical detail. More real marker
pixels, followed by independent validation, are still needed. Evidence and scripts
are retained in `logs/tool-marker-angle-adjusted-20261007/`,
`logs/marker-wrist-pose-scan-20261007/`,
`logs/marker-wrist-pose-scan-negative-retry-20261007/`, and
`logs/marker-recalibration-20261007.json`.

### Recheck with one clear table anchor

The next 18-second capture retained the original primary tabletop anchor in all
79 frames. ID 8 produced a valid marker-center position in **79 / 79 frames**;
74 orientations were unambiguous and five were ambiguous. All six raw encoder
spans were zero, all torques were off, and the median native short edge was
**44.0 pixels**. X/Y/Z standard deviations were **1.18 / 0.76 / 2.95 mm**;
the Z range was **21.31 mm**. This is stationary observation noise, not measured
control error. The observed joint pose differs from the preceding wrist scan.

The median estimated camera-to-marker distance was **0.53 m**, and unambiguous
marker normals were approximately **10.3 degrees** from the camera optical axis.
The face is already close to frontal; a smaller field of view with more real
marker pixels is needed before fine control verification. One complete visible
primary anchor is sufficient to retain the saved table frame. A secondary marker
can substitute only when its layout has already been registered in that frame.

At this pose, the camera marker center and the nominal CAD mount transformed
through the current encoder mapping differed by **34.27 mm**. That diagnostic
combines uncertainties in the joint mapping, saved base, camera calibration, and
unvalidated physical marker mount. It cannot identify a correction from one pose,
and no kinematic correction or control parameter change was activated. Evidence:
`logs/tool-marker-close-recheck-20261007/analysis.json` and
`logs/tool-marker-close-recheck-20261007/model-comparison.json`.

### Camera-observed small shoulder-pan trial

```sh
# Finite physical motion: support the arm with all six motors initially off.
# Close serial viewers first; this entry starts its own camera-only observer.
./scripts/check_marker_precision.sh --execute --amplitude-ticks 16 --cycles 3 --dwell 3
```

This entry enables only shoulder pan, requires its entire target range to fit the
canonical model and raw limits, and uses the shared controller from `configs/control.json`. It requires
eight fresh ID 8 observations before enabling motion, holds on marker loss beyond
800 ms, and stops if another joint encoder changes by more than four counts.
As with the isolated gripper test, cleanup holds, releases only the selected
non-support motor, and restores its original RAM motion settings. The other five
motors stay unpowered; no whole-arm path or Reset is executed.

The 2026-10-07 trial completed three cycles between raw 2148 and 2164, about
1.35 degrees according to the current angle mapping, with a 3-second dwell per
visit. Across all 145 settled encoder samples, RMS error was **0.814 counts** and
maximum error **one count**. Every other joint's encoder span was zero. All native
PID, protections and calibration files were retained; the final read confirmed
all torques off, Status 0, and restored acceleration/velocity RAM values of zero.

Independent native-image median marker centers had maximum pairwise separation
**0.812 pixels** at the lower target and **0.445 pixels** at the upper target.
Each of the nine settled visits had only 2–4 accepted optical observations. Missing
frames are not filled in, duplicate timestamps are excluded from image statistics,
and missing visits cannot imply perfect repeatability. Pixel-center repeatability
does not establish millimeter fingertip accuracy or validate the whole workspace.

The entry records raw marker observations under `camera/`, full-rate motor traces,
encoder statistics, and `marker_metrics` for future runs. The initial run's derived
image metrics are saved separately as `marker-metrics.json` beside its unchanged
raw result. Evidence: `logs/marker-pan-precision-20261007/` and
`logs/tool-marker-id8-20mm-verification-20261007.json`.

### Latest two-tag registration and repeatability recheck

The user confirmed that the existing auxiliary AprilTag **16h5 / ID 0** had not
moved and that the additional **36h11 / ID 3** black border was **150 mm**.
The primary **36h10 / ID 1** is partly outside the current view. ID 3 was registered
through the unchanged ID 0 layout, preserving the original table frame and both
saved arm bases. A 78-frame median planar fit gave center standard deviations of
**0.154 / 0.093 mm**, yaw standard deviation **0.043 degrees**, and median planar
fit residual **0.719 mm**. These measure fit consistency, not independent absolute
accuracy. The final layout and provenance are stored in local and remote
`data/calibration/tabletop_aux_id3_registration.json`; the active layout is in
`data/tabletop/static_map.json`.

Table markers are now detected in the original 1920 x 1080 image and their corners
are transformed into the calibrated pixel coordinates. This fixes missed markers
that are complete in the native frame but extend beyond the low-resolution crop.
An already registered stationary auxiliary can register another confirmed
same-size table marker. Registration persists even when the static T target is
not visible. If a reference marker moves, `--recalibrate-table` archives the table
map and both saved bases; expose the primary and both base markers to rebuild them.

After saving the median layout and restarting acquisition, **77 / 77** frames had
valid ID 8 positions. Both table auxiliaries were detected in every frame. The
20 mm tool marker's median short edge was **71.2 native pixels**; X/Y/Z stationary
standard deviations were **0.27 / 0.21 / 0.75 mm**. All six encoder spans and torques
were zero. The depth range was **6.19 mm**, so occasional depth outliers remain.
This noise result must not be advertised as mechanical positioning accuracy.

A supervised shoulder-pan trial then completed three 16-count cycles, approximately
1.35 degrees, between raw **2188** and **2204**, with four-second dwells. Across
139 settled encoder samples, RMS error was **0.809 counts** and maximum absolute
error **one count**. All five uncommanded encoders remained constant. The original
online receipt-time analysis had only one accepted camera sample in the final
visit. A separate offline analysis uses actual camera capture timestamps strictly
inside each recorded settled encoder window, on the same remote-host Unix clock;
it contains 3–4 real frames for each of all nine visits, without interpolation.
The original online results are retained unchanged.

The maximum pairwise distance between same-target visit median marker positions
was **0.550 mm** at raw 2188 and **0.638 mm** at raw 2204; native-image median
distances were **0.390 / 0.228 pixels**. One upper-target visit still had **4.51 mm**
depth standard deviation, reinforcing that small-sample median repeatability is
not absolute accuracy. The nominal encoder/CAD marker prediction differed from
the camera median by **11.15 mm** at the pre-motion pose. No joint zero, kinematic
correction, physical tool-transform validation, or PID change was inferred from
this finite trial. Full-arm absolute calibration still needs independent
multi-pose validation.

The final read-only hardware check confirmed all six torques and faults zero,
RAM acceleration and velocity restored to zero, PID **16 / 0 / 32**, and EEPROM
locks set. Shared V14, joint calibration, and both base calibration hashes were
unchanged. Local and remote each passed **46** targeted regression tests; Ruff
and whitespace checks passed. Evidence: `logs/tool-marker-two-tags-recheck-20261007/`,
`logs/tool-marker-two-tags-validation-20261007/`, and
`logs/marker-pan-two-tags-20261007/timestamp-aligned-marker-repeatability.json`.

## Public reference and practical target

The [official STS3215-C001 datasheet](https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf)
specifies a 12-bit encoder (4096 counts, approximately 0.088 degrees per count)
and backlash up to 0.5 degrees. Encoder resolution is not whole-arm accuracy.
The [Robo9 STS3215 experiment](https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/)
reports about ±0.3 mm repeatability at a 100 mm lever, approximately 0.17 degrees
or two encoder counts. That experiment used a different 12 V motor variant and
fixture; it is a comparison target, not a guaranteed SO101 Cartesian limit.

The isolated benchmark uses a strict criterion: every encoder sample in every
final 0.8-second dwell window must be within two counts of its nominal reference.
Report startup, direction, load and settling time rather than retaining only the
best visits. Full-arm absolute accuracy and gearbox output backlash remain
unmeasured until an independent moving marker is installed and validated.

## Supervised isolated gripper benchmark

Support the arm with all motors initially unpowered. Close ordinary motor viewers,
calibration tools and controllers. `--camera-only` can remain open. Each robot bus
is reserved before connection so cooperating workers cannot open it concurrently.

```sh
# Executes a finite, small gripper-only trial, using the current shared profile.
./scripts/check_gripper_precision.sh --execute --amplitude-ticks 16 --cycles 3

# Larger repeated trial, still only about 2.6 degrees of gripper rotation.
./scripts/check_gripper_precision.sh --execute --amplitude-ticks 32 --cycles 5

# Use an explicit nearby encoder center for comparable trials.
./scripts/check_gripper_precision.sh --execute --amplitude-ticks 32 --cycles 5 \
  --reference-raw 1583 --dwell 2
```

Defaults use the shared quintic entry curve. `--step` selects abrupt references for
comparison. A fixed center must be within one trial amplitude of the initial raw
position, and all targets require limit margin. Stale feedback, faults,
disconnects or movement of an uncommanded arm joint stop the trial. The worker
uses the shared watchdog and controller. Cleanup holds, releases only the gripper,
and restores temporary RAM motion settings. Check `completed`, `cleanup_error`,
and acknowledgement evidence in each generated `logs/gripper-precision-*/result.json`.
Full-rate remote traces are copied into that directory as `control.json`.

## Measurements from this installation

All completed trials below kept the other five arm motors unpowered, with no
observed encoder movement in those joints. Only the gripper was actuated. RMS and
maximum errors refer to the final 0.8 seconds of each 2-second dwell, including
initial visits. Repeated-visit range is the span of median readings at each target.

| Trial | Profile / reference | Center / open (raw counts) | Cycles | RMS / max error (counts) | Median range, center / open (counts) | All windows within 2 counts |
| --- | --- | --- | ---: | --- | --- | --- |
| Isolated small trial | V14 / step | 1583 / 1599 | 3 | 0.58 / 1 | 0 / 0 | Yes |
| Larger confirmation | V14 / step | 1583 / 1615 | 5 | 5.30 / 9 | 5 / 6 | No |
| Smooth trial | V14 / quintic | 1580 / 1612 | 5 | 1.26 / 2 | 2 / 0 | Yes |
| Matched-center smooth recheck | V14 / quintic | 1583 / 1615 | 5 | 1.94 / 5 | 5 / 4 | No |
| Software PI candidate | Temporary V15 / quintic | 1583 / 1615 | 5 | 3.16 / 5 | 7 / 2 | No |

The matched-center V14 recheck had up to five-count error in its first two visits;
later visits were within one count. The candidate changed only gripper software
feedback settings and did not improve the overall result. **The original V14
configuration was restored on both hosts.** Native PID and EEPROM/protection
registers were not changed. Native clockwise/counterclockwise dead zones already
read one count. This evidence does not support a universal two-count claim.

Trial directories are `logs/gripper-precision-isolated-20261007/`,
`logs/gripper-precision-confirmation-20261007/`,
`logs/gripper-precision-smooth-20261007/`,
`logs/gripper-precision-matched-smooth-20261007/`, and
`logs/gripper-precision-v15-candidate-20261007/`.
The earlier baseline attempt failed before its target cycle due to overlapping
motor access; its trace is retained, and exclusive pre-open ownership plus
acknowledged cleanup were added. It is not counted as a completed trial.

## Before whole-arm motion and calibration

The earlier resting elbow encoder mapped to approximately -117.9 degrees, outside
the canonical CAD elbow limits of ±96.8 degrees. Resolve the physically measured
joint zero/direction and valid range before executing whole-arm trajectories.
Do not clip measured feedback or widen CAD limits to hide the discrepancy. After
manual repositioning, the elbow mapped within its model range, but shoulder lift
(-100.94 degrees), wrist roll (-179.73 degrees) and gripper (-11.47 degrees) still
fell outside their canonical ranges. Read-only streaming exposes these measurements
without clipping them; whole-arm motion remains unvalidated.
Previous height corrections were archived when the base installation changed;
they have not been reapplied. Whole-arm Reset, circle and physical Push-T have
not been revalidated for this mounting setup.

The next calibration needs the moving marker's actual side and rigid mounting
transform, a physically checked encoder-to-model mapping, and independent
validation poses. Only then can measured optical residuals justify a new shared
controller or kinematic correction. See [Joint angle calibration](angle-calibration.md)
and [Height calibration](height-calibration.md).

## Five-joint marker alignment trials on 2026-10-07

The current task covers the first five follower joints. ID 8 is on the fixed
finger and does not observe the moving gripper jaw; the gripper motor remains
disabled. The white arm is still an unmeasured preview. **A 1 mm Cartesian
alignment has not been accepted or activated.**

`marker_sampling` locally orchestrates finite single-joint moves through the
remote shared controller, recording actual camera capture timestamps, encoder
feedback, nominal commands and acknowledgements. Each transition is limited to
32 counts and every target must remain within 160 counts of the inspected start,
with physical encoder-limit margin. The scanner requires fresh, decoded ID 8,
checks tracking/faults, and holds on failure. On completion it retains support
for the loaded joints and restores temporary native parameters. Do not release
the loaded arm merely to finish a recording.

The local `marker_alignment` fitter uses the nominal CAD marker center, measured
base and native 4096-count scale for the first five joints. It fits five zero
offsets on training poses only. Validation compares independent camera positions
against both actual-encoder FK and nominal-command FK. A small training residual
is insufficient: held-out errors, parameter observability, bounds and stationary
encoder spans are also checked. Candidates remain in `logs/`, with
`activated: false`; they are not compatible with the older height-correction
format and must never be relabeled or copied into that format.

| Completed scan | Poses | Profile | Max held-out camera/FK error | Max camera/command error | Outcome |
| --- | ---: | --- | ---: | ---: | --- |
| A | 13 | V14; 5-degree diagnostic bound | 0.898 mm | 1.631 mm | Failed: poor observability, correction bound, moving validation windows and command error |
| B | 33 | Five-joint V15; 5-degree diagnostic bound | 4.281 mm | 5.298 mm | Failed: instability, correction bounds and Cartesian error |
| C | 35 | V16; 10-degree diagnostic bound | 3.571 mm | 3.857 mm | Failed: Cartesian error |
| D | 13 | V17, fixed focus; 10-degree diagnostic bound | 8.562 mm | 8.292 mm | Failed: poor observability, correction bound and Cartesian error |
| E | 13 | V18, fixed focus, 15 s dwell; 10-degree diagnostic bound | 3.806 mm | 4.288 mm | Failed: instability, poor observability, correction bound and Cartesian error |

The 10-degree bound is an offline diagnostic setting, not permission to send
larger physical moves. B's wider poses improved five-parameter observability,
but zero-offset correction alone did not explain the optical geometry. C reduced
stationary encoder-span median from B's 8 counts to 0; 34 of C's 35 windows stayed
within four counts. It still had up to eight counts of nominal tracking error.
D had zero-to-one-count error in several later windows but up to ten counts in
an earlier window. These are incomplete performance improvements, not 1 mm proof.

All five completed scans recorded 107 poses. E used temporary native P=32, integral
gain 0.3 and deadband 0.0018 rad; its encoder-span median was one count, but it
still had a six-count window span and up to eleven counts of command error.
**The accepted V14 configuration was restored on both hosts.** Every post-trial
hardware read confirmed P/I/D=16/0/32, locks set and temporary acceleration/velocity
restored. The five loaded joints remain powered for support; the gripper remains off.

Evidence directories are `logs/five-joint-marker-scan-{a,b,c,d,e}-20261007/`.
The current acquisition code saves input files/hashes and checks the full local
and remote joint mappings before enabling motion. Reprocessing rejects stale
vision, joint, base or model inputs. Earlier scans retain their original candidate
reports; they must not be silently rebound to a new camera configuration.

### Temporary native gain experiments

ST firmware accepts P changes while `Lock=1`, without saving them across power
loss, according to the [manufacturer's memory map](https://docs.waveshare.net/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation/).
A live shoulder-pan probe confirmed 16 -> 24 -> 16 with lock and supporting torque
unchanged. The shared optional `native_stream.locked_p_coefficients` setting is
limited to the first five joints and values 16..32. Every write requires lock and
P readback; close/release restores the original P. The controller never unlocks
storage, modifies native I/D or alters protections to improve a score.

Completing a reference ramp now clears its integral/feedforward memory while
unwinding the existing correction within its rate limit. The scanner applies
this only to the joint that moved, retaining compensation in the other loaded
joints. All hardware entry points still read the same `configs/control.json`;
there is no separate scanner gain file. Tuning profiles remain experimental
until their independent validation passes. They are disabled in the restored V14 baseline.

### Camera intrinsics and fixed focus

The camera exposed continuous autofocus. The tabletop worker now supports
`camera_controls: {"autofocus": false, "focus": 68}` and requires hardware
readback before streaming. Locking focus alone did not resolve metrology:
`logs/five-joint-focus-locked-20261007/` accepted 76 of 93 positions and measured
XYZ standard deviations of **0.922 / 0.759 / 3.152 mm** during a nominal hold.
That recording did not stream encoders, so it cannot independently separate
optical noise from physical holding drift. It does not verify absolute accuracy.

A new [printable A4 ChArUco board](../assets/calibration/charuco-5x7-30mm.svg) uses
5 x 7 squares, `DICT_5X5_100`, 30 mm square side and 22 mm marker side. Print at
100%, disable fit-to-page, mount on a flat rigid surface, and measure the printed
square side. Prepare it before motion trials; present it only during a dedicated
camera-only capture, keeping the robot supported. Move the board through at
least twenty distinct positions and tilts, covering the image. Do not move the
stationary table markers or robot bases.

```sh
./scripts/view_tabletop_realtime.sh --camera-only --headless --fps 5 \
  --duration 90 --output logs/camera-charuco-session

ROOT=$PWD
. scripts/local_runtime.sh
PYTHONPATH=src "$LOCAL_PYTHON" -m lerobot_arm_control.calibration.camera_intrinsics \
  --capture logs/camera-charuco-session --square-mm 30 \
  --output logs/camera-charuco-session/intrinsics-candidate.json
```

Replace `30` with the physically measured square side. The fitter operates locally
on native images, requires verified fixed focus, and separates every fifth
accepted view for validation. Repeated nearly identical views do not count.
It checks held-out reprojection, tilt and image coverage and never changes the
active configuration automatically. Native intrinsics must be converted with an
explicit image mapping, then independently revalidated with the table, bases and
moving marker before accepting a Cartesian correction. Image reprojection alone
is not a physical millimeter certification.

### Final code verification and retained state

After restoring V14, the local simulation/vision suite passed **365 tests**, with
2 dependency-related skips. The local run excluded `test_safety.py` because the
macOS simulation environment has no LeRobot motor SDK; that module ran on the
remote host as part of **192 passing hardware/runtime/vision tests**. Relevant
Ruff checks and `git diff --check` passed. Synthetic calibration tests check the
algorithm and independent holdout gates; they do not establish physical accuracy.

Both hosts confirmed control hash
`861723ec9ad751ab02fb6b94066e81cbdaca3d9697470e66d3940262ee321887`
and vision hash
`cf9743a7905a1c3daa65e890a78ad01376a8c415085c39d000f2a03b2cd468aa`.
The remote runtime still has no MuJoCo. `follower_mujoco_kinematic.json` is absent;
no failed camera correction was activated. The last independent hardware read is
`logs/five-joint-marker-scan-e-20261007/hardware-after.json`; all statuses were zero,
all locks closed, P/I/D restored to 16/0/32, and gripper torque off. Loaded-joint
support remains on. Camera intrinsics and physical marker metrology require the
prepared board before continuing the 1 mm acceptance test.
