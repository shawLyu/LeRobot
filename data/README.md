# Runtime inputs and generated data

Keep complete source and offline simulation inputs locally. Deploy only shared modules, hardware runtime, and required inputs remotely.
`scripts/deploy_code.sh` uses `configs/remote-runtime.txt` and preserves remote calibration/tabletop registration.

The current code still uses these inputs:

| Path | Purpose |
| --- | --- |
| `calibration/follower_mujoco.json` | Measured black-arm mapping; currently also the explicitly temporary white-arm simulation mapping source |
| `calibration/follower_mujoco_kinematic.json` | Optional validated height correction, bound to raw calibration and base hashes; previous correction archived after base recalibration |
| `calibration/follower_rest.json` | Validated resting pose for hardware Reset |
| `calibration/table_reference.json` | Current accepted ID 1 / ID 0 table layout and independent frame validation |
| `calibration/follower_table_base.json` | Current fixed black-arm base in the ID 1 frame |
| `calibration/arm1_table_base.json` | Independently measured right white arm mounting pose |
| `calibration/two_arm_bases.json` | Both poses, their separation and measurement provenance |
| `calibration/arm1_mujoco.json` | White-arm angle mapping after its own physical 0/30/60-degree measurements; currently pending |
| `calibration/height_samples.json` | New height measurements for local refitting; previous base-bound samples archived |
| `trajectories/circle.json` | Circle trajectory input |
| `tabletop/static_map.json` | Persistent auxiliary marker registration |
| `tabletop/current-scene/` | Complete October 11 scene, photos, textures, meshes and final observation; mirrored remotely |
| `push_t/measured/` | Independent local offline/test observation snapshot; not deployed remotely |

Active control settings always live in `configs/control.json`.
Device calibration/registration carries identity; deployment preserves the remote records.
Verify identity and associated files before updating them. Do not remove them as ordinary output.

`.gitignore` explicitly permits accepted runtime inputs. Raw height samples,
calibration candidates and backups remain local and are excluded from Git;
accepted optional height corrections may be tracked after validation.
The offline `push_t/measured/` sample tracks only state, calibration, the marker
and appearance textures. Its generated scene, camera photos and mesh copies are
excluded; offline simulation copies the canonical meshes when needed.
Historical contact measurements are a separate test fixture at `tests/fixtures/height-first-round-samples.json`;
regression tests no longer depend on that file inside Push-T experiment outputs.
New `tabletop/<session>/`, `push_t/<session>/`, and dataset outputs are ignored by default;
capture/simulation entries create output directories as needed.
Before removing old sessions, stop their processes and retain required observations/calibration independently.

On 2026-10-07, old per-frame images, traces, and duplicate session models were removed on both hosts.
Small JSON results/status/verification summaries remain under each host's `logs/data-cleanup-20261007/`.
`manifest.json` records deletion scope and retained-file SHA-256 values.
Historical session paths document evidence provenance; original per-frame data was removed.
Summaries contain no complete image/trajectory data and cannot reconstruct the old sessions.

On 2026-10-11, the actual visible right marker decoded as AprilTag 36h10 / ID 1.
The previous ID 3 records were archived before rebuilding both bases and the table layout.
`configs/scenes/tabletop-current.json` is the versioned parameter/provenance record;
the complete generated scene is preserved on both hosts at `data/tabletop/current-scene/`.

The camera record selected on 2026-10-09 is versioned separately at
`configs/camera_intrinsics.json`. The October 7 base records were archived on both hosts under
`.cleanup/20261009-table-id3/`. They remain historical measurements: runtime signatures now bind them to the camera intrinsics,
distortion and selected source hash, so new base registration is required with
the selected camera calibration. Their previous separation is not a new measurement.

On 2026-10-11, the unused generated `data/push_t/run/` output was moved to
`.cleanup/20261011-unused-demo-*/run/` with a per-file SHA-256 manifest.
The measured snapshot's 18 cached mesh files were verified byte-for-byte against
`assets/so101/assets/` and removed (17,409,712 bytes). Offline scene creation
uses the canonical model meshes when this cache is absent. Observation,
calibration, marker and appearance inputs remain available for tasks and tests.
