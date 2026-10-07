# Runtime inputs and generated data

Keep complete source and offline simulation inputs locally. Deploy only shared modules, hardware runtime, and required inputs remotely.
`scripts/deploy_code.sh` uses `configs/remote-runtime.txt` and preserves remote calibration/tabletop registration.

The current code still uses these inputs:

| Path | Purpose |
| --- | --- |
| `calibration/follower_mujoco.json` | Measured encoder-to-angle mapping; its name does not require a simulation engine at runtime |
| `calibration/follower_mujoco_kinematic.json` | Optional validated height correction, bound to raw calibration and base hashes; previous correction archived after base recalibration |
| `calibration/follower_rest.json` | Validated resting pose for hardware Reset |
| `calibration/follower_table_base.json` | Measured fixed robot-base installation |
| `calibration/arm1_table_base.json` | Independently measured right white arm mounting pose |
| `calibration/two_arm_bases.json` | Both poses, their separation and measurement provenance |
| `calibration/height_samples.json` | New height measurements for local refitting; previous base-bound samples archived |
| `trajectories/circle.json` | Circle trajectory input |
| `tabletop/static_map.json` | Persistent auxiliary marker registration |
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
