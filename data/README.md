# Runtime inputs and generated data

The active system keeps these inputs. Source and local simulation resources remain
local; `configs/remote-runtime.txt` selects the remote hardware runtime.

| Path | Purpose |
| --- | --- |
| `calibration/follower_mujoco.json` | Independent black-arm angle mapping |
| `calibration/arm1_mujoco.json` | Independent white-arm six-joint mapping |
| `calibration/follower_rest.json` | Accepted hardware resting pose |
| `calibration/table_reference.json` | Accepted ID 1 / ID 0 table layout |
| `calibration/follower_table_base.json`, `arm1_table_base.json` | Independent fixed mounting records |
| `calibration/two_arm_bases.json` | Pair geometry and measurement provenance |
| `trajectories/circle.json` | Reproducible circle input |
| `trajectories/table-circle-8cm.json` | Reviewed 60°/s horizontal 8 cm circle |
| `trajectories/table-circle90-h{34,36,38}cm.json` | 90°/s circles at three table-relative heights |
| `trajectories/table-circle120-h38cm.json` | Explicit 120°/s circle at 38 cm; original joint budgets retained |
| `trajectories/table-circle120-tilt{15,30}.json` | 15°/30° circle-plane tilts about base Y, same center/radius/speed |
| `tabletop/static_map.json` | Persistent current table registration |
| `tabletop/current-scene/` | Current saved scene, images, textures, meshes and configuration copies |
| `push_t/measured/` | Small local offline/test observation with calibration and textures |

`configs/scenes/tabletop-current.json` is the versioned scene parameter record.
The complete current scene is saved on both hosts. Configuration and accepted
calibration carry device/model identities and are retained during deployment.

Optional new height samples, candidates and corrections belong in `calibration/`.
The old base-bound correction is not active; collect and validate measurements
against the current base before applying a new correction. Historical first-round
samples needed for regression live at `tests/fixtures/height-first-round-samples.json`.

`push_t/measured/` does not need exported XML, original camera photos or duplicate
meshes for offline execution. Scene construction uses its state/calibration/textures
and copies the canonical meshes from `assets/so101/assets/` when needed.

Current trigger endpoint and unresolved overload diagnostic records remain under
`logs/` for pending physical checks. Versioned motor-response measurements live
under `docs/measurements/`.

New captures and task runs recreate their output directories. Generated outputs
are ignored by default. Remove finished sessions only after stopping their
processes and retaining required calibration, current scenes and accepted evidence.
Never delete the entire `data/` directory or use `git clean -fdx` to prune it.
