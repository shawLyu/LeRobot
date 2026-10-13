# Retired-content cleanup — 2026-10-13

The user requested removal of unused old content. The primary filesystem pass
removed 112,574 files / 13,006,640,670 bytes locally and 27,859 files /
2,185,490,131 bytes remotely; two additional obsolete local verification documents
are recorded separately. Local cleanup removed retired
capture sessions, demo outputs, duplicate exported scenes/meshes, old ID 3
registration, historical backup folders, superseded log copies and reproducible
caches. Four superseded operation/verification documents were removed; active
guides were shortened to current procedures and versioned evidence links.

- [Completed local manifest](cleanup-2026-10-13.json): per-root counts/bytes and
  SHA256 of 370 retained input/evidence files. `additional_documentation_removals`
  records the two old verification documents removed after the filesystem pass.
- [Completed remote manifest](cleanup-2026-10-13.remote.json): the exact remote
  deletion scope and hashes of 359 retained inputs. Automatic approval initially
  rejected the bulk deletion; the user explicitly approved it, and execution
  completed with matching retained-file hashes.

Retained inputs include both independent arm calibrations, accepted rest/base/table
records, the complete current scene, the local offline Push-T snapshot, circle
input and versioned motor-response measurements. Current trigger endpoint and
unresolved overload diagnostics remain in `logs/` for pending physical validation.
Source, dependencies, model licenses and device settings were not pruned.

Removed ignored captures and historical backups are not retained elsewhere by this
cleanup. Versioned files remain recoverable from Git history. Historical report
paths or delivery hashes describe their original runs; they do not imply that an
old unversioned capture still exists. Future runs can recreate output directories,
and future deployments can create a new `.cleanup/` rollback snapshot.

Validation after removal: local **622 passed, 1 skipped**; deployed runtime
**363 passed**. All maintained Markdown links resolve. Post-test bytecode caches
and the single temporary archive created by this delivery were also pruned.
No robot motion was executed.
