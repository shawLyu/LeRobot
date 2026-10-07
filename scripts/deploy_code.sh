#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
HOST=${LEROBOT_HOST:-lerobot}
REMOTE_ROOT=${LEROBOT_REMOTE_ROOT:-/home/xy/Research/lerobot}
DRY_RUN=
WITH_CONFIG=false
for argument in "$@"; do
  case "$argument" in
    --dry-run) DRY_RUN=--dry-run ;;
    --config) WITH_CONFIG=true ;;
    *) echo "Usage: $0 [--dry-run] [--config]" >&2; exit 2 ;;
  esac
done
# Remote command arguments below are intentionally restricted to safe path characters.
case "$REMOTE_ROOT" in
  /home/*) ;;
  *) echo "REMOTE_ROOT must be an absolute project path under /home" >&2; exit 2 ;;
esac
case "$REMOTE_ROOT" in
  *[!a-zA-Z0-9_./-]*) echo "Unsupported REMOTE_ROOT characters" >&2; exit 2 ;;
esac
if [ -z "$DRY_RUN" ]; then
  ssh "$HOST" "mkdir -p '$REMOTE_ROOT' && cd '$REMOTE_ROOT' && mkdir -p .cleanup &&
    set --; for entry in src scripts tests docs configs README.md pyproject.toml uv.lock; do
      if [ -e \"\$entry\" ]; then set -- \"\$@\" \"\$entry\"; fi
    done;
    if [ \"\$#\" -gt 0 ]; then
      tar --exclude=__pycache__ -czf .cleanup/code-before-\$(date -u +%Y%m%dT%H%M%S)-\$\$.tar.gz \"\$@\"
    fi"
fi
# Preserve remote-only files, calibration, datasets, logs, the installed runtime
# and machine-specific device/perception configuration.
rsync -az $DRY_RUN --itemize-changes --files-from="$ROOT/configs/remote-runtime.txt" \
  "$ROOT/" "$HOST:$REMOTE_ROOT/"
for directory in docs configs; do
  rsync -az $DRY_RUN --itemize-changes \
    --exclude='__pycache__/' --exclude='*.pyc' --exclude='._*' --exclude='.DS_Store' \
    --exclude='arms.toml' --exclude='camera.toml' --exclude='tabletop.json' \
    "$ROOT/$directory/" "$HOST:$REMOTE_ROOT/$directory/"
done
rsync -az $DRY_RUN --itemize-changes "$ROOT/README.md" "$ROOT/pyproject.toml" \
  "$ROOT/uv.lock" "$ROOT/.gitignore" "$HOST:$REMOTE_ROOT/"
# Seed a new installation while preserving every existing device configuration.
rsync -az $DRY_RUN --ignore-existing --itemize-changes \
  "$ROOT/configs/arms.toml" "$ROOT/configs/camera.toml" "$ROOT/configs/tabletop.json" \
  "$HOST:$REMOTE_ROOT/configs/"
if [ "$WITH_CONFIG" = true ]; then
  rsync -az $DRY_RUN --itemize-changes "$ROOT/configs/tabletop.json" \
    "$HOST:$REMOTE_ROOT/configs/"
fi
