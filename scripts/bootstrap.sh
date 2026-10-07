#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
python3 scripts/install_uv.py "$@"
./scripts/uv sync --locked
./scripts/uv pip check
