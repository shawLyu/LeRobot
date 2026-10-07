#!/usr/bin/env bash
# No motor writes. Hardware tests are intentionally separate from pytest.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
./scripts/uv run --locked arm-control --output logs/environment.json doctor
./scripts/uv run --locked arm-control --output logs/read-test.json read --arm all --samples 3
