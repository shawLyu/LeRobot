#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
. "$ROOT/scripts/local_runtime.sh"
cd "$ROOT"
exec "$LOCAL_PYTHON" "$ROOT/scripts/verify_response.py" "$@"
