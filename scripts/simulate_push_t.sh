#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
. "$ROOT/scripts/local_runtime.sh"
exec "$LOCAL_PYTHON" -m lerobot_arm_control.planning.push_t "$@"
