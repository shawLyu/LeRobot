#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
. "$ROOT/scripts/local_runtime.sh"
exec "$LOCAL_PYTHON" -m arm_control.tabletop_client "$@"
