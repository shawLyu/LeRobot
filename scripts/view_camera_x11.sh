#!/usr/bin/env bash
# Run on the local Mac. The camera remains attached to the remote SSH host.
set -euo pipefail
if [ ! -x /opt/X11/bin/xauth ]; then
    echo "Install XQuartz first: https://www.xquartz.org/" >&2
    exit 1
fi
open -a XQuartz
camera_display="${DISPLAY:-$(launchctl getenv DISPLAY)}"
export DISPLAY="${camera_display:-:0}"
camera_host=${LEROBOT_HOST:-lerobot}
printf -v camera_remote_root '%q' "${LEROBOT_REMOTE_ROOT:-/home/xy/Research/lerobot}"
camera_command="cd $camera_remote_root && exec ./scripts/uv run --locked --extra hardware scripts/camera_x11.py"
for camera_argument in "$@"; do
    printf -v camera_quoted_argument '%q' "$camera_argument"
    camera_command+=" $camera_quoted_argument"
done
exec ssh -X -C \
    -o XAuthLocation=/opt/X11/bin/xauth \
    -o ForwardX11Timeout=0 \
    -o ControlMaster=no -o ControlPath=none \
    -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
    "$camera_host" \
    "$camera_command"
