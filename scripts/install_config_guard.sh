#!/bin/sh
# Run on the Linux motor host after deploying the canonical robot configuration.
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PYTHON="$ROOT/.venv/bin/python"
[ -x "$PYTHON" ] || { echo "Missing hardware environment: $PYTHON" >&2; exit 1; }
case "$ROOT" in *[!a-zA-Z0-9_./-]*) echo 'Unsupported project path for systemd' >&2; exit 2;; esac
UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
mkdir -p "$UNIT_DIR"
cat > "$UNIT_DIR/lerobot-config-guard.service" <<EOF
[Unit]
Description=LeRobot saved motor parameter guard

[Service]
Type=simple
WorkingDirectory=$ROOT
Environment=PYTHONPATH=$ROOT/src
Environment=LEROBOT_ARM_WORKSPACE=$ROOT
ExecStart=$PYTHON -u -m lerobot_arm_control config watch
Restart=on-failure
RestartSec=4

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable --now lerobot-config-guard.service
# A user service otherwise stops after logout. This does not grant motor access.
if [ "$(loginctl show-user "$(id -un)" -p Linger --value)" != yes ]; then
  loginctl enable-linger "$(id -un)"
fi
systemctl --user --no-pager status lerobot-config-guard.service
