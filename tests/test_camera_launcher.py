"""The Mac preview uses the prepared remote runtime, with quoted arguments."""

import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_preview_uses_remote_python_and_preserves_arguments(tmp_path):
    binary = tmp_path / 'bin'
    binary.mkdir()
    log = tmp_path / 'ssh.json'
    for name in ('open', 'launchctl'):
        stub = binary / name
        stub.write_text('#!/bin/sh\nexit 0\n')
        stub.chmod(0o755)
    ssh = binary / 'ssh'
    ssh.write_text(
        '#!/usr/bin/env python3\n'
        'import json, os, sys\n'
        'from pathlib import Path\n'
        'Path(os.environ["CAMERA_TEST_LOG"]).write_text(json.dumps(sys.argv[1:]))\n'
    )
    ssh.chmod(0o755)
    # Replace only the machine-specific XQuartz installation probe in this copy.
    wrapper = tmp_path / 'view-camera.sh'
    source = (ROOT / 'scripts/view_camera_x11.sh').read_text()
    wrapper.write_text(source.replace('[ ! -x /opt/X11/bin/xauth ]', '[ ! -x /bin/sh ]'))
    environment = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ['PATH'],
                       DISPLAY=':99', LEROBOT_HOST='camera-host',
                       LEROBOT_REMOTE_ROOT="/home/robot/project's files",
                       CAMERA_TEST_LOG=str(log))
    arguments = ['--device', '/dev/v4l/by-id/camera name', '--snapshot', 'logs/a frame.png']
    result = subprocess.run(['bash', str(wrapper), *arguments], env=environment,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    command = json.loads(log.read_text())
    assert command[-2] == 'camera-host'
    remote = command[-1]
    assert '.venv/bin/python scripts/camera_x11.py' in remote
    assert './scripts/uv' not in remote
    decoded = subprocess.run(['bash', '-c', 'printf "%s\\n" ' + remote.split(' && exec ', 1)[1]],
                             capture_output=True, text=True)
    assert decoded.returncode == 0, decoded.stderr
    assert decoded.stdout.splitlines() == ['.venv/bin/python', 'scripts/camera_x11.py', *arguments]
