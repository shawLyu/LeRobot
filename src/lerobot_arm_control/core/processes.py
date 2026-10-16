"""Shared process commands and interrupt handling for robot workers."""

import shlex


def remote_python(root, module, arguments=()):
    root = str(root)
    if not root.startswith("/"):
        raise ValueError("remote project root must be an absolute path")
    command = ["env", "PYTHONPATH=src", "LEROBOT_ARM_WORKSPACE=.",
               ".venv/bin/python", "-u", "-m", module, *map(str, arguments)]
    return f"cd {shlex.quote(root)} && exec " + shlex.join(command)


def interrupt(signum, frame):
    raise KeyboardInterrupt(f'signal {signum}')
