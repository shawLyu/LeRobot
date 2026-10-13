"""Connection and launcher settings; environment and CLI overrides remain explicit."""

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from lerobot_arm_control.core.paths import resource_path, workspace_root


@dataclass(frozen=True)
class RemoteRuntime:
    host: str
    root: str


def load_runtime_config():
    config = json.loads(resource_path('configs/runtime.json').read_text())
    if config.get('version') != 1:
        raise ValueError('invalid runtime configuration version')
    return config


def remote_runtime(host=None, root=None):
    config = load_runtime_config()['remote']
    host = host or os.environ.get('LEROBOT_HOST') or config['host']
    root = root or os.environ.get('LEROBOT_REMOTE_ROOT') or config['root']
    if (not isinstance(host, str) or not host or host.startswith('-')
            or any(c.isspace() for c in host)):
        raise ValueError('remote SSH host must be a nonempty host or alias')
    if not isinstance(root, str) or not PurePosixPath(root).is_absolute():
        raise ValueError('remote project root must be an absolute path')
    return RemoteRuntime(host, root)


def add_remote_arguments(parser):
    remote = remote_runtime()
    parser.add_argument('--host', default=remote.host, help='Hardware worker SSH host')
    parser.add_argument('--remote-root', default=remote.root,
                        help='Hardware worker project directory; unused in simulation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('setting', choices=('remote.host', 'remote.root',
                                           'local.fallback_environment'))
    args = parser.parse_args()
    if args.setting.startswith('remote.'):
        print(getattr(remote_runtime(), args.setting.split('.')[1]))
    else:
        configured = load_runtime_config()['local'].get('fallback_environment')
        if configured:
            path = Path(configured).expanduser()
            print(path.resolve() if path.is_absolute() else (workspace_root()/path).resolve())


if __name__ == '__main__':
    main()
