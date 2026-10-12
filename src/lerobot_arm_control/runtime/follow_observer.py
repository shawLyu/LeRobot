"""Relay an existing remote controller's telemetry; never import motor drivers."""

import argparse
import socket
import sys

from lerobot_arm_control.core.paths import workspace_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=str(workspace_root()/'logs/follow.sock'))
    args = parser.parse_args()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
        stream.connect(args.socket)
        with stream.makefile('r') as reader:
            try:
                for line in reader:
                    sys.stdout.write(line)
                    sys.stdout.flush()
            except BrokenPipeError:
                pass


if __name__ == '__main__':
    main()
