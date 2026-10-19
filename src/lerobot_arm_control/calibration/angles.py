"""Calibrate one arm's encoder-to-angle mapping in a read-only reference window."""

import argparse
import importlib
import sys
from pathlib import Path

from lerobot_arm_control.cli import command_arguments
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.runtime_config import add_remote_arguments

# Precision refinements belong to angle calibration, not separate calibration types.
REFINEMENTS = {
    'refine-height': 'height_client',
    'sample-markers': 'marker_sampling',
    'fit-markers': 'marker_alignment',
}


def main():
    arguments = sys.argv[1:]
    if arguments and arguments[0] in REFINEMENTS:
        module = importlib.import_module(
            'lerobot_arm_control.calibration.'+REFINEMENTS[arguments[0]])
        with command_arguments(arguments[1:]):
            module.main()
        return
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog='Advanced refinements: refine-height, sample-markers, fit-markers; '
               'append --help for their options.')
    add_remote_arguments(parser)
    parser.add_argument('--arm', default='follower', help='Configured arm, e.g. follower or arm1')
    parser.add_argument('--calibration', type=Path, help='Optional independent angle record')
    parser.add_argument('--model', type=Path, help='Optional reference model')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--screenshot', type=Path)
    parser.add_argument('--log', type=Path,
                        default=workspace_root()/'logs/angle-calibration.json')
    args = parser.parse_args(arguments)
    from lerobot_arm_control.runtime import realtime_client

    forwarded = ['--backend', 'hardware', '--calibration-mode', '--arm', args.arm,
                 '--host', args.host, '--remote-root', args.remote_root, '--log', str(args.log)]
    for key in ('calibration', 'model', 'duration', 'screenshot'):
        value = getattr(args, key)
        if value is not None:
            forwarded += ['--'+key, str(value)]
    if args.headless:
        forwarded.append('--headless')
    with command_arguments(forwarded):
        realtime_client.main()


if __name__ == '__main__':
    main()
