"""Start the remote-only follower controller in a terminal, without a simulator."""

import argparse
import subprocess
import time
from pathlib import Path, PurePosixPath

from lerobot_arm_control.core.processes import remote_python
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.transport.realtime import JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_remote_arguments(parser)
    parser.add_argument('--start-held', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--control-log')
    parser.add_argument('--latency-test', action='store_true')
    parser.add_argument('--latency-joints', nargs='+', choices=JOINTS, default=JOINTS)
    parser.add_argument('--log', type=Path, default=Path('logs/follow-remote-trace.json'))
    args = parser.parse_args()
    if args.duration is not None and not 0 < args.duration < float('inf'):
        parser.error('--duration must be finite and positive')
    options = ['--hardware-control', '--standalone']
    if args.start_held:
        options.append('--start-held')
    if args.duration is not None:
        options.extend(['--duration', str(args.duration)])
    if args.latency_test:
        args.control_log = args.control_log or (
            args.remote_root+f'/logs/follow-latency-{time.time_ns()}.json')
        options.extend(['--latency-test', '--latency-joints', *args.latency_joints])
    if args.control_log:
        if not PurePosixPath(args.control_log).is_absolute():
            args.control_log = str(PurePosixPath(args.remote_root)/args.control_log)
        options.extend(['--control-log', args.control_log])
    command = remote_python(args.remote_root, 'lerobot_arm_control.runtime.follow_server', options)
    try:
        result = subprocess.run(['ssh', '-t', '-o', 'ServerAliveInterval=5',
                                 '-o', 'ServerAliveCountMax=2', args.host, command])
        if args.latency_test and result.returncode == 0:
            import json

            from lerobot_arm_control.diagnostics.follow_latency import analyze, save_plot

            args.log.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(['scp', f'{args.host}:{args.control_log}', str(args.log)], check=True)
            trace = json.loads(args.log.read_text())
            report = analyze(trace, required_joints=args.latency_joints)
            args.log.with_suffix('.metrics.json').write_text(json.dumps(report, indent=2)+'\n')
            save_plot(trace, report, args.log.with_suffix('.png'))
            if not report['passed']:
                raise SystemExit(2)
        raise SystemExit(result.returncode)
    except KeyboardInterrupt:
        # SSH forwards the terminal interrupt; the remote ExitStack holds first.
        raise SystemExit(130) from None


if __name__ == '__main__':
    main()
