"""Check, capture or continuously restore the configured robot hardware baseline."""

import argparse
import json
import logging
import signal
import time
from dataclasses import replace

from lerobot_arm_control.core.config import load_arms, load_robot_config
from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.processes import interrupt
from lerobot_arm_control.robot.connection import connected


def check(arm):
    with connected(arm, allow_alarm=True, lock_timeout=0) as bus:
        return bus.robot_arm.startup_report


def watch(interval):
    last = {}
    while True:
        try:
            arms = load_arms()
            for name, arm in arms.items():
                try:
                    result = check(arm)
                    state = json.dumps(result, sort_keys=True)
                except Exception as exc:
                    state = str(exc)
                if last.get(name) != state:
                    print(json.dumps({'arm': name, 'state': state}), flush=True)
                    last[name] = state
        except Exception as exc:
            state = str(exc)
            if last.get('config') != state:
                print(json.dumps({'config_error': state}), flush=True)
                last['config'] = state
        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'capture', 'watch', 'confirm-power-cycle'))
    parser.add_argument('--arm', help='Default: all configured arms')
    parser.add_argument('--interval', type=float, default=4)
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING)
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    if args.action == 'watch':
        if not 1 <= args.interval <= 60:
            parser.error('interval must be 1..60 seconds')
        watch(args.interval)
        return
    arms = load_arms()
    names = [args.arm] if args.arm else list(arms)
    if any(name not in arms for name in names):
        parser.error('unknown arm name')
    if args.action == 'confirm-power-cycle':
        from lerobot_arm_control.robot.parameters import confirm_power_cycle

        if not args.arm:
            parser.error('confirm-power-cycle requires --arm')
        print(json.dumps(confirm_power_cycle(arms[args.arm]), indent=2))
        return
    config = load_robot_config()
    results = []
    for name in names:
        arm = arms[name]
        if args.action == 'check':
            results.append(check(arm))
        else:
            from lerobot_arm_control.robot.parameters import capture_parameters, validate_parameters

            # Explicit capture intentionally reads new settings before automatic restoration.
            with connected(replace(arm, parameters=None), allow_alarm=True) as bus:
                values = capture_parameters(bus, arm)
                validate_parameters(replace(arm, parameters=values))
                config['arms'][name]['parameters'] = values
            results.append({'arm': name, 'captured': list(values)})
    if args.action == 'capture':
        # Do not persist machine-local USB overrides in the shared configuration.
        path = workspace_root()/'configs/robot.json'
        original = json.loads(path.read_text())
        for name in names:
            original['arms'][name]['parameters'] = config['arms'][name]['parameters']
        original['captured_unix_ns'] = time.time_ns()
        atomic_json(path, original)
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
