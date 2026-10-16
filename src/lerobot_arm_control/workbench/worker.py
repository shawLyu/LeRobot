"""JSON-lines worker; the serial owner and response clock live on the robot host."""

import argparse
import queue
import signal
import threading
import time

from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.processes import interrupt
from lerobot_arm_control.runtime.common import next_deadline, receive, send
from lerobot_arm_control.transport.latest_output import LatestOutput
from lerobot_arm_control.workbench.controller import Workbench
from lerobot_arm_control.workbench.hardware import open_hardware
from lerobot_arm_control.workbench.registers import BAUDRATES, schema


def run(arm, ids, baudrate, *, hz=100):
    commands, disconnected = queue.Queue(maxsize=32), threading.Event()
    threading.Thread(target=receive, args=(commands, disconnected), daemon=True).start()
    directory = workspace_root() / 'logs/servo-workbench' / str(time.time_ns())
    with LatestOutput(send, disconnected) as publish:
        with open_hardware(arm, ids, baudrate) as (bus, discovery):
            controller = Workbench(bus, discovery, log_root=directory,
                                   context={'arm': arm.name, 'port': arm.port,
                                            'baudrate': baudrate, 'model': arm.model})
            try:
                publish({'type': 'hello', 'arm': arm.name, 'port': arm.port,
                         'baudrate': baudrate, 'schema': schema(), 'state': controller.state(),
                         'sample_hz': hz, 'log_directory': str(directory)})
                deadline = previous_publish = time.monotonic()
                while not disconnected.is_set():
                    # Stop/heartbeat can be acknowledged without waiting for an HTTP round trip.
                    for _ in range(8):
                        try:
                            received_at, message = commands.get_nowait()
                        except queue.Empty:
                            break
                        response = {'type': 'ack', 'id': message.get('id'), 'ok': False}
                        try:
                            if time.monotonic() - received_at > 2:
                                raise ValueError('指令已过期，请重新操作')
                            if message.get('operation') == 'disconnect':
                                controller.hold('界面断开')
                                disconnected.set()
                                result = {'disconnected': True}
                            else:
                                result = controller.command(message['operation'],
                                                            message.get('payload', {}))
                            response.update(ok=True, result=result)
                        except (KeyError, TypeError, ValueError, RuntimeError, OSError) as exc:
                            response['error'] = str(exc)
                        publish(response)
                    try:
                        state = controller.tick()
                    except Exception as exc:
                        controller.fault = f'采样失败：{exc}'
                        controller.hold(controller.fault)
                        controller.enabled.clear()
                        publish({'type': 'error', 'error': controller.fault})
                        break
                    now = time.monotonic()
                    if now - previous_publish >= .05:
                        publish({'type': 'state', **state})
                        previous_publish = now
                    deadline = next_deadline(deadline, time.monotonic(), 1 / hz)
                    disconnected.wait(max(0, deadline - time.monotonic()))
            finally:
                controller.hold('远端工作台退出')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', default='follower')
    parser.add_argument('--ids', help='Six comma-separated device IDs')
    parser.add_argument('--baudrate', type=int, choices=BAUDRATES, default=1_000_000)
    parser.add_argument('--hz', type=int, default=100, choices=range(20, 201))
    args = parser.parse_args()
    arm = load_arms()[args.arm]
    ids = [int(v) for v in args.ids.split(',')] if args.ids else list(arm.motors.values())
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    try:
        run(arm, ids, args.baudrate, hz=args.hz)
    except (Exception, KeyboardInterrupt) as exc:
        send({'type': 'error', 'error': str(exc)})
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
