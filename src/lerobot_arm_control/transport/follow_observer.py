"""One-way Unix-socket telemetry; observers have no command channel."""

import fcntl
import json
import socket
import threading
from pathlib import Path

from lerobot_arm_control.transport.realtime import VERSION


class FollowBroadcast:
    def __init__(self, path):
        self.path = Path(path)
        self.condition = threading.Condition()
        self.hello = self.latest = None
        self.sequence = 0
        self.closing = False
        self.clients = set()

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lease = self.path.with_suffix('.lock').open('a')
        try:
            fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lease.close()
            raise RuntimeError('remote follow controller is already running') from None
        # Only the exclusive owner can remove a socket left by a crashed worker.
        self.path.unlink(missing_ok=True)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.socket.bind(str(self.path))
            self.path.chmod(0o600)
            self.socket.listen(8)
            self.socket.settimeout(.2)
        except BaseException:
            self.socket.close()
            self.lease.close()
            raise
        self.worker = threading.Thread(target=self.accept, daemon=True)
        self.worker.start()
        return self.publish

    def publish(self, message):
        with self.condition:
            packet = {'version': VERSION, **message}
            if packet['type'] == 'hello':
                self.hello = packet
            elif packet['type'] == 'state':
                self.latest = packet
                self.sequence += 1
            self.condition.notify_all()

    def accept(self):
        while not self.closing:
            try:
                client, _ = self.socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            client.settimeout(.5)
            with self.condition:
                if len(self.clients) >= 8:
                    client.close()
                    continue
                self.clients.add(client)
            threading.Thread(target=self.stream, args=(client,), daemon=True).start()

    def stream(self, client):
        seen, greeted = -1, False
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.closing or (
                        self.hello is not None and self.latest is not None
                        and self.sequence != seen))
                    if self.closing:
                        return
                    packets = ([] if greeted else [self.hello])+[self.latest]
                    seen = self.sequence
                for packet in packets:
                    client.sendall((json.dumps(packet, allow_nan=False)+'\n').encode())
                greeted = True
        except (OSError, ValueError):
            pass  # A slow, closed or malicious observer cannot stop the controller.
        finally:
            client.close()
            with self.condition:
                self.clients.discard(client)

    def __exit__(self, *args):
        with self.condition:
            self.closing = True
            self.condition.notify_all()
            for client in list(self.clients):
                try:
                    client.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        self.socket.close()
        self.worker.join(timeout=.5)
        self.path.unlink(missing_ok=True)
        self.lease.close()
