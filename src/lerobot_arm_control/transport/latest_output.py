"""Keep SSH/display backpressure out of the physical control loop."""

import threading
from collections import deque


class LatestOutput:
    def __init__(self, send, disconnected):
        self.send, self.disconnected = send, disconnected
        self.condition = threading.Condition()
        self.responses = deque()
        self.latest = None
        self.closing = False
        self.worker = threading.Thread(target=self.run, daemon=True)

    def __enter__(self):
        self.worker.start()
        return self.publish

    def publish(self, message):
        with self.condition:
            if message['type'] == 'state':
                self.latest = message
            else:
                if len(self.responses) >= 64:
                    self.disconnected.set()
                    raise RuntimeError('follow response queue full; holding')
                self.responses.append(message)
            self.condition.notify()

    def run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.closing or self.responses
                                            or self.latest is not None)
                    if self.responses:
                        message = self.responses.popleft()
                    elif self.latest is not None:
                        message, self.latest = self.latest, None
                    elif self.closing:
                        return
                self.send(message)
        except (OSError, RuntimeError):
            self.disconnected.set()

    def __exit__(self, *args):
        with self.condition:
            self.closing = True
            self.condition.notify()
        # A blocked pipe may outlive this thread. Hardware has already been held
        # by ExitStack; a daemon writer must never prevent that cleanup.
        self.worker.join(timeout=.5)
