"""Local proxy for the remote motor executor; acknowledgements never imply completion."""

import copy
import threading


class MotorLink:
    def __init__(self, link, model_hash):
        self.link, self.model_hash = link, model_hash
        self.lock = threading.Lock()
        self.job_id = None
        self.progress = {"phase": "idle", "writes_enabled": False}

    def observe(self, state, received):
        # The remote perception worker provides its own fresh observations to the motor.
        pass

    def submit(self, state, segments, *, recovery=False):
        reference = {key: copy.deepcopy(state[key]) for key in
                     ("captured_unix_ns", "robot", "objects")}
        with self.lock:
            self.job_id = self.link.send("push_execute", reference=reference, segments=segments,
                                         recovery=recovery, model_sha256=self.model_hash)
            self.progress = {"phase": "pending", "writes_enabled": False}

    def cancel(self, reason):
        with self.lock:
            try:
                self.link.send("push_stop", reason=reason)
            except OSError:
                self.progress = {**self.progress, "phase": "stopped",
                                 "reason": "remote connection lost; watchdog holds"}
            else:
                self.progress = {**self.progress, "phase": "stopping", "reason": reason}

    def update(self, result):
        with self.lock:
            if result.get("job_id") == self.job_id and result.get("motor"):
                self.progress = dict(result["motor"])

    def acknowledge(self, message):
        with self.lock:
            if message.get("id") == self.job_id:
                if not message["ok"]:
                    self.progress = {"phase": "stopped", "writes_enabled": False,
                                     "reason": message.get("error")}
                elif message.get("result", {}).get("motor"):
                    self.progress = dict(message["result"]["motor"])

    def status(self):
        with self.lock:
            return dict(self.progress)
