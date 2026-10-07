"""A latest-state display channel; a slow viewer cannot block remote perception."""

import json
import threading
import time


class DisplayStream:
    def __init__(self, fps, stopped, *, write=None):
        self.period = 1/fps
        self.stopped = stopped
        self.write = write or (lambda value: print(value, flush=True))
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.latest = None
        self.textures = None
        self.samples = []
        self.dropped_samples = 0
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def offer(self, state):
        with self.lock:
            self.latest = dict(state)
            if 'appearance_pngs' in state:
                self.textures = state['appearance_pngs']
            self.samples.extend(state.get('motor_samples', []))
            if len(self.samples) > 1000:
                self.dropped_samples += len(self.samples)-1000
                self.samples = self.samples[-1000:]
        self.changed.set()

    def _run(self):
        last_sent = 0
        try:
            while not self.stopped.is_set():
                self.changed.wait(.1)
                self.changed.clear()
                if self.stopped.wait(max(0, self.period-(time.monotonic()-last_sent))):
                    break
                with self.lock:
                    state, self.latest = self.latest, None
                    textures, self.textures = self.textures, None
                    samples, self.samples = self.samples, []
                if state is None:
                    continue
                if textures:
                    state['appearance_pngs'] = textures
                state['motor_samples'] = samples
                state['display_motor_samples_dropped'] = self.dropped_samples
                # Describe age at transmission, including throttling or network backpressure.
                queued = (time.time_ns()-state['stream_queued_unix_ns'])/1e6
                state['capture_age_at_send_ms'] += max(0, queued)
                if 'robot' in state and 'sample_age_at_send_ms' in state['robot']:
                    state['robot'] = {**state['robot'], 'sample_age_at_send_ms':
                                     state['robot']['sample_age_at_send_ms']+max(0, queued)}
                self.write(json.dumps(state, allow_nan=False))
                last_sent = time.monotonic()
        except (BrokenPipeError, OSError):
            self.stopped.set()
