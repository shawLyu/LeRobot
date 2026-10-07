"""Build local CAD-masked textures without blocking incoming hardware feedback."""

import base64
import copy
import threading

import cv2
import numpy as np

from .tabletop_appearance import ObservedAppearance


class AppearanceWorker:
    def __init__(self, config):
        self.config = config
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.state = self.result = self.error = None
        self.delivered = False
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def offer(self, state):
        with self.lock:
            if self.result is None:
                self.state = copy.deepcopy(state)

    def take(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.result is None:
                return {}
            result = {"appearance": self.result["metadata"]}
            if not self.delivered:
                result["appearance_pngs"] = self.result["pngs"]
                self.delivered = True
            return result

    def _run(self):
        try:
            extractor = ObservedAppearance(self.config)
            while not self.stopped.is_set():
                with self.lock:
                    state, self.state = self.state, None
                if state:
                    native = "native_jpeg" in state
                    raw = base64.b64decode(state["native_jpeg" if native else "jpeg"],
                                          validate=True)
                    frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                    result = extractor.extract(frame, state, native)
                    if result:
                        with self.lock:
                            self.result = result
                        return
                self.stopped.wait(2 if state else .05)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def close(self):
        self.stopped.set()
        self.worker.join(timeout=3)
