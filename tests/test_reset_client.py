import pytest

from arm_control.reset_client import run_reset


class Link:
    error = None

    def __init__(self, failure=False):
        self.now = 0.0
        self.sent = []
        self.failure = failure
        self.process = self

    def poll(self):
        return None

    def send(self, op):
        self.sent.append(op)
        return len(self.sent)

    def wait_ack(self, command, timeout=5):
        return {"ok": True}

    def sleep(self, delay):
        self.now += delay

    def snapshot(self):
        phase = "moving" if self.now < 0.5 else (
            "cancelled: hold" if self.failure else "boundary hold")
        state = {"control": {"parking": {"phase": phase, "notice": "motor fault: hold"}},
                 "telemetry": {"Status": {"gripper": 0}}}
        return None, state, self.now, 1, None


def test_windowless_client_sends_one_reset_and_heartbeats_only():
    link = Link()
    state = run_reset(link, clock=lambda: link.now, sleep=link.sleep)
    assert state["control"]["parking"]["phase"] == "boundary hold"
    assert link.sent.count("reset") == 1
    assert set(link.sent) == {"reset", "ping"}


def test_windowless_client_requests_hold_when_remote_reset_aborts():
    link = Link(failure=True)
    with pytest.raises(RuntimeError, match="motor fault"):
        run_reset(link, clock=lambda: link.now, sleep=link.sleep)
    assert link.sent[-1] == "hold"
