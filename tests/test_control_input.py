from contextlib import nullcontext

from arm_control.control_input import JogCommands, ResetThenCircle


class Link:
    def __init__(self):
        self.acks = {}
        self.sent = []
        self.lock = nullcontext()

    def send(self, operation, **payload):
        self.sent.append((operation, payload))
        return len(self.sent)


def state():
    return {"limits": {"gripper": [-0.1745, 1.7453]}, "target_qpos": {"gripper": 0.5},
            "last_command_id": None}


def test_circle_waits_for_ack_and_fresh_reset_completion_then_starts_once():
    link = Link()
    start = ResetThenCircle(link)
    s = {"control": {"last_command_id": None,
                     "parking": {"phase": "boundary hold", "notice": ""}}}
    start.advance(link, s, 0)
    link.acks[1] = {"ok": True}
    start.advance(link, s, 0)
    assert link.sent == [("reset", {})]
    s["control"]["last_command_id"] = 1
    s["control"]["parking"]["phase"] = "moving"
    start.advance(link, s, 0)
    assert len(link.sent) == 1
    s["control"]["parking"]["phase"] = "boundary hold"
    start.advance(link, s, 0)
    start.advance(link, s, 0)
    assert link.sent == [("reset", {}), ("circle_start", {})]


def test_pending_circle_cancels_on_user_hold_rejected_reset_or_stale_feedback():
    for cause in ("user", "rejected", "stale", "fault"):
        link = Link()
        start = ResetThenCircle(link)
        s = {"control": {"last_command_id": 1,
                         "parking": {"phase": "boundary hold", "notice": "fault"}}}
        link.acks[1] = {"ok": cause != "rejected", "error": "rejected"}
        if cause == "user":
            start.cancel()
        if cause == "fault":
            s["control"]["parking"]["phase"] = "cancelled: hold"
        start.advance(link, s, .6 if cause == "stale" else 0)
        start.advance(link, s, 0)
        assert "circle_start" not in [op for op, _ in link.sent]
        if cause == "stale":
            assert link.sent[-1][0] == "hold"


def test_reset_then_circle_passes_finite_lap_count():
    link = Link()
    start = ResetThenCircle(link, laps=3)
    link.acks[1] = {"ok": True}
    s = {"control": {"last_command_id": 1,
                     "parking": {"phase": "unpowered rest", "notice": ""}}}
    start.advance(link, s, 0)
    assert link.sent[-1] == ("circle_start", {"laps": 3})


def test_fast_key_presses_accumulate_before_feedback():
    jogs, link, control = JogCommands(), Link(), state()
    jogs.request("gripper", 0.03, control, 0.5)
    jogs.request("gripper", 0.03, control, 0.5)
    jogs.flush(link, control)
    assert link.sent[0][1]["angle"] == 0.56
    jogs.request("gripper", -0.03, control, 0.5)
    jogs.flush(link, control)
    assert len(link.sent) == 1
    link.acks[1] = {"ok": True}
    jogs.flush(link, control)
    assert len(link.sent) == 1  # ACK alone is insufficient: this is still an old state.
    control["last_command_id"] = 1
    control["target_qpos"]["gripper"] = 0.56
    jogs.flush(link, control)
    assert link.sent[1][1]["angle"] == 0.53


def test_hold_cancels_unsent_jogs():
    jogs, link, control = JogCommands(), Link(), state()
    jogs.request("gripper", 0.03, control, 0.5)
    jogs.clear()
    jogs.flush(link, control)
    assert link.sent == []


def test_rejected_target_is_not_retried_until_new_keypress():
    jogs, link, control = JogCommands(), Link(), state()
    jogs.request("gripper", 0.03, control, 0.5)
    jogs.flush(link, control)
    link.acks[1] = {"ok": False}
    jogs.flush(link, control)
    jogs.flush(link, control)
    assert len(link.sent) == 1


def test_hold_repeat_accumulates_to_calibration_boundary_without_relative_cap():
    jogs, control = JogCommands(), state()
    for _ in range(100):
        jogs.request("gripper", 0.03, control, 0.5)
    assert jogs.desired["gripper"] == control["limits"]["gripper"][1]


def test_outside_bound_jog_is_gradual_inward_and_never_snaps_or_moves_outward():
    jogs, control = JogCommands(), state()
    control["target_qpos"]["gripper"] = -0.22
    jogs.request("gripper", -0.03, control, -0.22)
    assert not jogs.desired
    jogs.request("gripper", 0.03, control, -0.22)
    assert abs(jogs.desired["gripper"] - (-0.19)) < 1e-9
    control["target_qpos"]["gripper"] = 1.8
    jogs.clear()
    jogs.request("gripper", 0.03, control, 1.8)
    assert not jogs.desired
    jogs.request("gripper", -0.03, control, 1.8)
    assert jogs.desired["gripper"] == 1.77
