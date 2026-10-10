"""Coalesce keyboard jogs without losing presses to stale feedback frames."""


class ResetThenCircle:
    """Wait for the acknowledged remote Reset without blocking live rendering."""

    def __init__(self, link, laps=None):
        self.laps = laps
        self.pending = link.send("reset")
        self.notice = "Circle pending: Reset first"

    def cancel(self):
        self.pending = None
        self.notice = "Pending circle cancelled"

    def advance(self, link, state, age):
        if self.pending is None:
            return
        if age >= .5:
            self.cancel()
            link.send("hold")
            return
        with link.lock:
            ack = link.acks.get(self.pending)
        if ack is None:
            return
        if not ack["ok"]:
            self.notice = "Circle cancelled: " + ack.get("error", "Reset rejected")
            self.pending = None
            return
        control = state["control"]
        # An ACK can arrive before its corresponding state frame. Never consume
        # a previous run's boundary-hold state as completion of this Reset.
        if (control.get("last_command_id") or 0) < self.pending:
            return
        parking = control["parking"]
        if parking["phase"] in ("boundary hold", "unpowered rest"):
            link.send("circle_start", **({"laps": self.laps} if self.laps is not None else {}))
            self.pending = None
            self.notice = "Reset complete: circle requested"
        elif parking["phase"] not in ("moving", "settling", "verifying release", "ready"):
            self.pending = None
            self.notice = "Circle cancelled: " + parking["notice"]


class JogCommands:
    def __init__(self):
        self.desired = {}
        self.pending = None

    def clear(self):
        self.desired.clear()
        self.pending = None

    def request(self, joint, delta, control, measured):
        low, high = control["limits"][joint]
        # Recover gradually from a supported model-edge pose; never snap to the boundary.
        if measured < low:
            if delta <= 0:
                return
            low = measured
        elif measured > high:
            if delta >= 0:
                return
            high = measured
        target = self.desired.get(joint, control["target_qpos"][joint]) + delta
        self.desired[joint] = max(low, min(high, target))

    def flush(self, link, control):
        if self.pending:
            command_id, joint = self.pending
            with link.lock:
                ack = link.acks.get(command_id)
            if ack is None:
                return
            if not ack["ok"]:
                self.desired.pop(joint, None)
            elif (control.get("last_command_id") or 0) < command_id:
                return
            self.pending = None
        for joint, angle in list(self.desired.items()):
            if abs(angle - control["target_qpos"][joint]) < 1e-8:
                self.desired.pop(joint)
                continue
            self.pending = (link.send("joint_target", joint=joint, angle=angle), joint)
            break
