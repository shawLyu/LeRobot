from types import SimpleNamespace

import pytest

from arm_control.servo_feedback import FIELDS, read_control_feedback

TABLE = {"Torque_Enable": (40, 1), "Acceleration": (41, 1), "Goal_Position": (42, 2),
         "Present_Position": (56, 2), "Present_Velocity": (58, 2), "Present_Load": (60, 2),
         "Present_Voltage": (62, 1), "Status": (65, 1), "Moving": (66, 1),
         "Present_Current": (69, 2)}


class Reader:
    def __init__(self):
        self.calls = []
        self.available = True
        self.result = 0
        self.memory = {1: bytearray(71), 2: bytearray(71)}

    def txRxPacket(self):
        self.calls.append(self.span)
        return self.result

    def isAvailable(self, motor_id, address, size):
        return self.available

    def getData(self, motor_id, address, size):
        start, length = self.span
        assert start <= address < address + size <= start + length
        return int.from_bytes(self.memory[motor_id][address:address + size], "little")


class Bus:
    def __init__(self):
        self.motors = {name: SimpleNamespace(id=i, model="sts3215")
                       for i, name in enumerate(("elbow_flex", "gripper"), 1)}
        self.model_ctrl_table = {"sts3215": dict(TABLE)}
        self.sync_reader = Reader()
        self.packet_handler = SimpleNamespace(getTxRxResult=lambda result: f"error {result}")

    def _setup_sync_reader(self, ids, start, size):
        assert ids == [1, 2]
        self.sync_reader.span = (start, size)

    def _is_comm_success(self, result):
        return result == 0

    def _decode_sign(self, field, values):
        bit = {"Goal_Position": 15, "Present_Position": 15,
               "Present_Velocity": 15, "Present_Load": 10}.get(field)
        return {i: -(v & ~(1 << bit)) if bit is not None and v & (1 << bit) else v
                for i, v in values.items()}


def test_two_small_blocks_decode_goal_health_and_signed_feedback():
    bus = Bus()
    expected = {field: {"elbow_flex": 0, "gripper": 0} for field in FIELDS}
    for field, raw in (("Torque_Enable", 1), ("Acceleration", 254), ("Goal_Position", 2910),
                       ("Present_Position", 2953), ("Present_Velocity", 0x8005),
                       ("Present_Load", 0x405), ("Present_Voltage", 52), ("Status", 1),
                       ("Moving", 1), ("Present_Current", 2)):
        address, length = TABLE[field]
        bus.sync_reader.memory[1][address:address + length] = raw.to_bytes(length, "little")
        expected[field]["elbow_flex"] = -5 if field in ("Present_Load", "Present_Velocity") else raw
    assert read_control_feedback(bus) == expected
    assert bus.sync_reader.calls == [(40, 4), (56, 15)]


@pytest.mark.parametrize("cause", ["communication", "missing_motor", "table"])
def test_bad_feedback_is_rejected_instead_of_becoming_zero_positions(cause):
    bus = Bus()
    if cause == "communication":
        bus.sync_reader.result = -1
    elif cause == "missing_motor":
        bus.sync_reader.available = False
    else:
        bus.model_ctrl_table["sts3215"]["Present_Position"] = (100, 2)
    with pytest.raises((ConnectionError, ValueError)):
        read_control_feedback(bus)


def test_transient_packet_errors_use_two_official_extra_attempts():
    bus = Bus()
    original = bus.sync_reader.txRxPacket
    remaining = [2]

    def transient():
        result = original()
        if remaining[0]:
            remaining[0] -= 1
            return -1
        return result

    bus.sync_reader.txRxPacket = transient
    assert set(read_control_feedback(bus)) == set(FIELDS)
    assert bus.sync_reader.calls == [(40, 4)] * 3 + [(56, 15)]


def test_persistent_packet_error_stops_after_three_attempts():
    bus = Bus()
    bus.sync_reader.result = -1
    with pytest.raises(ConnectionError, match="after read retries"):
        read_control_feedback(bus)
    assert bus.sync_reader.calls == [(40, 4)] * 3
