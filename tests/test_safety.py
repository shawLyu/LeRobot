"""Safety boundaries are tested without connected hardware."""

from types import SimpleNamespace

import pytest

from lerobot_arm_control.control import motion
from lerobot_arm_control.core.config import ArmConfig
from lerobot_arm_control.robot.so101 import driver


def test_serial_bus_reserved_before_any_second_connection(tmp_path, monkeypatch):
    port = tmp_path/'serial-device'
    alias = tmp_path/'by-id'
    alias.symlink_to(port)
    with driver.serial_owner(str(port)):
        with pytest.raises(RuntimeError, match='already owned'):
            with driver.serial_owner(str(alias)):
                pytest.fail('A second worker acquired the same device')
    with driver.serial_owner(str(alias)):
        pass  # Closing the owner releases the reservation, including on failure.


@pytest.fixture
def arm():
    return ArmConfig(
        "follower", "/dev/serial/by-id/test", "follower", "sts3215", 1_000_000, {"gripper": 6}
    )


class FakeBus:
    def __init__(self, torque=0, goal_enables_torque=False):
        self.goal_enables_torque = goal_enables_torque
        self.values = {
            "Operating_Mode": 0,
            "Present_Position": 2000,
            "Goal_Position": 1900 if not torque else 2000,
            "Torque_Enable": torque,
            "Moving": 0,
            "Status": 0,
            "Min_Position_Limit": 1000,
            "Max_Position_Limit": 3000,
        }
        self.writes = []

    def read(self, register, joint, *, normalize):
        assert joint == "gripper" and normalize is False
        return self.values[register]

    def write(self, register, joint, value, *, normalize):
        assert joint == "gripper" and normalize is False
        self.writes.append((register, value))
        self.values[register] = value
        if register == "Goal_Position" and self.goal_enables_torque:
            self.values["Torque_Enable"] = 1
        if self.values["Torque_Enable"]:
            self.values["Present_Position"] = self.values["Goal_Position"]


@pytest.mark.parametrize("delta", [0, 1, -15, 33, -33, 2.5])
def test_out_of_range_motion_is_rejected(arm, delta):
    bus = FakeBus()
    with pytest.raises(ValueError):
        motion.plan_motion(arm, bus, delta)
    assert not bus.writes


@pytest.mark.parametrize("role,id_", [("unassigned", 6), ("leader", 6), ("follower", 5)])
def test_wrong_role_or_motor_is_rejected(arm, role, id_):
    invalid = ArmConfig(arm.name, arm.port, role, arm.model, arm.baudrate, {"gripper": id_})
    with pytest.raises(ValueError):
        motion.plan_motion(invalid, FakeBus(), 32)


@pytest.mark.parametrize(
    "register,value",
    [
        ("Operating_Mode", 1),
        ("Status", 1),
        ("Moving", 1),
        ("Present_Position", 2995),
        ("Min_Position_Limit", 3000),
    ],
)
def test_unsafe_motor_state_is_rejected(arm, register, value):
    bus = FakeBus()
    bus.values[register] = value
    with pytest.raises(ValueError):
        motion.plan_motion(arm, bus, 32)
    assert not bus.writes


def test_dry_run_only_reads(arm):
    bus = FakeBus()
    plan = motion.plan_motion(arm, bus, 32)
    assert plan["target_raw"] == 2032
    assert not bus.writes


@pytest.mark.parametrize("torque", [0, 1])
@pytest.mark.parametrize("auto_enable", [False, True])
def test_motion_returns_and_restores_initial_state(arm, torque, auto_enable, monkeypatch):
    bus = FakeBus(torque, goal_enables_torque=auto_enable)
    plan = motion.plan_motion(arm, bus, 32)
    monkeypatch.setattr(motion.time, "sleep", lambda _: None)
    result = motion.execute_motion(bus, plan)
    assert result["reached_raw"] == 2032
    assert result["returned_raw"] == 2000
    assert bus.values["Torque_Enable"] == torque
    expected_goal = plan["original_goal_raw"] if torque else plan["start_raw"]
    assert bus.values["Goal_Position"] == expected_goal
    assert bus.writes[-1] == ("Torque_Enable", torque)
    assert bus.writes[0] == ("Goal_Position", 2000)
    if torque == 0:
        assert bus.writes[1] == ("Torque_Enable", 1)
    assert {register for register, _ in bus.writes} <= {"Goal_Position", "Torque_Enable"}


@pytest.mark.parametrize("failure", [TimeoutError, KeyboardInterrupt])
def test_motion_failure_releases_only_gripper(arm, monkeypatch, failure):
    bus = FakeBus()
    plan = motion.plan_motion(arm, bus, 32)
    calls = 0

    def wait(bus, target):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise failure()
        return target

    monkeypatch.setattr(motion, "wait_for_position", wait)
    monkeypatch.setattr(motion.time, "sleep", lambda _: None)
    with pytest.raises(failure):
        motion.execute_motion(bus, plan)
    assert bus.writes[-1] == ("Torque_Enable", 0)


def test_no_actual_motion_cannot_pass(arm, monkeypatch):
    bus = FakeBus()
    plan = motion.plan_motion(arm, bus, 32)
    monkeypatch.setattr(motion, "wait_for_position", lambda *args: 2000)
    monkeypatch.setattr(motion.time, "sleep", lambda _: None)
    with pytest.raises(RuntimeError, match="Insufficient measured movement"):
        motion.execute_motion(bus, plan)
    assert bus.writes[-1] == ("Torque_Enable", 0)


def test_feedback_failure_stops_and_releases_gripper(arm, monkeypatch):
    bus = FakeBus(goal_enables_torque=True)
    plan = motion.plan_motion(arm, bus, 32)
    monkeypatch.setattr(motion.time, "sleep", lambda _: None)

    def disconnected(phase, target):
        raise RuntimeError("SSH disconnected")

    with pytest.raises(RuntimeError, match="SSH disconnected"):
        motion.execute_motion(bus, plan, observer=disconnected)
    assert bus.values["Torque_Enable"] == 0
    assert bus.values["Present_Position"] == plan["start_raw"]
    assert bus.writes[-1] == ("Torque_Enable", 0)


def test_feedback_within_tolerance_must_be_settled(arm, monkeypatch):
    bus = FakeBus()
    bus.values["Present_Position"] = 2004
    monkeypatch.setattr(motion.time, "sleep", lambda _: None)
    assert motion.wait_for_position(bus, 2000) == 2004
    bus.values["Status"] = 1
    with pytest.raises(RuntimeError, match="fault status"):
        motion.wait_for_position(bus, 2000)


@pytest.mark.parametrize("model", [777, 999])
def test_read_session_never_disables_torque(arm, monkeypatch, model):
    events = []

    class ReadBus:
        def __init__(self, **kwargs):
            self.is_connected = False
            self.port_handler = SimpleNamespace(ser=SimpleNamespace(exclusive=False))

        def connect(self, handshake):
            assert handshake is False
            self.is_connected = True

        def ping(self, name, raise_on_error):
            return model

        def disconnect(self, disable_torque):
            events.append(("disconnect", disable_torque))
            self.is_connected = False

    monkeypatch.setattr(driver, "make_bus", lambda arm: ReadBus())
    if model == 777:
        with driver.connected(arm):
            pass
    else:
        with pytest.raises(RuntimeError):
            with driver.connected(arm):
                pass
    assert events == [("disconnect", False)]


@pytest.mark.parametrize('allow_alarm,model,comm,error,expected', [
    (False, 777, 0, 32, RuntimeError),
    (True, 777, 0, 32, None),
    (True, 999, 0, 32, RuntimeError),
    (True, 777, -1, 32, ConnectionError),
    (True, 777, 0, 0, RuntimeError),
])
def test_alarm_monitor_still_checks_identity_and_transport_without_writes(
        arm, monkeypatch, allow_alarm, model, comm, error, expected):
    events = []

    class AlarmBus:
        is_connected = False
        port_handler = SimpleNamespace(ser=SimpleNamespace(exclusive=False))
        motors = {'gripper': SimpleNamespace(id=6)}

        def __init__(self):
            def ping(port, motor_id):
                assert port is self.port_handler and motor_id == 6
                events.append('packet ping')
                return model, comm, error
            self.packet_handler = SimpleNamespace(
                ping=ping, getTxRxResult=lambda _: 'bad transport')

        def connect(self, handshake):
            assert handshake is False
            self.is_connected = True

        def ping(self, name, raise_on_error):
            raise RuntimeError('[RxPacketError] Overload error!')

        def _is_comm_success(self, result):
            return result == 0

        def disconnect(self, disable_torque):
            assert disable_torque is False
            events.append('disconnect')

    monkeypatch.setattr(driver, 'make_bus', lambda _: AlarmBus())
    if expected:
        with pytest.raises(expected):
            with driver.connected(arm, allow_alarm=allow_alarm):
                pytest.fail('Invalid hardware session accepted')
    else:
        with driver.connected(arm, allow_alarm=True):
            pass
    assert events == (['packet ping', 'disconnect'] if allow_alarm else ['disconnect'])
