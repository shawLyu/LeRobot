"""Use LeRobot's Feetech bus without configuration or EEPROM writes."""

import fcntl
import hashlib
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path

from lerobot_arm_control.core.config import ArmConfig


@contextmanager
def serial_owner(port):
    """Reserve the device before open/baudrate/ping can disturb another worker."""
    identity = hashlib.sha256(str(Path(port).resolve()).encode()).hexdigest()[:20]
    path = Path(tempfile.gettempdir())/f'lerobot-serial-{os.getuid()}-{identity}.lock'
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f'Motor bus {port} is already owned by another worker') from exc
        yield
    finally:
        os.close(descriptor)


def make_bus(arm: ArmConfig):
    """Load the optional LeRobot driver only when opening a hardware session."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    return FeetechMotorsBus(
        port=arm.port,
        motors={name: Motor(motor_id, arm.model, MotorNormMode.RANGE_M100_100)
                for name, motor_id in arm.motors.items()},
    )


@contextmanager
def connected(arm: ArmConfig, *, allow_alarm=False):
    with serial_owner(arm.port):
        bus = make_bus(arm)
        try:
            # The high-level SO robot connect() also configures motors; use only the bus here.
            bus.connect(handshake=False)
            bus.port_handler.ser.exclusive = True
            for name in arm.motors:
                try:
                    model = bus.ping(name, raise_on_error=True)
                except RuntimeError as exc:
                    if not allow_alarm or '[RxPacketError]' not in str(exc):
                        raise
                    # Verify identity even when the SDK's ping wrapper refuses
                    # an alarm-bearing reply. The subsequent monitor retains the
                    # device Status; no configuration or torque write occurs here.
                    model, comm, error = bus.packet_handler.ping(
                        bus.port_handler, bus.motors[name].id)
                    if not bus._is_comm_success(comm):
                        raise ConnectionError(bus.packet_handler.getTxRxResult(comm)) from exc
                    if not error:
                        raise RuntimeError('inconsistent alarm ping; retry connection') from exc
                if model != 777:
                    raise RuntimeError(f"{arm.name}/{name}: expected STS3215 (777), got {model}")
            yield bus
        finally:
            # Default disconnect disables torque and unlocks EEPROM; diagnostics must not do that.
            if bus.is_connected:
                bus.disconnect(disable_torque=False)


def read_state(bus, joint: str) -> dict:
    fields = {
        "position_raw": "Present_Position",
        "voltage_raw": "Present_Voltage",
        "temperature_c": "Present_Temperature",
        "torque_enabled": "Torque_Enable",
        "moving": "Moving",
    }
    return {
        key: int(bus.read(register, joint, normalize=False)) for key, register in fields.items()
    }
