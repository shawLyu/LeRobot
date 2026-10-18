"""Exclusive bus discovery, retaining healthy packets that contain device alarms."""

from contextlib import contextmanager
from dataclasses import replace

from lerobot_arm_control.robot.connection import make_bus, serial_owner
from lerobot_arm_control.robot.feetech import require_sts3215
from lerobot_arm_control.workbench.registers import BAUDRATES, install_table


class ServoAccess:
    def __init__(self, bus):
        self.bus = bus

    def __getattr__(self, name):
        return getattr(self.bus, name)

    def read(self, key, joint, *, normalize=False, **kwargs):
        motor = self.motors[joint]
        address, size = self.model_ctrl_table[motor.model][key]
        value, comm, _ = self.bus._read(address, size, motor.id, raise_on_error=False)
        if not self.bus._is_comm_success(comm):
            raise ConnectionError(self.packet_handler.getTxRxResult(comm))
        return int(self.bus._decode_sign(key, {motor.id: value})[motor.id])

    def write(self, key, joint, value, *, normalize=False, **kwargs):
        motor = self.motors[joint]
        address, size = self.model_ctrl_table[motor.model][key]
        encoded = self.bus._encode_sign(key, {motor.id: value})[motor.id]
        comm, _ = self.bus._write(address, size, motor.id, encoded, raise_on_error=False)
        if not self.bus._is_comm_success(comm):
            raise ConnectionError(self.packet_handler.getTxRxResult(comm))

    def set_id(self, joint, motor_id):
        self.bus.motors[joint].id = motor_id

    def id_available(self, motor_id):
        _, comm, _ = self.bus.packet_handler.ping(self.bus.port_handler, motor_id)
        return not self.bus._is_comm_success(comm)

    def set_baudrate(self, value):
        if not self.bus.port_handler.setBaudRate(value):
            raise ConnectionError('串口无法切换到新波特率')


@contextmanager
def open_hardware(arm, ids, baudrate):
    require_sts3215(arm)
    if baudrate not in BAUDRATES:
        raise ValueError('不支持该波特率')
    if (len(ids) != len(arm.motors) or len(set(ids)) != len(ids)
            or any(type(i) is not int or not 1 <= i <= 253 for i in ids)):
        raise ValueError('需要六个互不重复的舵机 ID（1..253）')
    arm = replace(arm, motors=dict(zip(arm.motors, ids, strict=True)))
    with serial_owner(arm.port):
        bus = make_bus(arm)
        try:
            bus.connect(handshake=False)
            bus.port_handler.ser.exclusive = True
            access = ServoAccess(bus)
            access.set_baudrate(baudrate)
            online, discovery = {}, {}
            for joint, motor in bus.motors.items():
                model, comm, alarm = bus.packet_handler.ping(bus.port_handler, motor.id)
                healthy_packet = bus._is_comm_success(comm)
                found = healthy_packet and model == 777
                discovery[joint] = {'id': motor.id, 'online': found,
                                    'model': model if healthy_packet else None,
                                    'packet_alarm': alarm if healthy_packet else None}
                if found:
                    online[joint] = motor
            bus.motors = online
            install_table(bus)
            yield access, discovery
        finally:
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
