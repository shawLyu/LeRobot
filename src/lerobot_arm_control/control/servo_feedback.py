"""Two bounded STS3215 RAM reads using the installed Feetech sync reader."""

from lerobot_arm_control.control.control_defaults import READ_RETRIES

FIELDS = ("Torque_Enable", "Acceleration", "Goal_Position", "Present_Position",
          "Present_Velocity", "Present_Load", "Present_Voltage", "Status", "Moving",
          "Present_Current")
SPANS = ((40, 4), (56, 15))


def read_control_feedback(bus, num_retry=READ_RETRIES):
    ids = [motor.id for motor in bus.motors.values()]
    fields = {}
    for start, size in SPANS:
        for attempt in range(num_retry + 1):
            bus._setup_sync_reader(ids, start, size)
            result = bus.sync_reader.txRxPacket()
            available = bus._is_comm_success(result) and all(
                bus.sync_reader.isAvailable(motor_id, start, size) for motor_id in ids)
            if available:
                break
            if attempt == num_retry:
                raise ConnectionError("control snapshot failed after read retries: "
                                      + bus.packet_handler.getTxRxResult(result))
        for field in FIELDS:
            values = {}
            for name, motor in bus.motors.items():
                address, length = bus.model_ctrl_table[motor.model][field]
                if start <= address and address + length <= start + size:
                    values[motor.id] = bus.sync_reader.getData(motor.id, address, length)
            if values:
                if len(values) != len(ids):
                    raise ValueError(f"{field}: incompatible STS3215 register table")
                values = bus._decode_sign(field, values)
                fields[field] = {name: int(values[motor.id]) for name, motor in bus.motors.items()}
    if set(fields) != set(FIELDS):
        raise ValueError("incompatible STS3215 feedback table")
    return fields
