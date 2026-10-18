"""Shared calibrated position bus for hardware-free controller tests."""

from lerobot_arm_control.transport.realtime import JOINTS


class Calibration:
    def count(self):
        return 6

    def fit(self, joint):
        return {"zero_raw": 2000, "radians_per_tick": 0.0015}


class Bus:
    def __init__(self):
        self.registers = {name: {"Present_Position": 2200, "Goal_Position": 1000,
                                 "Torque_Enable": 0, "Status": 0, "Operating_Mode": 0,
                                 "Present_Voltage": 52, "Min_Voltage_Limit": 40,
                                 "Max_Voltage_Limit": 90, "Acceleration": 0,
                                 "Maximum_Acceleration": 254, "Goal_Time": 0,
                                 "Goal_Velocity": 0, "Maximum_Velocity_Limit": 1000,
                                 "Velocity_Unit_factor": 1, "Lock": 1,
                                 "P_Coefficient": 16, "I_Coefficient": 0}
                          for name in JOINTS}
        self.writes = []

    def read(self, register, joint, normalize=False, num_retry=0):
        return self.registers[joint][register]

    def sync_read(self, register, normalize=False):
        return {joint: values[register] for joint, values in self.registers.items()}

    def sync_write(self, register, values, normalize=False):
        for joint, value in values.items():
            self.write(register, joint, value, normalize)

    def write(self, register, joint, value, normalize=False):
        self.writes.append((register, joint, value))
        self.registers[joint][register] = value
        if register == "Goal_Position":
            self.registers[joint]["Torque_Enable"] = 1
