"""Check native settings and apply the shared controller RAM acceleration."""

import json
import time
from pathlib import Path

from lerobot_arm_control.control.control_defaults import CONTROL, DEFAULTS, POSITION_FEEDBACK
from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.robot.so101.driver import connected

ROOT = Path(__file__).resolve().parents[1]


def configure():
    acceleration = POSITION_FEEDBACK["native_stream"]["acceleration"]
    expected = {"Return_Delay_Time": DEFAULTS["return_delay_time"],
                "Maximum_Acceleration": DEFAULTS["maximum_acceleration"],
                "Operating_Mode": DEFAULTS["operating_mode"],
                "P_Coefficient": DEFAULTS["position_p_coefficient"],
                "I_Coefficient": DEFAULTS["position_i_coefficient"],
                "D_Coefficient": DEFAULTS["position_d_coefficient"]}
    gripper = {"Max_Torque_Limit": DEFAULTS["gripper_max_torque_limit"],
               "Protection_Current": DEFAULTS["gripper_protection_current"],
               "Overload_Torque": DEFAULTS["gripper_overload_torque"]}
    with connected(load_arms()["follower"]) as bus:
        actual = {key: bus.sync_read(key, normalize=False) for key in expected}
        mismatches = {key: values for key, values in actual.items()
                      if any(value != expected[key] for value in values.values())}
        actual_gripper = {key: bus.read(key, "gripper", normalize=False) for key in gripper}
        if mismatches or actual_gripper != gripper:
            raise ValueError("Native settings differ; support the arm before EEPROM configuration: "
                             + json.dumps({"motors": mismatches, "gripper": actual_gripper}))
        phase = bus.sync_read("Phase", normalize=False)
        if any(value & 0x10 for value in phase.values()):
            raise ValueError("Native angle-feedback mode differs; "
                             "preserve calibration before configuring")
        before = bus.sync_read("Acceleration", normalize=False)
        changed = {name: acceleration for name, value in before.items()
                   if value != acceleration}
        if changed:
            bus.sync_write("Acceleration", changed, normalize=False)
        after = bus.sync_read("Acceleration", normalize=False)
        if any(value != acceleration for value in after.values()):
            raise RuntimeError("Native acceleration readback failed")
        proof = {"time_unix_ns": time.time_ns(), "control": CONTROL,
                 "native_settings": actual, "gripper_settings": actual_gripper,
                 "phase": phase, "acceleration_before": before, "acceleration_after": after,
                 "ram_acceleration_writes": changed, "eeprom_writes": 0,
                 "goal_position_writes": 0, "torque_writes": 0,
                 "torque_after": bus.sync_read("Torque_Enable", normalize=False)}
    path = ROOT / f"logs/control-settings-{proof['time_unix_ns']}.json"
    path.write_text(json.dumps(proof, indent=2) + "\n")
    print(json.dumps({"ok": True, "proof": str(path), "ram_acceleration_writes": changed}))


if __name__ == "__main__":
    configure()
