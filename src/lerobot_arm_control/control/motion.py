"""Small, reversible ID-6 gripper tests using raw encoder units."""

import time

from lerobot_arm_control.core.config import ArmConfig

MAX_DELTA = 32  # STS3215: 4096 ticks per revolution; 32 ticks is about 2.8 degrees.
MIN_DELTA = 16  # Smaller steps did not reliably overcome this gripper's startup deadband.
# Hardware settled 4 ticks short. This tolerance does not require PID tuning.
POSITION_TOLERANCE = 5


def plan_motion(arm: ArmConfig, bus, delta: int) -> dict:
    if arm.role != "follower" or arm.motors.get("gripper") != 6:
        raise ValueError("Motion is supported only on a confirmed follower gripper at ID 6")
    if type(delta) is not int or not MIN_DELTA <= abs(delta) <= MAX_DELTA:
        raise ValueError(f"absolute delta must be an integer in {MIN_DELTA}..{MAX_DELTA} ticks")

    def read(register):
        return int(bus.read(register, "gripper", normalize=False))

    if read("Operating_Mode") != 0:
        raise ValueError("Gripper must already be in position mode; no mode changes are made")
    if read("Status"):
        raise ValueError("Gripper reports a fault; motion must not start")
    start = read("Present_Position")
    original_goal = read("Goal_Position")
    torque = read("Torque_Enable")
    moving = read("Moving")
    low, high = read("Min_Position_Limit"), read("Max_Position_Limit")
    if not 0 <= low < high <= 4095:
        raise ValueError("Position limits are invalid; calibrate limits first")
    target = start + delta
    if not low + 8 <= min(start, target) <= max(start, target) <= high - 8:
        raise ValueError("Start or target is too close to a configured position limit")
    if torque not in (0, 1) or moving:
        raise ValueError("Unexpected torque state or gripper is already moving")
    if torque and abs(original_goal - start) > 8:
        raise ValueError("Gripper is not settled near its current goal")
    return {
        "joint": "gripper",
        "motor_id": 6,
        "start_raw": start,
        "target_raw": target,
        "delta_raw": delta,
        "original_goal_raw": original_goal,
        "original_torque": torque,
        "min_raw": low,
        "max_raw": high,
    }


def wait_for_position(bus, target: int, timeout: float = 2.0, *, observer=None,
                      phase="settling") -> int:
    deadline = time.monotonic() + timeout
    position = moving = None
    settled_samples = 0
    while time.monotonic() < deadline:
        position = int(bus.read("Present_Position", "gripper", normalize=False))
        moving = int(bus.read("Moving", "gripper", normalize=False))
        status = int(bus.read("Status", "gripper", normalize=False))
        if status:
            raise RuntimeError(f"Gripper fault status: {status}")
        if observer:
            observer(phase, target)
        if abs(position - target) <= POSITION_TOLERANCE and not moving:
            settled_samples += 1
            if settled_samples >= 3:
                return position
        else:
            settled_samples = 0
        time.sleep(0.05)
    raise TimeoutError(
        f"Gripper did not settle at {target}; feedback={position}, moving={moving}, "
        f"tolerance={POSITION_TOLERANCE} ticks"
    )


def execute_motion(bus, plan: dict, *, observer=None) -> dict:
    def write(register, value):
        bus.write(register, "gripper", value, normalize=False)

    def wait(target, phase):
        if observer is None:
            return wait_for_position(bus, target)
        return wait_for_position(bus, target, observer=observer, phase=phase)

    completed = False
    try:
        # Goal writes can themselves enable torque on this hardware: only write a safe goal.
        write("Goal_Position", plan["start_raw"])
        if not plan["original_torque"]:
            write("Torque_Enable", 1)
        settled_start = wait(plan["start_raw"], "start")
        step = 2 if plan["delta_raw"] > 0 else -2
        for target in range(plan["start_raw"] + step, plan["target_raw"], step):
            write("Goal_Position", target)
            time.sleep(0.08)
            if observer:
                observer("outbound", target)
        write("Goal_Position", plan["target_raw"])
        reached = wait(plan["target_raw"], "target")
        actual_delta = reached - settled_start
        if actual_delta * plan["delta_raw"] <= 0 or abs(actual_delta) < abs(plan["delta_raw"]) / 2:
            raise RuntimeError(f"Insufficient measured movement: {actual_delta} ticks")
        write("Goal_Position", plan["start_raw"])
        returned = wait(plan["start_raw"], "return")
        completed = True
        return {
            **plan,
            "settled_start_raw": settled_start,
            "reached_raw": reached,
            "returned_raw": returned,
            "actual_delta_raw": actual_delta,
            "position_tolerance_raw": POSITION_TOLERANCE,
            "success": True,
        }
    finally:
        # On failure/interrupt, release only the tested gripper; do not attempt blind return motion.
        if not completed:
            write("Torque_Enable", 0)
        elif plan["original_torque"]:
            write("Goal_Position", plan["original_goal_raw"])
            write("Torque_Enable", 1)
        else:
            # Do not restore a stale goal (which may be outside calibrated limits) on a limp motor.
            # The goal remains at the safe return position. Torque must be the LAST write.
            write("Torque_Enable", 0)
        expected = plan["original_torque"] if completed else 0
        actual = int(bus.read("Torque_Enable", "gripper", normalize=False))
        if actual != expected:
            raise RuntimeError(f"Torque cleanup failed: expected {expected}, read {actual}")
