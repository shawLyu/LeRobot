"""Small versioned JSON-lines protocol shared by both ends of the SSH link."""

import math

from lerobot_arm_control.robot.so101.joints import JOINTS

VERSION = 1
WATCHDOG_SECONDS = 0.75


def checked_command(link, acknowledgements, operation, *, fallback_error="command rejected",
                    **payload):
    """Record the acknowledgement before rejecting a failed worker command."""
    ack = link.wait_ack(link.send(operation, **payload))
    acknowledgements.append(ack)
    if not ack["ok"]:
        raise RuntimeError(ack.get("error", fallback_error))
    return ack


def preview_joint_target(bus, joint, delta):
    """Raw encoder preview for a single joint, using live feedback and EEPROM limits."""
    if joint not in JOINTS or type(delta) is not int or not 1 <= abs(delta) <= 32:
        raise ValueError("joint preview requires a known joint and 1..32 encoder ticks")
    if int(bus.read("Status", joint, normalize=False)):
        raise ValueError("joint reports a fault")
    start = int(bus.read("Present_Position", joint, normalize=False))
    low = int(bus.read("Min_Position_Limit", joint, normalize=False))
    high = int(bus.read("Max_Position_Limit", joint, normalize=False))
    target = start + delta
    if not 0 <= low < high <= 4095 or not low + 8 <= target <= high - 8:
        raise ValueError("preview target too close to configured position limits")
    return {"preview": True, "joint": joint, "start_raw": start, "target_raw": target,
            "delta_raw": delta, "executed": False}


def validate_target(values, current, ranges):
    if not isinstance(values, list) or len(values) != len(JOINTS):
        raise ValueError("target must contain six joint angles in radians")
    for value, bounds in zip(values, ranges, strict=True):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("target contains a non-finite angle")
    return [max(low, min(high, value)) for value, (low, high) in zip(values, ranges, strict=True)]


def raw_to_angles(raw, limits, model_ranges):
    """Uncalibrated display estimate using EEPROM limits, never used for motion."""
    result = []
    for name, bounds in zip(JOINTS, model_ranges, strict=True):
        low, high = limits[name]
        if high <= low:
            raise ValueError(f"invalid encoder limits: {name}")
        fraction = (raw[name] - low) / (high - low)
        result.append(bounds[0] + fraction * (bounds[1] - bounds[0]))
    return result
