"""Operator-facing motor fault descriptions, using Feetech SDK error bits."""


def motor_fault_text(status):
    names = ((1, "input voltage error"), (2, "position sensor error"), (4, "overheating"),
             (8, "overcurrent"), (32, "overload"))
    reasons = [name for bit, name in names if status & bit]
    if status & ~47:
        reasons.append("unknown fault bit")
    return ", ".join(reasons) or "normal"
