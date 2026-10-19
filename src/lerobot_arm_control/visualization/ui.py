"""Operator-facing live status and motor fault descriptions."""


def tool_marker_text(marker):
    edge = marker.get('source_short_edge_px')
    size = f'{edge:.1f} px' if edge is not None else 'unavailable'
    return (f"Tool ID {marker['id']}: {marker['status']} | edge {size}"
            '\nTool precision: unvalidated')


def motor_fault_text(status):
    names = ((1, "input voltage error"), (2, "position sensor error"), (4, "overheating"),
             (8, "overcurrent"), (32, "overload"))
    reasons = [name for bit, name in names if status & bit]
    if status & ~47:
        reasons.append("unknown fault bit")
    return ", ".join(reasons) or "normal"
