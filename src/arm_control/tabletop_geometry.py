"""Shared metric bounds for the display patch and its photograph-derived texture."""


def workspace_bounds(state, config):
    half = (config["marker_size_m"] or 1.0) / 2
    points = [[-half, -half], [half, half]]
    for obj in state["objects"].values():
        points.extend(obj.get("complete_outline_xy", [obj["xy"]]))
    for corners in state.get("secondary_layouts", {}).values():
        points.extend(corner[:2] for corner in corners)
    if config.get("robot", {}).get("enabled"):
        base = state.get("robot", {}).get("registration", {}).get("position", [0, 0.32, 0.0024])
        points.extend([[base[0] - 0.20, base[1] - 0.20], [base[0] + 0.20, base[1] + 0.20]])
    margin = (config["marker_size_m"] or 1.0) * 0.35
    return [
        min(p[0] for p in points) - margin,
        max(p[0] for p in points) + margin,
        min(p[1] for p in points) - margin,
        max(p[1] for p in points) + margin,
    ]
