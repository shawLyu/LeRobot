"""Shared metric bounds for the display patch and its photograph-derived texture."""


def workspace_bounds(state, config):
    half = (config["marker_size_m"] or 1.0) / 2
    points = [[-half, -half], [half, half]]
    for obj in state["objects"].values():
        points.extend(obj.get("complete_outline_xy", [obj["xy"]]))
    for corners in state.get("secondary_layouts", {}).values():
        points.extend(corner[:2] for corner in corners)
    for index, entry in enumerate([config.get('robot', {}), *config.get('additional_robots', [])]):
        if not entry.get('enabled'):
            continue
        registration = state.get('base_registrations', {}).get(entry.get('arm'), {})
        if index == 0:
            registration = state.get('robot', {}).get('registration', registration)
        base = registration.get('position', [0, .32, .0024] if index == 0 else None)
        if base is not None:
            points.extend([[base[0]-.20, base[1]-.20], [base[0]+.20, base[1]+.20]])
    margin = (config["marker_size_m"] or 1.0) * 0.35
    return [
        min(p[0] for p in points) - margin,
        max(p[0] for p in points) + margin,
        min(p[1] for p in points) - margin,
        max(p[1] for p in points) + margin,
    ]
