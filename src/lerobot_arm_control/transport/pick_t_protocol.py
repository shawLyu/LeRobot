"""Pick/place stage checks shared by the local policy and remote executor; no physics."""

import copy
import hashlib
import json
import math

from lerobot_arm_control.tasks.push_t.push_t_motor import observation_issue

STAGES = ('prepare', 'approach', 'descend', 'close', 'lift', 'transfer', 'lower', 'open', 'retreat')
CARRYING = ('lift', 'transfer', 'lower', 'open', 'retreat')


def segment_digest(segments):
    encoded = json.dumps(segments, sort_keys=True, allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def pick_observation_issue(state, received, now, stage, reference=None):
    if stage not in STAGES:
        return 'unknown pick/place stage'
    if state is None:
        return 'waiting for pick/place observations'
    checked = copy.deepcopy(state)
    if stage in CARRYING and reference:
        # A carried object leaves the table plane and is often hidden by the fingers.
        # Keep the original grasp reference, while checking live camera/table/motors/base/target.
        checked.setdefault('objects', {})['wood_t'] = reference['objects']['wood_t']
    elif stage in ('descend', 'close') and reference:
        wood = checked.get('objects', {}).get('wood_t', {})
        if not wood.get('observed'):
            if not 0 <= wood.get('observation_age_s', math.inf) <= 2:
                return 'grasp occlusion exceeded two seconds'
            checked['objects']['wood_t'] = reference['objects']['wood_t']
    issue = observation_issue(checked, received, now, require_base=True, initialized=True)
    if issue:
        base = checked.get('robot', {}).get('registration', {})
        if base.get('status') == 'pose_rejected':
            return (f"{issue}; base mount fit {base['reprojection_error_px']:.2f} > "
                    f"{base['max_reprojection_error_px']:.2f} processing pixels: "
                    f"{base['rejection_reason']}")
        return issue
    base = checked['robot']['registration']
    if not (base.get('fixed_to_table') and base.get('status') == 'fixed'):
        return 'pick/place requires a saved fixed base in the current table frame'
    if stage in ('prepare', 'approach'):
        wood = checked['objects']['wood_t']
        if not wood.get('observed') or wood.get('tracking_status') != 'observed':
            return 'pick/place requires a complete visible T before closing'
    return None


def stage_segments_issue(stage, segments):
    """Opening/closing changes only the jaw; other stages keep the jaw constant."""
    if stage not in STAGES:
        return 'invalid pick/place stage'
    for segment in segments:
        delta = [b-a for a, b in zip(segment['start'], segment['end'], strict=True)]
        if stage in ('prepare', 'close', 'open'):
            if max(map(abs, delta[:5])) > 1e-8:
                return 'jaw stages cannot move the arm joints'
            if (stage == 'close' and delta[5] > 1e-8) or (
                    stage in ('prepare', 'open') and delta[5] < -1e-8):
                return 'jaw stage moves in the wrong direction'
        elif abs(delta[5]) > 1e-8:
            return 'arm stages cannot change the jaw opening'
        if stage != 'approach' and segment['speed_rad_s'] > .3:
            return 'pick/place contact or carrying speed exceeds 0.3 rad/s'
    if sum(s['seconds'] for s in segments) > 30:
        return 'pick/place stage exceeds 30 seconds'
    return None


def placement_error(state, target):
    """Final acceptance uses a newly visible T, not encoder arrival or an old prediction."""
    obj = state.get('objects', {}).get('wood_t', {})
    if (state.get('units') != 'm' or not state.get('marker_visible') or not obj.get('pose_valid')
            or not obj.get('observed') or obj.get('tracking_status') != 'observed'):
        return None
    dx, dy = (obj['xy'][i]-target['xy'][i] for i in (0, 1))
    angle = obj['yaw']-target['yaw']
    return math.hypot(dx, dy), abs(math.atan2(math.sin(angle), math.cos(angle)))
