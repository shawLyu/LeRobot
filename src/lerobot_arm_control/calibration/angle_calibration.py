"""Measured encoder-to-MJCF alignment; never writes motor registers."""

import json
import math
import statistics
from copy import deepcopy
from pathlib import Path

from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.transport.realtime import JOINTS

TICKS_PER_RADIAN = 4096 / (2 * math.pi)
PITCH_JOINTS = ('shoulder_lift', 'elbow_flex', 'wrist_flex')
NATIVE_HEIGHT_METHOD = 'fixed_jaw_height_native_pitch_scale'


def migrate_sts3215_reference(saved, fingerprint, *, unchanged_assembly_confirmed=False):
    """Explicit software-only migration; never called by the calibration loader.

    Feetech/LeRobot: Present_Position = Actual_Position - Homing_Offset.
    Preserve measured angles by translating every reference by old_offset-new_offset.
    A limit-only update changes no angle; device, direction and model must match.
    Kinematic corrections bound to the old file still require separate validation.
    """
    old = saved.get('fingerprint', {})
    mutable = {'homing_offsets', 'raw_limits'}
    if (unchanged_assembly_confirmed is not True or saved.get('version') != 1
            or old.get('model') != 'so101' or set(old) != set(fingerprint)
            or any(old[key] != fingerprint[key] for key in old.keys() - mutable)):
        raise ValueError('reference migration requires unchanged assembly, device and model')
    for identity in (old, fingerprint):
        for field in ('homing_offsets', 'raw_limits', 'phase', 'mode'):
            if set(identity.get(field, {})) != set(JOINTS):
                raise ValueError('reference migration requires six complete motor references')
        for joint in JOINTS:
            offset = identity['homing_offsets'][joint]
            bounds = identity['raw_limits'][joint]
            if (type(offset) is not int or not -2047 <= offset <= 2047
                    or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                    or any(type(v) is not int for v in bounds)
                    or not 0 <= bounds[0] < bounds[1] <= 4095
                    or identity['mode'][joint] != 0):
                raise ValueError('invalid STS3215 position-mode reference')
    result = deepcopy(saved)
    shifts = {}
    for joint in JOINTS:
        shift = old['homing_offsets'][joint] - fingerprint['homing_offsets'][joint]
        for point in result.get('points', {}).get(joint, {}).values():
            raw = point.get('raw')
            if (type(raw) not in (int, float) or not math.isfinite(raw)
                    or not 0 <= raw + shift <= 4095):
                raise ValueError('reference migration crosses the encoder wrap; recalibrate')
            point['raw'] = raw + shift
        shifts[joint] = shift
    result['fingerprint'] = deepcopy(fingerprint)
    result['reference_migration'] = {
        'method': 'new_raw = old_raw + old_homing_offset - new_homing_offset',
        'source_fingerprint': deepcopy(old), 'reference_shift_ticks': shifts,
        'unchanged_assembly_confirmed': True,
        'new_physical_measurement': False,
    }
    return result


def effective_scale(joint, original_scale, correction):
    if (correction and correction.get('method') == NATIVE_HEIGHT_METHOD
            and joint in PITCH_JOINTS):
        return math.copysign(1/TICKS_PER_RADIAN, original_scale)
    return original_scale


def validate_correction(saved, angles_hash, fingerprint):
    offsets = saved.get('offsets_rad', {})
    validation = saved.get('validation', {})
    native = saved.get('method') == NATIVE_HEIGHT_METHOD
    if (saved.get('version') != 1 or saved.get('method') not in (
            'fixed_jaw_height_zero_offsets', NATIVE_HEIGHT_METHOD)
            or (native and (saved.get('native_resolution') != 4096
                            or saved.get('training_count', 0) < 5))
            or saved.get('joint_calibration_sha256') != angles_hash
            or saved.get('model_sha256') != fingerprint.get('mjcf_sha256')
            or set(offsets) != set(JOINTS) or any(type(v) not in (int, float)
                or not math.isfinite(v) or abs(v) >= math.radians(4.75) for v in offsets.values())
            or any(offsets[j] != 0 for j in ('shoulder_pan', 'wrist_roll', 'gripper'))
            or validation.get('passed') is not True
            or any(not math.isfinite(validation.get(key, math.inf)) or validation[key] > limit
                   for key, limit in (('training_rmse_m', .0015), ('held_out_rmse_m', .002),
                                      ('held_out_max_m', .003), ('jacobian_condition', 100)))
            or validation.get('min_singular_value_m_rad', 0) < .003
            or saved.get('sample_count', 0) < (7 if native else 10)
            or len(saved.get('validation_indices', [])) < 2):
        raise ValueError('Kinematic correction is invalid, stale, or failed validation')
    return offsets


class AngleCalibration:
    def __init__(self, path: Path, fingerprint: dict, *, use_kinematic=True,
                 leader_trigger=False):
        self.path = path
        self.fingerprint = fingerprint
        self.points = {joint: {} for joint in JOINTS}
        self.correction = None
        self.leader_trigger = leader_trigger
        if path.exists():
            saved = json.loads(path.read_text())
            if saved.get("version") != 1 or saved.get("fingerprint") != fingerprint:
                previous = saved.get('fingerprint') or {}
                changed = []
                for field in sorted(set(previous) | set(fingerprint)):
                    if previous.get(field) != fingerprint.get(field):
                        before, after = previous.get(field), fingerprint.get(field)
                        joints = (sorted(j for j in set(before) | set(after)
                                         if before.get(j) != after.get(j))
                                  if isinstance(before, dict) and isinstance(after, dict) else [])
                        changed.append(field + (' ['+', '.join(joints)+']' if joints else ''))
                detail = '; '.join(changed) or 'unsupported calibration version'
                raise ValueError(f'Calibration identity/EEPROM changed for {path.name}: {detail}. '
                                 'Restore the saved device configuration or recalibrate; '
                                 'do not reuse the old angle mapping with changed offsets.')
            self.points.update(saved["points"])
            self.leader_trigger = saved.get('leader_trigger', False)
            correction_path = path.with_name(path.stem+'_kinematic.json')
            if use_kinematic and correction_path.exists():
                correction = json.loads(correction_path.read_text())
                validate_correction(correction, file_digest(path),
                                    fingerprint)
                base_name = correction.get('base_calibration_file', '')
                if not base_name or Path(base_name).name != base_name:
                    raise ValueError('Invalid base file in kinematic correction')
                base_path = path.parent/base_name
                if (not base_path.exists() or file_digest(base_path)
                        != correction.get('base_calibration_sha256')):
                    raise ValueError('Kinematic correction base calibration changed')
                self.correction = correction

    def reference_angles(self, joint):
        return (0, 15, 30) if self.leader_trigger and joint == 'gripper' else (0, 30, 60)

    def fit(self, joint):
        points = self.points[joint]
        references = self.reference_angles(joint)
        if any(str(degrees) not in points for degrees in references):
            return None
        zero, middle, check = [points[str(degrees)]["raw"] for degrees in references]
        span = middle - zero
        expected = math.radians(references[1]) * TICKS_PER_RADIAN
        if not 0.9 * expected <= abs(span) <= 1.1 * expected:
            raise ValueError(
                f"{joint}: 0/{references[1]} span outside native-encoder guard\n"
                f"raw {zero} -> {middle}: {span:+.1f} ticks; "
                f"expected magnitude {expected:.1f} +/-10% "
                f"({0.9 * expected:.1f}..{1.1 * expected:.1f})\n"
                f"At 4096 ticks/turn this implies {abs(span) * 360 / 4096:.2f} degrees; "
                f"{references[2]}-degree check raw={check}. "
                "Verify reference alignment and encoder scaling."
            )
        scale = math.radians(references[1]) / span
        error = abs((check - zero) * scale - math.radians(references[2]))
        if error > math.radians(2):
            raise ValueError(
                f"{joint}: independent {references[2]} degree check error {math.degrees(error):.1f}"
            )
        offset = self.correction['offsets_rad'][joint] if self.correction else 0.
        scale = effective_scale(joint, scale, self.correction)
        return {"zero_raw": zero-offset/scale, "radians_per_tick": scale,
                "check_error_degrees": math.degrees(error)}

    def capture(self, joint, degrees, states):
        if self.correction:
            raise ValueError('Archive the kinematic correction before redoing manual angles')
        if joint not in JOINTS or degrees not in self.reference_angles(joint):
            raise ValueError("capture only the selected joint at its reference angles")
        if len(states) < 9:
            raise ValueError("wait for nine live feedback samples")
        recent = states[-9:]
        if recent[-1]["seq"] - recent[0]["seq"] != 8:
            raise ValueError("feedback samples are not consecutive")
        telemetry = recent[-1]["telemetry"]
        if any(telemetry["Torque_Enable"].values()):
            raise ValueError("manual calibration requires all six torques already off")
        if any(telemetry["Status"].values()):
            raise ValueError("motor reports a fault")
        raw_values = [state["raw"][joint] for state in recent]
        if max(raw_values) - min(raw_values) > 4:
            raise ValueError("joint is moving: hold it still for half a second")
        key = str(degrees)
        previous = self.points[joint].get(key)
        self.points[joint][key] = {"raw": statistics.median(raw_values),
                                  "sample_count": len(raw_values),
                                  "time_unix_ns": recent[-1]["time_unix_ns"]}
        try:
            fit = self.fit(joint)
        except ValueError:
            if previous is None:
                del self.points[joint][key]
            else:
                self.points[joint][key] = previous
            raise
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"version": 1, "model": "so101",
                                         "fingerprint": self.fingerprint,
                                         "leader_trigger": self.leader_trigger,
                                         "points": self.points}, indent=2) + "\n")
        temporary.replace(self.path)
        return fit

    def count(self):
        return sum(self.fit(joint) is not None for joint in JOINTS)

    def angles(self, raw, estimated):
        result = list(estimated)
        for index, joint in enumerate(JOINTS):
            fit = self.fit(joint)
            if fit:
                result[index] = (raw[joint] - fit["zero_raw"]) * fit["radians_per_tick"]
        return result

    def check_gripper_test(self, plan):
        fit = self.fit("gripper")
        if fit is None:
            raise ValueError("gripper angle calibration is incomplete")
        start = (plan["start_raw"] - fit["zero_raw"]) * fit["radians_per_tick"]
        target = (plan["target_raw"] - fit["zero_raw"]) * fit["radians_per_tick"]
        if not math.radians(3) <= min(start, target) <= max(start, target) <= math.radians(95):
            raise ValueError("manually open gripper away from its endpoints before testing")
