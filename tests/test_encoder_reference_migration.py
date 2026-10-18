import copy
import json

import pytest

from lerobot_arm_control.calibration.angle_calibration import (
    AngleCalibration,
    migrate_sts3215_reference,
)
from lerobot_arm_control.core.paths import resource_path


def source():
    return json.loads(resource_path('data/calibration/follower_mujoco.json').read_text())


def test_explicit_reference_shift_preserves_angles_and_independent_check(tmp_path):
    old = source()
    original = copy.deepcopy(old)
    fp = copy.deepcopy(old['fingerprint'])
    fp['homing_offsets']['shoulder_pan'] += 104
    fp['raw_limits']['gripper'] = [452, 1893]
    migrated = migrate_sts3215_reference(old, fp, unchanged_assembly_confirmed=True)
    a, b = tmp_path/'old.json', tmp_path/'new.json'
    a.write_text(json.dumps(old))
    b.write_text(json.dumps(migrated))
    before = AngleCalibration(a, old['fingerprint'], use_kinematic=False)
    after = AngleCalibration(b, fp, use_kinematic=False)
    for joint in before.points:
        shift = old['fingerprint']['homing_offsets'][joint] - fp['homing_offsets'][joint]
        fit_a, fit_b = before.fit(joint), after.fit(joint)
        assert fit_a['radians_per_tick'] == fit_b['radians_per_tick']
        assert fit_a['check_error_degrees'] == fit_b['check_error_degrees']
        assert ((2400-fit_a['zero_raw'])*fit_a['radians_per_tick'] == pytest.approx(
            (2400+shift-fit_b['zero_raw'])*fit_b['radians_per_tick']))
    assert old == original


@pytest.mark.parametrize('change', ['unconfirmed', 'port', 'phase', 'mode', 'wrap'])
def test_migration_does_not_accept_other_reference_changes(change):
    old = source()
    fp = copy.deepcopy(old['fingerprint'])
    confirmed = True
    if change == 'unconfirmed':
        confirmed = False
    elif change == 'port':
        fp['port'] = 'another-device'
    elif change == 'phase':
        fp['phase']['elbow_flex'] ^= 1
    elif change == 'mode':
        fp['mode']['elbow_flex'] = 1
    else:
        fp['homing_offsets']['shoulder_lift'] = -2047
    with pytest.raises(ValueError):
        migrate_sts3215_reference(old, fp, unchanged_assembly_confirmed=confirmed)
