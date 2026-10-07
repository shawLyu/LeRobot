import copy
import json
import math
import shutil

import numpy as np
import pytest

from arm_control.angle_calibration import NATIVE_HEIGHT_METHOD, AngleCalibration
from arm_control.height_calibration import (
    ANGLES,
    BASE,
    MODEL,
    PITCH_JOINTS,
    JawHeight,
    apply_candidate,
    digest,
    fit_samples,
    read_sample,
)
from arm_control.manual_control import ManualControl
from arm_control.realtime import JOINTS


@pytest.fixture
def measured(tmp_path):
    angles, base = tmp_path/ANGLES.name, tmp_path/BASE.name
    shutil.copyfile(ANGLES, angles)
    shutil.copyfile(BASE, base)
    calibration = AngleCalibration(angles, json.loads(angles.read_text())['fingerprint'])
    base_record = json.loads(base.read_text())
    height = JawHeight(base_record['pose'])
    offsets = np.array([-.018, .024, -.012])
    rng = np.random.default_rng(41)
    samples = []
    while len(samples) < 15:
        q = [rng.uniform(-.4, .4), rng.uniform(.1, 1.2), rng.uniform(-1.4, -.1),
             rng.uniform(.4, 1.6), rng.uniform(-1.6, 1.6), .1]
        raw = {joint: round(q[i]/calibration.fit(joint)['radians_per_tick']
                            + calibration.fit(joint)['zero_raw']) for i, joint in enumerate(JOINTS)}
        q = calibration.angles(raw, [0.]*6)
        h = height(q, offsets)
        if not .01 < h < .14:
            continue
        sample = {'measured_height_m': h, 'qpos': q, 'stable': True,
                  'raw_span_ticks': dict.fromkeys(JOINTS, 0),
                  'samples': [{'raw': raw, 'status': dict.fromkeys(JOINTS, 0)} for _ in range(12)],
                  'writes_performed': False, 'joint_calibration_sha256': digest(angles),
                  'base_calibration': base_record}
        samples.append(sample)
    return samples, offsets, angles, base


def fit(measured):
    samples, _, angles, base = measured
    return fit_samples(samples, angles_path=angles, base_path=base)


def test_diverse_poses_recover_pitch_offsets_and_predict_held_out_heights(measured):
    candidate = fit(measured)
    assert candidate['validation']['passed'], candidate['validation']
    assert candidate['validation_indices'] == [4, 9, 14]
    assert [candidate['offsets_rad'][j] for j in PITCH_JOINTS] == pytest.approx(
        measured[1], abs=1e-5)
    assert candidate['validation']['held_out_max_m'] < 1e-6


def test_measurement_noise_is_validated_on_independent_samples(measured):
    rng = np.random.default_rng(99)
    for sample in measured[0]:
        sample['measured_height_m'] += rng.normal(0, .00015)
    candidate = fit(measured)
    assert candidate['validation']['passed'], candidate['validation']
    assert candidate['validation']['held_out_max_m'] < .001


def test_bad_held_out_measurement_cannot_be_applied(measured):
    measured[0][4]['measured_height_m'] += .012
    candidate = fit(measured)
    assert not candidate['validation']['passed']
    with pytest.raises(ValueError, match='validation'):
        apply_candidate(candidate, angles_path=measured[2], base_path=measured[3])
    assert not measured[2].with_name(measured[2].stem+'_kinematic.json').exists()


def test_repeated_or_insufficient_poses_are_rejected(measured):
    samples, _, angles, base = measured
    with pytest.raises(ValueError, match='at least 10'):
        fit_samples(samples[:2], angles_path=angles, base_path=base)
    with pytest.raises(ValueError, match='Repeated'):
        fit_samples([samples[0]]*10, angles_path=angles, base_path=base)


def test_indistinguishable_pitch_poses_do_not_pass_observability(measured):
    samples, _, angles, base = measured
    calibration = AngleCalibration(angles, json.loads(angles.read_text())['fingerprint'])
    height = JawHeight(samples[0]['base_calibration']['pose'])
    for index, sample in enumerate(samples):
        raw = copy.deepcopy(samples[0]['samples'][0]['raw'])
        raw['shoulder_pan'] += 80*index
        sample['qpos'] = calibration.angles(raw, [0.]*6)
        sample['samples'] = [{'raw': raw, 'status': dict.fromkeys(JOINTS, 0)} for _ in range(12)]
        sample['measured_height_m'] = height(sample['qpos'], measured[1])
    candidate = fit(measured)
    assert not candidate['validation']['passed']
    assert candidate['validation']['jacobian_condition'] > 100


@pytest.mark.parametrize('changed', ['calibration', 'base', 'moving', 'angles', 'fault',
                                   'hidden_motion'])
def test_inconsistent_sample_evidence_is_rejected(measured, changed):
    sample = measured[0][0]
    if changed == 'calibration':
        sample['joint_calibration_sha256'] = 'other'
    elif changed == 'base':
        sample['base_calibration'] = {'pose': {}}
    elif changed == 'moving':
        sample['raw_span_ticks']['elbow_flex'] = 9
    elif changed == 'angles':
        sample['qpos'][1] += .01
    elif changed == 'fault':
        sample['samples'][0]['status']['elbow_flex'] = 1
    else:
        sample['samples'] = copy.deepcopy(sample['samples'])
        sample['samples'][0]['raw']['elbow_flex'] += 8
    with pytest.raises(ValueError):
        fit(measured)


def test_application_preserves_raw_points_and_corrects_control_inverse_mapping(measured):
    candidate = fit(measured)
    _, _, angles, base = measured
    original = angles.read_bytes()
    destination = apply_candidate(candidate, angles_path=angles, base_path=base)
    assert angles.read_bytes() == original
    identity = json.loads(original)['fingerprint']
    uncorrected = AngleCalibration(angles, identity, use_kinematic=False)
    corrected = AngleCalibration(angles, identity)
    raw = {j: uncorrected.fit(j)['zero_raw'] for j in JOINTS}
    assert corrected.angles(raw, [0.]*6) == pytest.approx(
        [candidate['offsets_rad'][j] for j in JOINTS])
    control = ManualControl(None, corrected, [[-3, 3]]*6, identity['raw_limits'])
    for joint in JOINTS:
        assert control.raw(joint, control.angle(joint, 2200)) == 2200
    with pytest.raises(ValueError, match='Archive'):
        corrected.capture('elbow_flex', 0, [])
    apply_candidate(candidate, angles_path=angles, base_path=base)
    assert len(list(destination.parent.glob(destination.stem+'-backup-*.json'))) == 1


@pytest.mark.parametrize('changed', ['manual', 'base', 'model', 'validation', 'envelope'])
def test_stale_or_invalid_active_correction_cannot_silently_load(measured, changed):
    candidate = fit(measured)
    _, _, angles, base = measured
    path = apply_candidate(candidate, angles_path=angles, base_path=base)
    identity = json.loads(angles.read_text())['fingerprint']
    if changed == 'manual':
        angles.write_text(angles.read_text()+' ')
    elif changed == 'base':
        base.write_text(base.read_text()+' ')
    else:
        value = json.loads(path.read_text())
        if changed == 'model':
            value['model_sha256'] = 'other'
        elif changed == 'validation':
            value['validation']['held_out_rmse_m'] = .008
        else:
            value['offsets_rad']['elbow_flex'] = math.radians(10)
        path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        AngleCalibration(angles, identity)


def test_sampling_uses_only_reads_and_rejects_motion_or_fault(measured):
    samples, _, angles, base = measured
    calibration = AngleCalibration(angles, json.loads(angles.read_text())['fingerprint'])

    class Bus:
        calls = 0
        moving = fault = False

        def sync_read(self, key, normalize):
            assert normalize is False
            assert key in ('Present_Position', 'Status')
            if key == 'Status':
                return dict.fromkeys(JOINTS, int(self.fault))
            self.calls += 1
            raw = samples[0]['samples'][0]['raw'].copy()
            if self.moving:
                raw['elbow_flex'] += self.calls
            return raw

    bus = Bus()
    sample = read_sample(bus, calibration, 0., json.loads(base.read_text()), sleep=lambda _: None)
    assert sample['writes_performed'] is False
    assert len(sample['samples']) == 12
    bus.moving = True
    with pytest.raises(ValueError, match='moved'):
        read_sample(bus, calibration, 0., sample['base_calibration'], sleep=lambda _: None)
    bus.fault = True
    with pytest.raises(ValueError, match='fault'):
        read_sample(bus, calibration, 0., sample['base_calibration'], sleep=lambda _: None)


def test_model_is_the_same_as_original_joint_calibration():
    assert json.loads(ANGLES.read_text())['fingerprint']['mjcf_sha256'] == digest(MODEL)


def test_fresh_capture_checks_the_effective_corrected_mapping(measured):
    from arm_control.push_t_capture import calibration_bytes

    candidate = fit(measured)
    _, _, angles, base = measured
    apply_candidate(candidate, angles_path=angles, base_path=base)
    identity = json.loads(angles.read_text())['fingerprint']
    corrected = AngleCalibration(angles, identity)
    hello = {'robot': {'read_only': True, 'fingerprint': identity,
                       'kinematic_correction': candidate,
                       'fits': {j: corrected.fit(j) for j in JOINTS}}}
    assert calibration_bytes(hello, angles) == angles.read_bytes()
    hello['robot']['fits']['elbow_flex']['zero_raw'] += 1
    with pytest.raises(ValueError, match='calibrations differ'):
        calibration_bytes(hello, angles)


def test_offline_simulation_uses_corrected_encoder_limits(measured, tmp_path):
    from arm_control.push_t import build_scene

    candidate = fit(measured)
    _, _, angles, base = measured
    apply_candidate(candidate, angles_path=angles, base_path=base)
    identity = json.loads(angles.read_text())['fingerprint']
    corrected = AngleCalibration(angles, identity)
    source = tmp_path/'snapshot'
    shutil.copytree(ANGLES.parents[1]/'push_t/measured', source)
    shutil.copyfile(angles, source/'joint_calibration.json')
    metadata = json.loads((source/'calibration.json').read_text())
    metadata['robot']['fits'] = {j: corrected.fit(j) for j in JOINTS}
    (source/'calibration.json').write_text(json.dumps(metadata))
    _, _, model_path = build_scene(source, tmp_path/'sim')
    model = __import__('mujoco').MjModel.from_xml_path(str(model_path))
    joint = 'elbow_flex'
    fit_value = corrected.fit(joint)
    high = max((raw-fit_value['zero_raw'])*fit_value['radians_per_tick']
               for raw in identity['raw_limits'][joint])
    assert model.joint(joint).range[1] == pytest.approx(high)


def test_native_resolution_corrects_real_contact_data_with_independent_validation(tmp_path):
    from pathlib import Path

    path = Path(__file__).parent/'fixtures/height-first-round-samples.json'
    samples = json.loads(path.read_text())['samples']
    # Historical measurements retain their original base, independent of live recalibration.
    base = tmp_path/BASE.name
    base.write_text(json.dumps(samples[0]['base_calibration'], indent=2)+'\n')
    candidate = fit_samples(samples, base_path=base, native_pitch_scale=True)
    assert candidate['method'] == NATIVE_HEIGHT_METHOD
    assert candidate['source_indices'] == [0, 1, 3, 5, 7, 8, 9]
    assert candidate['sample_count'] == 7
    assert candidate['training_count'] == 5
    assert candidate['validation_indices'] == [2, 5]
    assert candidate['validation']['passed'], candidate['validation']
    assert candidate['validation']['training_rmse_m'] < .001
    assert candidate['validation']['held_out_max_m'] < .0015
    with pytest.raises(ValueError, match='7 known'):
        fit_samples(samples[:6], base_path=base, native_pitch_scale=True)


def test_native_scale_changes_only_pitch_axes_and_keeps_the_motor_inverse_consistent(measured):
    from arm_control.push_t_capture import calibration_bytes

    candidate = fit(measured)
    candidate.update(method=NATIVE_HEIGHT_METHOD, native_resolution=4096)
    _, _, angles, base = measured
    original = AngleCalibration(angles, json.loads(angles.read_text())['fingerprint'])
    apply_candidate(candidate, angles_path=angles, base_path=base)
    corrected = AngleCalibration(angles, original.fingerprint)
    control = ManualControl(None, corrected, [[-3, 3]]*6, original.fingerprint['raw_limits'])
    for joint in JOINTS:
        fit_value = corrected.fit(joint)
        if joint in PITCH_JOINTS:
            assert abs(fit_value['radians_per_tick']) == pytest.approx(2*math.pi/4096)
        else:
            assert fit_value == original.fit(joint)
        assert control.raw(joint, control.angle(joint, 2400)) == 2400
    hello = {'robot': {'read_only': True, 'fingerprint': original.fingerprint,
                       'kinematic_correction': candidate,
                       'fits': {j: corrected.fit(j) for j in JOINTS}}}
    assert calibration_bytes(hello, angles) == angles.read_bytes()


@pytest.mark.parametrize('bad', ['resolution', 'count', 'training'])
def test_native_scale_cannot_skip_resolution_or_independent_evidence_checks(measured, bad):
    candidate = fit(measured)
    candidate.update(method=NATIVE_HEIGHT_METHOD, native_resolution=4096)
    if bad == 'resolution':
        candidate['native_resolution'] = 8192
    elif bad == 'count':
        candidate['sample_count'] = 6
    else:
        candidate['training_count'] = 4
    with pytest.raises(ValueError, match='validation'):
        apply_candidate(candidate, angles_path=measured[2], base_path=measured[3])
