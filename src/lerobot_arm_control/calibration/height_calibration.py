"""Local-only MuJoCo height fitting; remote sampling is handled by height_sampling."""

import argparse
import copy
import json
import math
import time
from pathlib import Path

import numpy as np

from lerobot_arm_control.calibration.angle_calibration import (
    NATIVE_HEIGHT_METHOD,
    PITCH_JOINTS,
    AngleCalibration,
    effective_scale,
)
from lerobot_arm_control.calibration.height_records import (
    ANGLES,
    BASE,
    CANDIDATE,
    DATA,
    check_samples,
    save_json,
)
from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.transport.realtime import JOINTS

PITCH_INDICES = [JOINTS.index(joint) for joint in PITCH_JOINTS]
BOUND = math.radians(5)


class JawHeight:
    """Lowest vertex of the fixed jaw visual mesh in the saved table frame."""
    def __init__(self, base_pose, model_path=MODEL):
        import mujoco

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.qadr = [self.model.joint(joint).qposadr[0] for joint in JOINTS]
        self.base_z = base_pose['position'][2]
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, np.asarray(base_pose['quaternion'], float))
        self.vertical = rotation.reshape(3, 3)[2]
        mesh = self.model.mesh('wrist_roll_follower_so101_v1').id
        self.geom = next(i for i in range(self.model.ngeom)
                         if self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                         and self.model.geom_dataid[i] == mesh)
        start, count = self.model.mesh_vertadr[mesh], self.model.mesh_vertnum[mesh]
        self.vertices = self.model.mesh_vert[start:start+count].astype(float)

    def __call__(self, qpos, offsets):
        import mujoco

        qpos = np.asarray(qpos, float).copy()
        qpos[PITCH_INDICES] += offsets
        self.data.qpos[self.qadr] = qpos
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.geom_xmat[self.geom].reshape(3, 3)
        direction = rotation.T@self.vertical
        return float((self.vertices@direction).min()
                     + self.data.geom_xpos[self.geom]@self.vertical + self.base_z)

def residuals(height, samples, offsets):
    return np.array([height(sample['qpos'], offsets)-sample['measured_height_m']
                     for sample in samples])


def jacobian(height, samples, offsets):
    delta = 1e-5
    eye = np.eye(3)*delta
    return np.column_stack([(residuals(height, samples, offsets+step)
                             - residuals(height, samples, offsets-step))/(2*delta)
                            for step in eye])


def robust_cost(errors):
    absolute = np.abs(errors)
    return float(np.where(absolute <= .002, errors**2/2,
                          .002*(absolute-.001)).sum())


def solve_offsets(height, samples):
    """Three-parameter robust LM; no scipy and no dynamics/hardware execution."""
    offsets = np.zeros(3)
    damping = 1e-4
    for _ in range(80):
        error = residuals(height, samples, offsets)
        j = jacobian(height, samples, offsets)
        weights = np.minimum(1., .002/np.maximum(np.abs(error), 1e-12))
        normal = j.T@(weights[:, None]*j)
        step = np.linalg.solve(normal+np.eye(3)*damping, -j.T@(weights*error))
        proposed = np.clip(offsets+step, -BOUND, BOUND)
        if np.max(np.abs(proposed-offsets)) < 1e-8:
            break
        if robust_cost(residuals(height, samples, proposed)) < robust_cost(error):
            offsets = proposed
            damping = max(1e-10, damping*.3)
        else:
            damping = min(1e6, damping*10)
    return offsets


def fit_samples(samples, *, angles_path=ANGLES, base_path=BASE, model_path=MODEL,
                native_pitch_scale=False):
    base = check_samples(samples, angles_path, base_path)
    source_indices = list(range(len(samples)))
    if native_pitch_scale:
        source_indices = [i for i, s in enumerate(samples) if s['measured_height_m'] == 0]
        samples = [copy.deepcopy(samples[i]) for i in source_indices]
        if len(samples) < 7:
            raise ValueError('Native pitch scale needs 7 known table-contact poses')
        saved = json.loads(Path(angles_path).read_text())
        calibration = AngleCalibration(Path(angles_path), saved['fingerprint'],
                                       use_kinematic=False)
        for sample in samples:
            for index, joint in zip(PITCH_INDICES, PITCH_JOINTS):
                original = calibration.fit(joint)['radians_per_tick']
                sample['qpos'][index] *= effective_scale(joint, original, {
                    'method': NATIVE_HEIGHT_METHOD})/original
    elif len(samples) < 10:
        raise ValueError('Need at least 10 poses: 8 fitting and 2 held-out verification poses')
    qpos = np.asarray([s['qpos'] for s in samples])
    for index, q in enumerate(qpos):
        if any(np.linalg.norm(q[:5]-previous[:5]) < .08 for previous in qpos[:index]):
            raise ValueError('Repeated poses: change extension and wrist orientation')
    validation_indices = (list(range(2, len(samples), 3)) if native_pitch_scale
                          else list(range(4, len(samples), 5)))
    training = [s for i, s in enumerate(samples) if i not in validation_indices]
    validation = [samples[i] for i in validation_indices]
    height = JawHeight(base['pose'], model_path)
    offsets = solve_offsets(height, training)
    singular = np.linalg.svd(jacobian(height, training, offsets), compute_uv=False)
    condition = float(singular[0]/max(singular[-1], 1e-15))
    train_error = residuals(height, training, offsets)
    verify_error = residuals(height, validation, offsets)
    baseline = residuals(height, validation, np.zeros(3))
    train_rmse = float(np.sqrt(np.mean(train_error**2)))
    verify_rmse = float(np.sqrt(np.mean(verify_error**2)))
    verify_max = float(np.max(np.abs(verify_error)))
    baseline_rmse = float(np.sqrt(np.mean(baseline**2)))
    reasons = []
    if singular[-1] < .003 or condition > 100:
        reasons.append('Insufficient independent poses; vary wrist pitch and extension more')
    if np.max(np.abs(offsets)) >= .95*BOUND:
        reasons.append('Correction reached 5-degree bound; check angle scale/mounting')
    if train_rmse > .0015 or verify_rmse > .002 or verify_max > .003:
        reasons.append('Height error exceeds validation limits; inspect scale, flex or geometry')
    if baseline_rmse > .001 and verify_rmse > baseline_rmse*.75:
        reasons.append('Held-out error did not improve by at least 25 percent')
    corrected = dict.fromkeys(JOINTS, 0.)
    corrected.update(zip(PITCH_JOINTS, offsets.tolist()))
    result = {'version': 1, 'method': (NATIVE_HEIGHT_METHOD if native_pitch_scale
                                     else 'fixed_jaw_height_zero_offsets'),
            'joint_calibration_sha256': file_digest(angles_path),
            'model_sha256': file_digest(model_path), 'base_calibration_file': Path(base_path).name,
            'base_calibration_sha256': file_digest(base_path), 'offsets_rad': corrected,
            'created_unix_ns': time.time_ns(), 'sample_count': len(samples),
            'training_count': len(training), 'source_indices': source_indices,
            'validation_indices': validation_indices,
            'validation': {'passed': not reasons, 'reasons': reasons,
                           'training_rmse_m': train_rmse, 'held_out_rmse_m': verify_rmse,
                           'held_out_max_m': verify_max, 'baseline_held_out_rmse_m': baseline_rmse,
                           'jacobian_condition': condition, 'min_singular_value_m_rad':
                           float(singular[-1])},
            'residuals_m': residuals(height, samples, offsets).tolist()}
    if native_pitch_scale:
        result.update(native_resolution=4096,
                      scale_source='installed LeRobot STS3215 model_resolution_table',
                      measurement_selection='Known 0 mm table-contact constraints only; '
                      'suspended ruler heights excluded by measurement method')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('fit',))
    parser.add_argument('--data', type=Path, default=DATA)
    parser.add_argument('--native-pitch-scale', action='store_true')
    args = parser.parse_args()
    samples = json.loads(args.data.read_text())['samples']
    candidate = fit_samples(samples, native_pitch_scale=args.native_pitch_scale)
    candidate['dataset_sha256'] = file_digest(args.data)
    save_json(CANDIDATE, candidate)
    print(json.dumps({'candidate': str(CANDIDATE), 'validation': candidate['validation']},
                     indent=2), flush=True)


if __name__ == "__main__":
    main()
