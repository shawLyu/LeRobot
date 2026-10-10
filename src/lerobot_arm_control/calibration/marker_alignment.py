"""Local-only independent camera marker fitting; candidates never activate themselves."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.transport.realtime import JOINTS

ROOT = workspace_root()
BOUND = math.radians(5)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class MarkerKinematics:
    """Nominal CAD marker in the measured table frame, with five joint zero offsets."""

    def __init__(self, *, native_scale=True):
        import mujoco

        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.data = mujoco.MjData(self.model)
        self.qadr = [self.model.joint(j).qposadr[0] for j in JOINTS]
        self.body = self.model.body('gripper').id
        angles = ROOT/'data/calibration/follower_mujoco.json'
        saved = json.loads(angles.read_text())
        calibration = AngleCalibration(angles, saved['fingerprint'], use_kinematic=False)
        self.zeros = np.array([calibration.fit(j)['zero_raw'] for j in JOINTS])
        self.scales = np.array([calibration.fit(j)['radians_per_tick'] for j in JOINTS])
        if native_scale:
            self.scales[:5] = np.sign(self.scales[:5])*2*math.pi/4096
        base = json.loads((ROOT/'data/calibration/follower_table_base.json').read_text())['pose']
        self.base_position = np.array(base['position'])
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, np.array(base['quaternion']))
        self.base_rotation = matrix.reshape(3, 3)
        config = json.loads((ROOT/'configs/tabletop.json').read_text())
        self.mount = np.array(config['robot']['tool_marker']['cad_reference'][
            'marker_pose_in_gripper']['position_m'])

    def __call__(self, raw, offsets):
        import mujoco

        q = (np.array([raw[j] for j in JOINTS])-self.zeros)*self.scales
        q[:5] += offsets
        self.data.qpos[self.qadr] = q
        mujoco.mj_forward(self.model, self.data)
        point = (self.data.xpos[self.body]
                 + self.data.xmat[self.body].reshape(3, 3)@self.mount)
        return self.base_position+self.base_rotation@point


def read_observations(directory):
    """Use real capture timestamps inside settled motor windows, without FK filtering."""
    directory = Path(directory)
    proof = json.loads((directory/'result.json').read_text())
    if (proof.get('completed') is not True or proof.get('error') or proof.get('cleanup_error')):
        raise ValueError('Incomplete acquisition cannot validate alignment')
    if proof.get('validation_scope', 'absolute marker alignment') != 'absolute marker alignment':
        raise ValueError('Relative-only acquisition cannot validate absolute alignment')
    recorded = proof.get('input_sha256', {})
    for key, path in {'angles': ROOT/'data/calibration/follower_mujoco.json',
                      'base': ROOT/'data/calibration/follower_table_base.json',
                      'model': MODEL}.items():
        if recorded.get(key) != digest(path):
            raise ValueError('Capture joint, base or model provenance is missing or stale')
    camera_config = json.loads((directory/'camera/calibration.json').read_text())['config']
    if camera_config != json.loads((ROOT/'configs/tabletop.json').read_text()):
        raise ValueError('Capture vision configuration differs from the current calibration')
    camera = [json.loads(line) for line in
              (directory/'camera/states.jsonl').read_text().splitlines()]
    poses = []
    for pose in proof['poses']:
        motors = [s['state'] for s in proof['samples']
                  if s['phase'] == pose['phase'] and s['settled']]
        if not motors:
            continue
        low, high = motors[0]['time_unix_ns'], motors[-1]['time_unix_ns']
        frames = [s for s in camera if low <= s['captured_unix_ns'] <= high
                  and s.get('tool_marker', {}).get('position_valid')]
        if len(frames) < 12:
            raise ValueError(f"Pose {pose['name']} has fewer than twelve real camera frames")
        raw = {j: float(np.median([s['raw'][j] for s in motors])) for j in JOINTS}
        span = {j: int(np.ptp([s['raw'][j] for s in motors])) for j in JOINTS}
        points = np.array([s['tool_marker']['position'] for s in frames])
        poses.append({'name': pose['name'], 'phase': pose['phase'], 'raw': raw,
                      'target_raw': {**pose['target_raw'], 'gripper': raw['gripper']},
                      'measured_position_m': np.median(points, axis=0).tolist(),
                      'camera_samples': len(frames), 'raw_span': span,
                      'raw_stable': max(span.values()) <= 4,
                      'position_std_mm': (points.std(0)*1000).tolist(),
                      'source': str(directory), 'capture_sequences':
                      [s['sequence'] for s in frames]})
    return poses


def errors(model, poses, offsets, *, use_targets=False):
    field = 'target_raw' if use_targets else 'raw'
    return np.array([model(p[field], offsets)-p['measured_position_m'] for p in poses])


def jacobian(model, poses, offsets):
    delta = 1e-5
    return np.column_stack([((errors(model, poses, offsets+step)
                             -errors(model, poses, offsets-step))/(2*delta)).ravel()
                            for step in np.eye(5)*delta])


def solve(model, poses, bound=BOUND):
    offsets = np.zeros(5)
    damping = 1e-4

    def cost(residual):
        norms = np.linalg.norm(residual, axis=1)
        return float(np.where(norms <= .002, norms**2/2, .002*(norms-.001)).sum())

    for _ in range(100):
        residual = errors(model, poses, offsets)
        j = jacobian(model, poses, offsets)
        weights = np.repeat(np.minimum(1., .002/np.maximum(
            np.linalg.norm(residual, axis=1), 1e-12)), 3)
        step = np.linalg.solve(j.T@(weights[:, None]*j)+np.eye(5)*damping,
                               -j.T@(weights*residual.ravel()))
        proposed = np.clip(offsets+step, -bound, bound)
        if np.max(np.abs(proposed-offsets)) < 1e-9:
            break
        if cost(errors(model, poses, proposed)) < cost(residual):
            offsets = proposed
            damping = max(1e-10, damping*.3)
        else:
            damping = min(1e6, damping*10)
    return offsets


def fit(poses, validation_indices, *, native_scale=True, offset_bound_deg=5):
    if len(poses) < 12 or len(validation_indices) < 3:
        raise ValueError('Need twelve poses and three held-out poses')
    if len(set(validation_indices)) != len(validation_indices) or any(
            type(i) is not int or not 0 <= i < len(poses) for i in validation_indices):
        raise ValueError('Invalid held-out indices')
    training = [p for i, p in enumerate(poses) if i not in validation_indices]
    validation = [poses[i] for i in validation_indices]
    model = MarkerKinematics(native_scale=native_scale)
    if offset_bound_deg not in (5, 10):
        raise ValueError('Diagnostic joint-offset bound must be 5 or 10 degrees')
    bound = math.radians(offset_bound_deg)
    offsets = solve(model, training, bound)
    singular = np.linalg.svd(jacobian(model, training, offsets), compute_uv=False)
    train = np.linalg.norm(errors(model, training, offsets), axis=1)
    verify = np.linalg.norm(errors(model, validation, offsets), axis=1)
    commanded = np.linalg.norm(errors(model, validation, offsets, use_targets=True), axis=1)
    condition = float(singular[0]/max(singular[-1], 1e-15))
    reasons = []
    if any(p.get('raw_stable') is False for p in validation):
        reasons.append('A held-out pose moved more than four counts while sampling')
    if singular[-1] < .003 or condition > 100:
        reasons.append('Five joint offsets are not independently observable in these poses')
    if max(abs(offsets)) >= .95*bound:
        reasons.append('A joint correction reached the diagnostic fitting bound')
    if max(verify) > .001 or max(commanded) > .001:
        reasons.append('Held-out camera/model or camera/commanded-pose error exceeds 1 mm')
    return {'version': 1, 'method': 'camera_marker_five_joint_offsets',
            'native_scale': native_scale, 'native_resolution': 4096,
            'offset_bound_deg': offset_bound_deg,
            'offsets_rad': dict(zip(JOINTS, [*offsets.tolist(), 0.], strict=True)),
            'model_sha256': digest(MODEL), 'joint_calibration_sha256':
            digest(ROOT/'data/calibration/follower_mujoco.json'), 'base_calibration_sha256':
            digest(ROOT/'data/calibration/follower_table_base.json'),
            'tabletop_config_sha256': digest(ROOT/'configs/tabletop.json'),
            'poses': poses, 'validation_indices': validation_indices,
            'validation': {'passed': not reasons, 'reasons': reasons,
                           'training_rmse_m': float(np.sqrt(np.mean(train**2))),
                           'held_out_rmse_m': float(np.sqrt(np.mean(verify**2))),
                           'held_out_max_m': float(max(verify)),
                           'commanded_held_out_max_m': float(max(commanded)),
                           'jacobian_condition': condition,
                           'min_singular_value_m_rad': float(singular[-1])},
            'geometry_error_m': np.linalg.norm(errors(model, poses, offsets), axis=1).tolist(),
            'commanded_error_m': np.linalg.norm(errors(model, poses, offsets,
                                                       use_targets=True), axis=1).tolist(),
            'activated': False,
            'accuracy_scope': 'ID 8 center at the tested stationary poses in the camera '
            'table frame; fingertip transform and camera absolute metrology unvalidated'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--validation-indices', type=int, nargs='+', required=True)
    parser.add_argument('--original-scale', action='store_true')
    parser.add_argument('--offset-bound-deg', type=int, choices=(5, 10), default=5)
    args = parser.parse_args()
    observations = [pose for path in args.capture for pose in read_observations(path)]
    result = fit(observations, args.validation_indices, native_scale=not args.original_scale,
                 offset_bound_deg=args.offset_bound_deg)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'candidate': str(args.output), 'offsets_rad': result['offsets_rad'],
                      'validation': result['validation']}, indent=2))


if __name__ == '__main__':
    main()
