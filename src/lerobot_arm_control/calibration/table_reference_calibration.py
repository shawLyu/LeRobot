"""Register two stationary table markers from a camera-only capture, locally."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from lerobot_arm_control.core.files import atomic_json, file_digest
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.perception.tabletop_perception import (
    Perception,
    pixels_on_plane,
    read_config,
)

ROOT = workspace_root()


def fit_capture(directory, config):
    directory = Path(directory)
    metadata = json.loads((directory/'calibration.json').read_text())
    if metadata['config'] != config:
        raise ValueError('Capture differs from the active vision configuration')
    if metadata.get('robot') is not None:
        raise ValueError('Table registration requires a camera-only capture')
    if not (metadata.get('camera_controls') or {}).get('verified'):
        raise ValueError('Table registration requires verified fixed focus')
    if len(config.get('additional_markers', [])) != 1:
        raise ValueError('Configure one primary marker and one auxiliary marker')
    auxiliary = config['additional_markers'][0]
    primary_key = (config['dictionary'], config['marker_id'])
    auxiliary_key = (auxiliary['dictionary'], auxiliary['id'])
    if primary_key == auxiliary_key:
        raise ValueError('Primary and auxiliary markers must have different identities')
    perception = Perception(config)
    source = perception.tag_points[:, :2]
    samples = []
    rejected = []
    previous_ns = 0
    for line in (directory/'states.jsonl').read_text().splitlines():
        frame = json.loads(line)
        captured_ns = frame['captured_unix_ns']
        if captured_ns <= previous_ns:
            raise ValueError('Calibration needs distinct, ordered camera timestamps')
        previous_ns = captured_ns
        markers = {}
        for marker in frame['detected_markers']:
            key = (marker['dictionary'], marker['id'])
            if key in markers:
                raise ValueError('Duplicate table marker identity in one frame')
            markers[key] = marker['corners']
        if primary_key not in markers or auxiliary_key not in markers:
            continue
        primary = np.asarray(markers[primary_key], dtype=float)
        extra = np.asarray(markers[auxiliary_key], dtype=float)
        if (primary.shape != (4, 2) or extra.shape != (4, 2)
                or not np.isfinite([primary, extra]).all()):
            raise ValueError('Invalid observed table corners')
        solution = perception.estimate_pose(perception.tag_points, primary, square=True)
        if solution is None:
            rejected.append(frame['sequence'])
            continue
        measured = pixels_on_plane(extra, perception.matrix, perception.distortion,
                                   solution[1], solution[2])[:, :2]
        center = measured.mean(axis=0)
        left, _, right = np.linalg.svd(source.T@(measured-center))
        rotation = left@right
        residual = float(np.sqrt(np.mean(np.sum(
            (source@rotation+center-measured)**2, axis=1))))
        if np.linalg.det(rotation) <= 0 or residual > .02*perception.scale:
            rejected.append(frame['sequence'])
            continue
        samples.append({'sequence': frame['sequence'], 'captured_unix_ns': captured_ns,
                        'primary': primary, 'auxiliary': extra, 'center': center,
                        'yaw': float(np.arctan2(rotation[0, 1], rotation[0, 0])),
                        'shape_residual_m': residual})
    training = [s for index, s in enumerate(samples) if index % 5 != 4]
    validation = [s for index, s in enumerate(samples) if index % 5 == 4]
    if len(training) < 20 or len(validation) < 5:
        raise ValueError('Need at least 20 training frames and five held-out frames with both tags')
    # A fixed camera has one extrinsic pose. Estimating its tilt separately from
    # four noisy corners in every frame amplifies errors at the distant auxiliary.
    primary = np.median([sample['primary'] for sample in training], axis=0)
    fixed_camera = perception.estimate_pose(perception.tag_points, primary, square=True)
    if fixed_camera is None:
        raise ValueError('Stationary camera reference could not be estimated')
    mapping = np.asarray(config['low_to_capture_affine'], dtype=float)
    primary_motion = []
    for sample in samples:
        delta = (sample['primary']-primary)@mapping[:, :2].T
        primary_motion.append(float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))))
        measured = pixels_on_plane(sample['auxiliary'], perception.matrix, perception.distortion,
                                   fixed_camera[1], fixed_camera[2])[:, :2]
        center = measured.mean(axis=0)
        left, _, right = np.linalg.svd(source.T@(measured-center))
        rotation = left@right
        sample.update(center=center, yaw=float(np.arctan2(rotation[0, 1], rotation[0, 0])))
    if max(primary_motion) > 2.:
        raise ValueError('Camera moved during stationary table calibration')
    centers = np.asarray([s['center'] for s in training])
    center = np.median(centers, axis=0)
    yaws = np.unwrap([s['yaw'] for s in training])
    yaw = float(np.median(yaws))
    spread = float(np.max(np.linalg.norm(centers-center, axis=1)))
    yaw_spread = float(np.max(np.abs(yaws-yaw)))
    if spread > .002 or yaw_spread > .025:
        raise ValueError('Stationary table-marker layout is not stable within 2 mm / 0.025 rad')
    rotation = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    layout = np.column_stack((source@rotation+center, np.zeros(4)))
    objects = np.vstack((perception.tag_points, layout))
    errors = []
    for sample in validation:
        pixels = np.vstack((sample['primary'], sample['auxiliary']))
        solution = perception.estimate_pose(objects, pixels)
        if solution is None:
            raise ValueError('Held-out two-marker pose could not be estimated')
        projected = cv2.projectPoints(objects, solution[3], solution[2],
                                      perception.matrix, perception.distortion)[0].reshape(-1, 2)
        native_error = (projected-pixels)@mapping[:, :2].T
        errors.append(float(np.sqrt(np.mean(np.sum(native_error**2, axis=1)))))
    if max(errors) > 2.:
        raise ValueError('Held-out table reprojection exceeds two native pixels')
    key = auxiliary['dictionary']+':'+str(auxiliary['id'])
    return {'version': 1, 'reference': perception.static_reference(),
            'frame': {'origin': f'{config["dictionary"]} ID {config["marker_id"]} center',
                      'axes': '+X decoded right, +Y decoded up, +Z above table', 'units': 'm'},
            'secondary_layouts': {key: layout.tolist()},
            'auxiliary': {**auxiliary, 'black_border_side_m': perception.scale,
                          'center_m': [*center.tolist(), 0.], 'yaw_rad': yaw,
                          'center_distance_m': float(np.linalg.norm(center))},
            'quality': {'method': 'stationary_camera_median_primary_corners',
                        'max_primary_corner_motion_native_px': max(primary_motion),
                        'training_frames': len(training), 'held_out_frames': len(validation),
                        'position_std_xy_m': centers.std(axis=0).tolist(),
                        'max_position_deviation_m': spread, 'max_yaw_deviation_rad': yaw_spread,
                        'shape_residual_median_m': float(np.median([
                            s['shape_residual_m'] for s in training])),
                        'held_out_native_rms_px': errors, 'passed': True},
            'provenance': {'capture': str(directory), 'camera_device': metadata['device'],
                           'camera_controls': metadata['camera_controls'],
                           'camera_calibration_sha256': config['camera_calibration']['sha256'],
                           'tabletop_config_sha256': hashlib.sha256(json.dumps(
                               config, sort_keys=True).encode()).hexdigest(),
                           'training_sequences': [s['sequence'] for s in training],
                           'held_out_sequences': [s['sequence'] for s in validation],
                           'rejected_sequences': rejected,
                           'capture_metadata_sha256': file_digest(directory / 'calibration.json'),
                           'observations_sha256': file_digest(directory / 'states.jsonl')},
            'observed_unix_ns': samples[-1]['captured_unix_ns'],
            'physical_precision_validated': False,
            'accuracy_scope': 'Stationary two-tag image registration; sampling stability '
                              'and reprojection do not certify physical 1 mm accuracy'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = fit_capture(args.capture, read_config(ROOT/'configs/tabletop.json'))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(args.output, result)
    print(json.dumps({'output': str(args.output), 'auxiliary': result['auxiliary'],
                      'quality': result['quality']}))


if __name__ == '__main__':
    main()
