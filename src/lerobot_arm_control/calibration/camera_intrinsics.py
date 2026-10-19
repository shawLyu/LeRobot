"""Fit a local, unactivated camera candidate from an independently moved ChArUco board."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def fit_capture(directory, square_mm):
    directory = Path(directory)
    if not np.isfinite(square_mm) or not 20 <= square_mm <= 40:
        raise ValueError('Measure the printed square side in millimeters')
    metadata = json.loads((directory/'calibration.json').read_text())
    if not (metadata.get('camera_controls') or {}).get('verified'):
        raise ValueError('Capture requires verified, fixed camera focus')
    board = cv2.aruco.CharucoBoard((5, 7), square_mm/1000, square_mm/1000*22/30,
                                  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
    detector = cv2.aruco.CharucoDetector(board)
    samples = []
    previous = None
    size = None
    for line in (directory/'states.jsonl').read_text().splitlines():
        frame = json.loads(line)
        image = cv2.imread(str(directory/frame['native_frame']))
        if image is None:
            raise ValueError('Missing native calibration image')
        dimensions = image.shape[1::-1]
        if size is not None and dimensions != size:
            raise ValueError('Capture resolution changed during calibration')
        size = dimensions
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is None or len(ids) < 16:
            continue
        current = {int(i): p for i, p in zip(ids[:, 0], corners[:, 0], strict=True)}
        if previous:
            common = set(current) & set(previous)
            if len(common) >= 12 and np.median([
                    np.linalg.norm(current[i]-previous[i]) for i in common]) < 25:
                continue
        previous = current
        samples.append({'sequence': frame['sequence'], 'objects':
                        board.getChessboardCorners()[ids[:, 0]], 'pixels': corners})
    if len(samples) < 20:
        raise ValueError(f'Need twenty distinct board views; found {len(samples)}')
    training = [s for i, s in enumerate(samples) if i % 5 != 4]
    validation = [s for i, s in enumerate(samples) if i % 5 == 4]
    rms, matrix, distortion, rotations, _ = cv2.calibrateCamera(
        [s['objects'] for s in training], [s['pixels'] for s in training], size, None, None)
    held_errors = []
    for sample in validation:
        ok, rotation, translation = cv2.solvePnP(sample['objects'], sample['pixels'],
                                                matrix, distortion)
        if not ok:
            raise ValueError('Held-out camera pose could not be estimated')
        projection = cv2.projectPoints(sample['objects'], rotation, translation,
                                        matrix, distortion)[0]
        held_errors.append(float(np.sqrt(np.mean(np.sum(
            (projection-sample['pixels'])**2, axis=2)))))
    normals = np.array([cv2.Rodrigues(r)[0][:, 2] for r in rotations])
    spread = np.degrees(np.arccos(np.clip(normals@normals.T, -1, 1))).max()
    centroids = np.array([s['pixels'].reshape(-1, 2).mean(0) for s in samples])
    coverage = np.ptp(centroids, axis=0)/size
    reasons = []
    if rms > .35 or max(held_errors) > .35:
        reasons.append('Image reprojection exceeds 0.35 native pixels')
    if spread < 25 or min(coverage) < .25:
        reasons.append('Board views need more tilt and image coverage')
    if (not .2*size[0] < matrix[0, 0] < 3*size[0]
            or not .2*size[1] < matrix[1, 1] < 4*size[1]
            or not 0 < matrix[0, 2] < size[0] or not 0 < matrix[1, 2] < size[1]):
        reasons.append('Fitted intrinsics are outside the diagnostic envelope')
    return {'version': 1, 'method': 'native_charuco_5x7_30mm', 'activated': False,
            'camera_device': metadata['device'], 'camera_controls': metadata['camera_controls'],
            'image_size': list(size), 'camera_matrix': matrix.tolist(),
            'dist_coeffs': distortion.ravel().tolist(), 'measured_square_mm': square_mm,
            'marker_to_square_ratio': 22/30, 'view_sequences': [s['sequence'] for s in samples],
            'held_out_sequences': [s['sequence'] for s in validation],
            'validation': {'passed': not reasons, 'reasons': reasons,
                           'training_reprojection_rms_px': float(rms),
                           'held_out_reprojection_rms_px': held_errors,
                           'normal_spread_deg': float(spread),
                           'centroid_coverage_fraction': coverage.tolist()},
            'accuracy_scope': 'Independent image reprojection; physical dimensions and '
            'Cartesian accuracy require separate verification'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--square-mm', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--live', action='store_true', help='Open loopback live acquisition UI')
    parser.add_argument('--device', help='Linux camera device for live acquisition')
    parser.add_argument('--port', type=int, default=8879)
    parser.add_argument('--fixed-focus', action='store_true',
                        help='Confirm lens focus is fixed for a new live capture')
    args = parser.parse_args()
    if args.live:
        if not args.device:
            parser.error('--live requires --device')
        from .camera_live import serve
        return serve(args)
    result = fit_capture(args.capture, args.square_mm)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'candidate': str(args.output), 'validation': result['validation']}))


if __name__ == '__main__':
    main()
