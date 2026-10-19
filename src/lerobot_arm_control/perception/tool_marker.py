"""Independent camera measurement of a rigid end-effector marker, without FK."""

import math

import cv2
import numpy as np

from lerobot_arm_control.perception.marker_detection import detect_markers


class ToolMarker:
    def __init__(self, config, matrix, distortion):
        self.config = config
        self.matrix, self.distortion = matrix, distortion
        size = config['marker_size_m']
        if not isinstance(size, (int, float)) or not math.isfinite(size) or not 0 < size < .15:
            raise ValueError('Tool marker requires its measured black-border side in meters')
        half = size/2
        self.points = np.array([[-half, half, 0], [half, half, 0],
                                [half, -half, 0], [-half, -half, 0]], float)
        self.reference = config.get('tool_reference')
        if self.reference:
            if type(self.reference.get('transform_validated', False)) is not bool:
                raise ValueError('Tool reference transform_validated must be a boolean')
            self.reference_offset = np.asarray(self.reference['offset_marker_m'], float)
            if (self.reference_offset.shape != (3,)
                    or not np.all(np.isfinite(self.reference_offset))
                    or np.linalg.norm(self.reference_offset) > .3):
                raise ValueError(
                    'Tool reference requires a finite three-dimensional offset in meters')
        parameters = cv2.aruco.DetectorParameters()
        methods = {'subpix': cv2.aruco.CORNER_REFINE_SUBPIX,
                   'contour': cv2.aruco.CORNER_REFINE_CONTOUR}
        method = config.get('corner_refinement', 'subpix')
        if method not in methods:
            raise ValueError('Tool marker corner refinement must be subpix or contour')
        parameters.cornerRefinementMethod = methods[method]
        parameters.cornerRefinementWinSize = 2
        constant = config.get('adaptive_thresh_constant', 7.)
        if (not isinstance(constant, (int, float)) or not math.isfinite(constant)
                or not 0 <= constant <= 20):
            raise ValueError('Tool marker adaptive threshold constant must be in 0..20')
        parameters.adaptiveThreshConstant = constant
        parameters.adaptiveThreshWinSizeMax = 51
        parameters.adaptiveThreshWinSizeStep = 2
        parameters.errorCorrectionRate = 0  # Require the observed code, without corrected bits.
        self.cells = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, config['dictionary'])).markerSize+2
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, config['dictionary'])), parameters)

    def estimate(self, corners, table):
        result = {'status': 'table_reference_lost', 'position_valid': False,
                  'orientation_valid': False, 'dictionary': self.config['dictionary'],
                  'id': self.config['id'], 'marker_size_m': self.config['marker_size_m'],
                  'measurement_source': 'camera marker; no encoder or forward kinematics'}
        if not table.get('marker_visible'):
            return result
        corners = np.asarray(corners, float).reshape(4, 1, 2)
        result['corners'] = corners.reshape(4, 2).tolist()
        solved = cv2.solvePnPGeneric(self.points, corners, self.matrix, self.distortion,
                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        for rvec, tvec in zip(solved[1], solved[2]):
            rotation = cv2.Rodrigues(rvec)[0]
            if not np.all((self.points@rotation.T+tvec[:, 0])[:, 2] > 0):
                continue
            pixels = cv2.projectPoints(self.points, rvec, tvec, self.matrix,
                                      self.distortion)[0]
            error = float(np.sqrt(np.mean(np.sum((pixels-corners)**2, axis=2))))
            candidates.append((error, rotation, tvec[:, 0]))
        candidates.sort(key=lambda value: value[0])
        result['status'] = 'geometry_rejected'
        if not candidates or candidates[0][0] > self.config.get('max_reprojection_error_px', 1):
            return result
        error, rotation, translation = candidates[0]
        position_ambiguous = orientation_ambiguous = False
        if len(candidates) > 1 and candidates[1][0] <= error+.1:
            _, other_rotation, other_translation = candidates[1]
            position_gap = float(np.linalg.norm(translation-other_translation))
            rotation_gap = float(np.arccos(np.clip(
                (np.trace(rotation.T@other_rotation)-1)/2, -1, 1)))
            result.update(alternative_position_gap_m=position_gap,
                          alternative_orientation_gap_rad=rotation_gap)
            position_ambiguous = position_gap > .002
            orientation_ambiguous = rotation_gap > math.radians(5)
        table_rotation = np.asarray(table['camera_rotation'])
        position = np.asarray(table['camera_position'])+table_rotation.T@translation
        result.update(status='ambiguous' if position_ambiguous or orientation_ambiguous
                      else 'observed', position_valid=not position_ambiguous,
                      orientation_valid=not orientation_ambiguous,
                      position=position.tolist(), rotation=(table_rotation.T@rotation).tolist(),
                      reprojection_error_px=error, captured_unix_ns=table['captured_unix_ns'])
        if self.reference:
            pose_valid = not position_ambiguous and not orientation_ambiguous
            validated = self.reference.get('transform_validated', False)
            reference = {'name': self.reference['name'], 'position_valid': pose_valid and validated,
                         'mount_transform_validated': validated, 'precision_validated': False,
                         'status': ('pose_ambiguous' if not pose_valid else
                                    'observed' if validated else 'nominal_mount')}
            if pose_valid:
                point = position+table_rotation.T@rotation@self.reference_offset
                reference['position' if validated else 'nominal_position'] = point.tolist()
            result['tool_reference'] = reference
        return result

    def process(self, frame, table, native_frame=None, low_to_native=None):
        image = native_frame if native_frame is not None else frame
        corners, ids, _ = detect_markers(self.detector, image)
        if ids is None or self.config['id'] not in ids:
            return {'status': 'marker_missing', 'position_valid': False,
                    'orientation_valid': False, 'id': self.config['id']}
        matches = np.flatnonzero(ids.flatten() == self.config['id'])
        if len(matches) != 1:
            return {'status': 'duplicate_marker', 'position_valid': False,
                    'orientation_valid': False, 'id': self.config['id']}
        corner = corners[matches[0]]
        native_corner = corner[0].copy()
        short_edge = float(np.min(np.linalg.norm(
            native_corner-np.roll(native_corner, -1, axis=0), axis=1)))
        if native_frame is not None and low_to_native is not None:
            corner = cv2.transform(corner, cv2.invertAffineTransform(
                np.asarray(low_to_native, float)))
        result = self.estimate(corner, table)
        result.update(source_image_size=list(image.shape[1::-1]),
                      source_corners=native_corner.tolist(),
                      source_short_edge_px=short_edge,
                      source_pixels_per_cell=short_edge/self.cells,
                      error_correction_rate=0,
                      precision_validated=False)
        return result
