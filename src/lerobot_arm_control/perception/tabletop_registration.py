"""Remote ArUco base registration; unknown mounts never become calibrated poses."""

import copy
import hashlib
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.perception.marker_detection import detect_markers
from lerobot_arm_control.perception.tabletop_perception import pixels_on_plane


def base_registration_configs(config):
    """Each physical arm owns distinct marker identities and a distinct pose file."""
    entries = [config.get('robot', {}), *config.get('additional_robots', [])]
    enabled = [entry for entry in entries if entry.get('enabled')]
    names, files, markers = set(), set(), set()
    for entry in enabled:
        name = entry['arm']
        path = entry.get('fixed_base', {}).get('file')
        selected = entry.get('localization_marker_ids')
        identities = {(marker['dictionary'], marker['id']) for marker in entry['markers']
                      if selected is None or marker['id'] in selected}
        if name in names or (path and path in files) or identities & markers:
            raise ValueError('Arms must have distinct names, marker identities and base files')
        names.add(name)
        if path:
            files.add(path)
        markers.update(identities)
    return enabled


def base_distances(poses):
    """Distances use measured bases in one table frame; missing/held poses are excluded."""
    valid = [(name, pose) for name, pose in poses.items()
             if pose.get('calibrated') and pose.get('status') in ('tracked', 'fixed')]
    distances = []
    for index, (first, a) in enumerate(valid):
        for second, b in valid[index+1:]:
            delta = np.asarray(b['position'])-np.asarray(a['position'])
            distances.append({'arms': [first, second],
                              'relative_position_m': delta.tolist(),
                              'center_distance_m': float(np.linalg.norm(delta)),
                              'planar_distance_m': float(np.linalg.norm(delta[:2])),
                              'both_fixed': a['status'] == b['status'] == 'fixed'})
    return distances


def rotation_quaternion(rotation):
    vector = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0].reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return [1., 0., 0., 0.]
    return [float(np.cos(angle/2)), *(vector * (np.sin(angle/2)/angle)).tolist()]


class BaseRegistration:
    def __init__(self, config, matrix, distortion):
        self.config = config
        self.matrix, self.distortion = matrix, distortion
        self.last_pose, self.last_seen = None, 0
        self.pose_samples = deque(maxlen=config.get('pose_filter_samples', 5))
        self.fixed_pose = None
        self.fixed_samples = []
        self.fixed_last_capture_ns = 0
        self.fixed_config = config.get('fixed_base')
        if self.fixed_config:
            path = Path(self.fixed_config['file'])
            root = workspace_root()
            self.fixed_path = path if path.is_absolute() else root/path
            spec = {'markers': [{key: m.get(key) for key in (
                'dictionary', 'id', 'corners_base_m')} for m in self.markers],
                'table_height': config.get('base_table_z_m'),
                'reference': self.fixed_config['reference'],
                'model_sha256': self.fixed_config['model_sha256'],
                'camera_matrix': np.asarray(matrix, dtype=float).tolist(),
                'dist_coeffs': np.asarray(distortion, dtype=float).reshape(-1).tolist(),
                'camera_calibration_sha256': self.fixed_config.get('camera_calibration_sha256')}
            self.fixed_signature = hashlib.sha256(json.dumps(
                spec, sort_keys=True, allow_nan=False).encode()).hexdigest()
            if self.fixed_path.exists():
                saved = json.loads(self.fixed_path.read_text())
                if saved.get('version') == 1 and saved.get('signature') == self.fixed_signature:
                    pose = saved['pose']
                    if (pose.get('calibrated') is True and pose.get('fixed_to_table') is True
                            and pose.get('status') == 'fixed'
                            and pose.get('pose_source') == 'fixed_marker_calibration'
                            and len(pose.get('position', [])) == 3
                            and len(pose.get('quaternion', [])) == 4
                            and np.isfinite([*pose['position'], *pose['quaternion']]).all()
                            and abs(np.linalg.norm(pose['quaternion'])-1) < 1e-6):
                        self.fixed_pose = pose
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 2
        parameters.detectInvertedMarker = True
        parameters.minMarkerPerimeterRate = 0.01
        parameters.adaptiveThreshWinSizeMax = 53
        parameters.adaptiveThreshWinSizeStep = 4
        self.detectors = {}
        self.marker_rois = {}
        self.roi_seen = {}
        self.search_at = {}
        for marker in self.markers:
            dictionary = marker["dictionary"]
            if dictionary not in self.detectors:
                self.detectors[dictionary] = cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)), parameters
                )

    @property
    def markers(self):
        selected = self.config.get('localization_marker_ids')
        return [m for m in self.config.get('markers', [])
                if selected is None or m['id'] in selected]

    def lock_base(self, result, table_state):
        if not self.fixed_config or result['status'] != 'tracked':
            return result
        # Calibrate in the primary anchor coordinates, not an unverified auxiliary-only layout.
        if not table_state.get('anchor_visible'):
            self.fixed_samples.clear()
            return result
        captured_ns = table_state.get('captured_unix_ns', 0)
        if captured_ns <= self.fixed_last_capture_ns:
            return result
        self.fixed_last_capture_ns = captured_ns
        now = time.monotonic()
        position = np.array(result['measurement_position'])
        q = result['measurement_quaternion']
        yaw = 2*np.arctan2(q[3], q[0])
        if self.fixed_samples:
            sampled, previous_position, previous_yaw = self.fixed_samples[-1]
            change = np.arctan2(np.sin(yaw-previous_yaw), np.cos(yaw-previous_yaw))
            if (now-sampled > 1 or np.linalg.norm(position-previous_position) > .002
                    or abs(change) > .035):
                self.fixed_samples.clear()
            else:
                yaw = previous_yaw+change
        self.fixed_samples.append((now, position, yaw))
        count = self.fixed_config.get('sample_count', 20)
        self.fixed_samples = self.fixed_samples[-count:]
        result['fixed_calibration_samples'] = len(self.fixed_samples)
        result['fixed_calibration_required'] = count
        if len(self.fixed_samples) < count or now-self.fixed_samples[0][0] < 1:
            return result
        positions = np.array([p for _, p, _ in self.fixed_samples])
        yaws = np.array([y for _, _, y in self.fixed_samples])
        center, angle = np.median(positions, axis=0), np.median(yaws)
        spread = float(np.max(np.linalg.norm(positions-center, axis=1)))
        yaw_spread = float(np.max(np.abs(yaws-angle)))
        if spread > .002 or yaw_spread > .025:
            return result
        self.fixed_pose = {**result, 'status': 'fixed', 'position': center.tolist(),
                           'quaternion': [float(np.cos(angle/2)), 0, 0, float(np.sin(angle/2))],
                           'pose_source': 'fixed_marker_calibration', 'fixed_to_table': True,
                           'markers': [], 'calibrated_unix_ns': time.time_ns(),
                           'calibration_position_spread_m': spread,
                           'calibration_yaw_spread_rad': yaw_spread}
        for key in ('measurement_position', 'measurement_quaternion', 'pose_filter_samples'):
            self.fixed_pose.pop(key, None)
        saved = {'version': 1, 'signature': self.fixed_signature,
                 'reference': self.fixed_config['reference'], 'pose': self.fixed_pose,
                 'camera_matrix': np.asarray(self.matrix).tolist(),
                 'dist_coeffs': np.asarray(self.distortion).reshape(-1).tolist(),
                 'camera_calibration_sha256': self.fixed_config.get('camera_calibration_sha256'),
                 'markers': self.markers}
        self.fixed_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.fixed_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(saved, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.fixed_path)
        return self.fixed_result()

    def fixed_result(self):
        result = copy.deepcopy(self.fixed_pose)
        result['calibration_age_s'] = max(0, (time.time_ns()-result['calibrated_unix_ns'])/1e9)
        return result

    def detect_border(self, marker, image):
        """Recover a complete tag from observed edge lines when a shadow joins its border."""
        seed = np.asarray(marker['border_seed_native'], dtype=np.float32)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker['dictionary']))
        cells = dictionary.markerSize+2
        edge_lengths = np.linalg.norm(np.roll(seed, -1, axis=0)-seed, axis=1)
        if edge_lengths.min() < 4:
            return None
        # Detector quads can follow an inner edge. Search within one code cell,
        # then require an exact decode of the observed outer border.
        radius = max(5., .8*float(np.median(edge_lengths))/cells)
        margin = max(20., 2*radius)
        height, width = image.shape[:2]
        lo = np.maximum(np.floor(seed.min(axis=0)-margin).astype(int), 0)
        hi = np.minimum(np.ceil(seed.max(axis=0)+margin).astype(int), [width, height])
        crop = image[lo[1]:hi[1], lo[0]:hi[0]]
        if crop.size == 0:
            return None
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
        detected = cv2.createLineSegmentDetector().detect(gray)[0]
        if detected is None:
            return None
        lines = detected[:, 0].reshape(-1, 2, 2)+lo
        chosen = []
        for first, second in zip(seed, np.roll(seed, -1, axis=0)):
            tangent = second-first
            length = np.linalg.norm(tangent)
            tangent /= length
            normal = np.array([-tangent[1], tangent[0]])
            options = []
            for line in lines:
                direction = line[1]-line[0]
                span = np.linalg.norm(direction)
                offset = abs(np.dot(line.mean(axis=0)-(first+second)/2, normal))
                if span > length*.3 and abs(direction@tangent)/span > .98 and offset < radius:
                    options.append((offset, line))
            if not options:
                return None
            chosen.append(min(options, key=lambda option: option[0])[1])
        corners = []
        for previous, following in zip(np.roll(chosen, 1, axis=0), chosen):
            matrix = np.column_stack((previous[1]-previous[0], following[0]-following[1]))
            if abs(np.linalg.det(matrix)) < 1e-6:
                return None
            fraction = np.linalg.solve(matrix, following[0]-previous[0])[0]
            corners.append(previous[0]+fraction*(previous[1]-previous[0]))
        corners = np.asarray(corners, np.float32)
        if (np.max(np.linalg.norm(corners-seed, axis=1)) > max(8., np.sqrt(2)*radius)
                or not cv2.isContourConvex(corners)):
            return None
        side = cells*20
        transform = cv2.getPerspectiveTransform(corners, np.array(
            [[0, 0], [side-1, 0], [side-1, side-1], [0, side-1]], np.float32))
        source = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        warped = cv2.warpPerspective(source, transform, (side, side))
        threshold = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)[0]
        bits = np.array([[warped[y*20+6:y*20+14, x*20+6:x*20+14].mean() > threshold
                          for x in range(1, cells-1)] for y in range(1, cells-1)], np.uint8)
        # Exact code match: no synthesized pixels and no error correction for this fallback.
        ok, marker_id, rotation = dictionary.identify(bits, 0.0)
        if not ok or marker_id != marker['id']:
            return None
        self.marker_rois.pop((marker['dictionary'], marker['id']), None)
        return np.roll(corners, rotation, axis=0).reshape(1, 4, 2)

    def detect(self, detector, dictionary, image):
        """Decode small tags at two scales; track a crop after the first detection."""
        corners, ids, rejected = detect_markers(detector, image)
        found = {} if ids is None else dict(zip(ids.flatten(), corners))
        expected = {m['id'] for m in self.markers
                    if m['dictionary'] == dictionary}
        height, width = image.shape[:2]
        for marker_id in expected - found.keys():
            key = (dictionary, marker_id)
            roi = self.marker_rois.get(key)
            now = time.monotonic()
            if now-self.roi_seen.get(key, -1) > 2:
                roi = None
            if roi is None:
                if now-self.search_at.get(key, -1) < .5:
                    continue
                self.search_at[key] = now
                roi = (0, 0, width, height)
            x, y, right, bottom = roi
            crop = image[y:bottom, x:right]
            decoded, decoded_ids, _ = detect_markers(detector, cv2.resize(
                crop, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
            scale = 2
            if (decoded_ids is None or marker_id not in decoded_ids) and max(crop.shape[:2]) < 512:
                scale = 4
                decoded, decoded_ids, _ = detect_markers(detector, cv2.resize(
                    crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC))
            if decoded_ids is None or marker_id not in decoded_ids:
                continue
            corner = decoded[np.flatnonzero(decoded_ids.flatten() == marker_id)[0]]
            corner = (corner+.5)/scale-.5 + np.array([x, y], dtype=np.float32)
            # A second, local decode rejects isolated false detections from enlargement.
            lo = np.maximum(np.floor(corner[0].min(axis=0)-20).astype(int), 0)
            hi = np.minimum(np.ceil(corner[0].max(axis=0)+20).astype(int), [width, height])
            local = image[lo[1]:hi[1], lo[0]:hi[0]]
            check, check_ids, _ = detect_markers(detector, cv2.resize(
                local, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
            if check_ids is None or marker_id not in check_ids:
                gray = cv2.cvtColor(local, cv2.COLOR_BGR2GRAY) if local.ndim == 3 else local
                enhanced = cv2.createCLAHE(2, (4, 4)).apply(gray)
                check, check_ids, _ = detect_markers(detector, cv2.resize(
                    enhanced, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
            if check_ids is None or marker_id not in check_ids:
                continue
            other = check[np.flatnonzero(check_ids.flatten() == marker_id)[0]]
            other = (other+.5)/3-.5 + lo
            if np.max(np.linalg.norm(corner-other, axis=2)) <= 3:
                found[marker_id] = ((corner+other)/2).astype(np.float32)
        for marker in self.markers:
            if marker['dictionary'] == dictionary and marker.get('prefer_observed_border'):
                # A dark robot surrounding a white margin can produce an oversized quad.
                # Seed from this frame, including undecoded quads joined to shadows.
                # A historical pixel location must never veto a visible marker.
                candidates = ([found[marker['id']]] if marker['id'] in found else [])
                candidates += list(rejected)
                recovered = None
                for candidate in candidates:
                    observed = {**marker, 'border_seed_native': candidate[0].tolist()}
                    recovered = self.detect_border(observed, image)
                    if recovered is not None:
                        break
                if recovered is None:
                    found.pop(marker['id'], None)
                else:
                    found[marker['id']] = recovered
        for marker_id in expected & found.keys():
            corner = found[marker_id][0]
            margin = max(20, float(np.ptp(corner, axis=0).max()))
            lo = np.maximum(np.floor(corner.min(axis=0)-margin).astype(int), 0)
            hi = np.minimum(np.ceil(corner.max(axis=0)+margin).astype(int), [width, height])
            self.marker_rois[dictionary, marker_id] = (*lo, *hi)
            self.roi_seen[dictionary, marker_id] = time.monotonic()
        return found

    def flat_pose(self, obj, img, table_state):
        """Known corner heights and desk support constrain base height, roll and pitch."""
        height = self.config['base_table_z_m']
        table_rotation = np.asarray(table_state['camera_rotation'])
        camera = np.asarray(table_state['camera_position'])
        translation = -table_rotation@camera
        world = pixels_on_plane(img, self.matrix, self.distortion, table_rotation,
                                translation, height+obj[:, 2])
        a, b = obj[:, :2]-obj[:, :2].mean(axis=0), world[:, :2]-world[:, :2].mean(axis=0)
        left, _, right = np.linalg.svd(a.T@b)
        planar = right.T@left.T
        if np.linalg.det(planar) < 0:
            right[-1] *= -1
            planar = right.T@left.T
        rotation = np.eye(3)
        rotation[:2, :2] = planar
        position = np.r_[world[:, :2].mean(axis=0)-planar@obj[:, :2].mean(axis=0), height]
        camera_from_base = table_rotation@rotation
        rvec = cv2.Rodrigues(camera_from_base)[0]
        tvec = (table_rotation@(position-camera)).reshape(3, 1)
        return rvec, tvec

    def process(self, frame, table_state, native_frame=None, low_to_native=None):
        if self.fixed_pose is not None:
            # A bolted base keeps its table transform. No detector or base fit after calibration.
            return self.fixed_result()
        detections = []
        points, pixels = [], []
        image = native_frame if native_frame is not None else frame
        inverse = (
            cv2.invertAffineTransform(np.asarray(low_to_native, dtype=float))
            if (low_to_native is not None)
            else None
        )
        for dictionary, detector in self.detectors.items():
            found = self.detect(detector, dictionary, image)
            for marker in self.markers:
                if marker["dictionary"] != dictionary:
                    continue
                if marker['id'] not in found:
                    continue
                corner = found[marker['id']][0]
                if cv2.contourArea(corner) < 16:
                    continue
                if inverse is not None:
                    corner = cv2.transform(corner.reshape(1, 4, 2), inverse)[0]
                detections.append(
                    {
                        "dictionary": dictionary,
                        "id": marker["id"],
                        "mount": marker["mount"],
                        "corners": corner.tolist(),
                    }
                )
                if marker.get("corners_base_m") is not None:
                    points.extend(marker["corners_base_m"])
                    pixels.extend(corner)
        result = {
            "status": "mount_calibration_missing",
            "markers": detections,
            "marker_size_m": self.config["marker_size_m"],
            "reference": "primary tabletop anchor",
            "calibrated": False,
        }
        calibrated = [
            marker
            for marker in self.markers
            if marker.get("corners_base_m") is not None
        ]
        if not calibrated:
            return result
        result["status"] = "markers_lost"
        if not table_state["marker_visible"]:
            result["status"] = "table_reference_lost"
            return result
        if len(points) >= 4:
            obj = np.asarray(points, dtype=float)
            img = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
            constrained = self.config.get('base_constraint') == 'flat_on_table'
            if constrained:
                rvec, tvec = self.flat_pose(obj, img, table_state)
                ok = True
            else:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, self.matrix, self.distortion, flags=cv2.SOLVEPNP_SQPNP)
            if ok:
                if not constrained:
                    rvec, tvec = cv2.solvePnPRefineLM(
                        obj, img, self.matrix, self.distortion, rvec, tvec)
                camera_from_base = cv2.Rodrigues(rvec)[0]
                projected = cv2.projectPoints(obj, rvec, tvec, self.matrix, self.distortion)[0]
                error = float(np.sqrt(np.mean(np.sum((img - projected) ** 2, axis=2))))
                table_rotation = np.asarray(table_state["camera_rotation"])
                rotation = table_rotation.T @ camera_from_base
                position = (
                    np.asarray(table_state["camera_position"]) + table_rotation.T @ tvec[:, 0]
                )
                in_front = np.all((obj @ camera_from_base.T + tvec[:, 0])[:, 2] > 0)
                result.update(status='pose_rejected', reprojection_error_px=error,
                              max_reprojection_error_px=self.config.get(
                                  'max_reprojection_error_px', 1.5),
                              constraint='base_flat_on_table' if constrained else 'marker_pnp')
                if not in_front:
                    result['rejection_reason'] = 'marker corners are behind the camera'
                elif error > result['max_reprojection_error_px']:
                    result['rejection_reason'] = 'known marker mount does not fit observed corners'
                elif rotation[2, 2] <= .9:
                    result['rejection_reason'] = 'base tilt violates table support'
                elif not -.01 < position[2] < .1:
                    result['rejection_reason'] = 'base height violates table support'
                if (
                    in_front
                    and error <= self.config.get("max_reprojection_error_px", 1.5)
                    and rotation[2, 2] > 0.9
                    and -0.01 < position[2] < 0.1
                ):
                    measurement_position = position.copy()
                    measurement_quaternion = rotation_quaternion(rotation)
                    if constrained:
                        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                        now = time.monotonic()
                        if self.pose_samples:
                            sampled, previous_position, previous_yaw = self.pose_samples[-1]
                            change = np.arctan2(np.sin(yaw-previous_yaw), np.cos(yaw-previous_yaw))
                            if (now-sampled > 1 or np.linalg.norm(position-previous_position) > .002
                                    or abs(change) > .06):
                                self.pose_samples.clear()
                            else:
                                yaw = previous_yaw+change
                        self.pose_samples.append((now, position.copy(), yaw))
                        filtered_position = np.median([p for _, p, _ in self.pose_samples], axis=0)
                        filtered_yaw = np.median([y for _, _, y in self.pose_samples])
                        cs, sn = np.cos(filtered_yaw), np.sin(filtered_yaw)
                        filtered_rotation = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
                        filtered_rvec = cv2.Rodrigues(table_rotation@filtered_rotation)[0]
                        filtered_tvec = table_rotation@(
                            filtered_position-np.asarray(table_state['camera_position']))
                        filtered_pixels = cv2.projectPoints(
                            obj, filtered_rvec, filtered_tvec, self.matrix, self.distortion)[0]
                        filtered_error = float(np.sqrt(np.mean(np.sum(
                            (img-filtered_pixels)**2, axis=2))))
                        if filtered_error <= self.config.get('max_reprojection_error_px', 1.5):
                            position, rotation = filtered_position, filtered_rotation
                            error = filtered_error
                    result.update(
                        status="tracked",
                        calibrated=True,
                        position=position.tolist(),
                        quaternion=rotation_quaternion(rotation),
                        reprojection_error_px=error,
                        pose_source="known_marker_mounts",
                        constraint='base_flat_on_table' if constrained else 'marker_pnp',
                        measurement_position=measurement_position.tolist(),
                        measurement_quaternion=measurement_quaternion,
                        pose_filter_samples=len(self.pose_samples),
                    )
                    result.pop('rejection_reason', None)
                    self.last_pose, self.last_seen = dict(result), time.monotonic()
        if result["status"] != "tracked" and self.last_pose is not None:
            # Both tags are fixed to the base. A cached pose is explicitly labelled and timed.
            age = time.monotonic() - self.last_seen
            if age < self.config.get("base_hold_s", 2):
                result.update(
                    {key: self.last_pose[key] for key in ("position", "quaternion", "pose_source")}
                )
                result.update(status="held", calibrated=True, pose_age_s=age)
        return self.lock_base(result, table_state)
