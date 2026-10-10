"""Remote-only AprilTag plane reconstruction and T silhouette tracking.

World: tag center, +X tag right, +Y tag up, +Z out of the tabletop.
All geometric estimation, segmentation and annotation happen on the camera host.
"""

import base64
import hashlib
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.perception.tabletop_tracking import TTracker


def pixels_on_plane(pixels, matrix, distortion, rotation, translation, height=0):
    normalized = cv2.undistortPoints(
        np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2), matrix, distortion
    ).reshape(-1, 2)
    center = -rotation.T @ translation.reshape(3)
    rays = np.column_stack((normalized, np.ones(len(normalized)))) @ rotation
    distances = (height - center[2]) / rays[:, 2]
    if np.any(distances <= 0) or not np.isfinite(distances).all():
        raise ValueError("Points do not intersect the visible tabletop plane")
    return center + distances[:, None] * rays


def fit_t(points):
    """Fit a cap + centered stem, selecting orientation by silhouette IoU.

    Positive local Y points toward the cap. Two boxes preserve concavity in MuJoCo.
    """
    points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    origin = points.mean(axis=0)
    best = None

    def evaluate(degrees):
        nonlocal best
        angle = math.radians(degrees)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        local = (points - origin) @ rotation
        lower, upper = local.min(axis=0), local.max(axis=0)
        extent = upper - lower
        if min(extent) <= 0 or not 0.4 < extent[0] / extent[1] < 2.5:
            return
        raster = np.zeros((80, 80), np.uint8)
        coords = (local - lower) / extent * 79
        coords[:, 1] = 79 - coords[:, 1]
        cv2.fillPoly(raster, [np.round(coords).astype(np.int32)], 1)
        # Estimate stem edges from the lower half of the silhouette.
        occupied = np.flatnonzero(raster[40:75].mean(axis=0) > 0.65)
        if len(occupied) < 8 or len(occupied) > 42:
            return
        left, right = int(occupied[0]), int(occupied[-1]) + 1
        if abs((left + right) / 2 - 40) > 9:
            return
        integral = cv2.integral(raster)
        area = int(raster.sum())
        for cap in range(10, 38):
            intersection = (integral[cap, 80]
                            + integral[80, right] - integral[cap, right]
                            - integral[80, left] + integral[cap, left])
            template_area = cap * 80 + (80 - cap) * (right - left)
            score = float(intersection / max(1, area + template_area - intersection))
            if best is None or score > best["iou"]:
                center = origin + rotation @ ((lower + upper) / 2)
                best = {"iou": score, "xy": center.tolist(), "yaw": angle,
                        "width": float(extent[0]), "length": float(extent[1]),
                        "cap_depth": float(extent[1] * cap / 80),
                        "stem_width": float(extent[0] * (right - left) / 80)}

    for degrees in range(0, 360, 5):
        evaluate(degrees)
    if best:
        degrees = math.degrees(best["yaw"])
        for value in np.arange(degrees - 4, degrees + 4.1, 0.5):
            evaluate(float(value))
    return best


class Perception:
    def __init__(self, config):
        self.config = config
        self.matrix = np.array(config["camera_matrix"], dtype=float)
        self.distortion = np.array(config["dist_coeffs"], dtype=float)
        self.scale = config["marker_size_m"] or 1.0
        self.units = "m" if config["marker_size_m"] else "marker_side"
        self.wood_height = config["wood_height_m"] or 0.0
        if self.wood_height and self.units != "m":
            raise ValueError("Wood height requires a measured marker size")
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["dictionary"])),
            parameters,
        )
        self.additional_detectors = [(marker, cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker["dictionary"])),
            parameters)) for marker in config.get("additional_markers", [])]
        half = self.scale / 2
        self.tag_points = np.array([[-half, half, 0], [half, half, 0],
                                    [half, -half, 0], [-half, -half, 0]], float)
        self.models = {}
        self.secondary_layouts = {}
        self.trackers = {}
        self.robot_registration = None
        from lerobot_arm_control.perception.tabletop_registration import (
            BaseRegistration,
            base_registration_configs,
        )

        self.base_registrations = {
            entry['arm']: BaseRegistration(entry, self.matrix, self.distortion)
            for entry in base_registration_configs(config)
        }
        if config.get('robot', {}).get('enabled'):
            self.robot_registration = self.base_registrations[config['robot']['arm']]
        self.tool_marker = None
        tool = config.get('robot', {}).get('tool_marker', {})
        if tool.get('enabled'):
            from lerobot_arm_control.perception.tool_marker import ToolMarker

            identities = {(marker['dictionary'], marker['id']) for entry in
                          base_registration_configs(config) for marker in entry['markers']}
            identities.add((config['dictionary'], config['marker_id']))
            identities.update((entry['dictionary'], entry['id']) for entry in
                              config.get('additional_markers', []))
            if (tool['dictionary'], tool['id']) in identities:
                raise ValueError('End-effector marker must differ from all stationary markers')
            self.tool_marker = ToolMarker(tool, self.matrix, self.distortion)
        known = config.get("t_model")
        if known and all(known.get(key) is not None for key in (
                "width", "length", "cap_depth", "stem_width")):
            for name, height in (("black_t", 0), ("wood_t", self.wood_height)):
                self.models[name] = {**known, "height": height, "height_measured": True,
                                     "dimension_source": "user_measurement"}
                self.trackers[name] = TTracker(
                    self.models[name], static=name == "black_t",
                    hold_seconds=config.get("occlusion_hold_s", 2.0))

    def static_reference(self):
        reference = {key: self.config.get(key) for key in (
            "dictionary", "marker_id", "marker_size_m", "t_model")}
        reference['camera_calibration_sha256'] = self.config.get(
            'camera_calibration', {}).get('sha256')
        return reference

    def restore_static_map(self, path):
        """Restore only the fixed paper target, retaining its true observation age."""
        path = Path(path)
        if not path.exists():
            return
        saved = json.loads(path.read_text())
        if saved.get("reference") != self.static_reference():
            return
        known = {m['dictionary']+':'+str(m['id'])
                 for m in self.config.get('additional_markers', [])}
        for key, layout in saved.get('secondary_layouts', {}).items():
            points = np.asarray(layout, dtype=float)
            if (key in known and points.shape == (4, 3) and np.isfinite(points).all()
                    and np.max(np.abs(points[:, 2])) < 1e-6):
                sides = np.linalg.norm(points-np.roll(points, 1, axis=0), axis=1)
                if np.all(np.abs(sides-self.scale) < self.scale*.05):
                    self.secondary_layouts[key] = points.tolist()
        pose = saved.get("black_t")
        tracker = self.trackers.get("black_t")
        if tracker and pose and np.isfinite([*pose["xy"], pose["yaw"]]).all():
            tracker.last = {key: pose[key] for key in ("xy", "yaw", "iou")}
            age = max(0, (time.time_ns()-saved["observed_unix_ns"])/1e9)
            tracker.last_observed = time.monotonic()-age

    def save_static_map(self, path, state):
        pose = state.get("objects", {}).get("black_t")
        if not pose or not pose.get("observed"):
            pose = None
        if pose is None and not self.secondary_layouts:
            return
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        saved = {"reference": self.static_reference(), "black_t": pose,
                 'secondary_layouts': state.get('secondary_layouts', self.secondary_layouts),
                 "observed_unix_ns": state["captured_unix_ns"]}
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(saved, allow_nan=False))
        temporary.replace(path)

    def estimate_pose(self, points_3d, points_2d, square=False):
        points_3d = np.array(points_3d, dtype=float)
        points_2d = np.array(points_2d, dtype=float).reshape(-1, 1, 2)
        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            points_3d, points_2d, self.matrix, self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE if square else cv2.SOLVEPNP_IPPE)
        solutions = []
        for rvec, tvec in zip(rvecs, tvecs, strict=True):
            rvec, tvec = cv2.solvePnPRefineLM(points_3d, points_2d, self.matrix,
                                            self.distortion, rvec, tvec)
            rotation = cv2.Rodrigues(rvec)[0]
            center = -rotation.T @ tvec.reshape(3)
            projected = cv2.projectPoints(points_3d, rvec, tvec,
                                          self.matrix, self.distortion)[0]
            error = float(np.sqrt(np.mean(np.sum((projected-points_2d)**2, axis=2))))
            if center[2] > 0 and tvec[2, 0] > 0:
                solutions.append((error, rotation, tvec, rvec, center))
        return min(solutions, key=lambda x: x[0]) if solutions else None

    def process(self, frame, sequence, captured_ns, native_frame=None, low_to_native=None,
                robot=None, *, marker_only=False):
        started = time.monotonic()
        if list(frame.shape[1::-1]) != self.config["image_size"]:
            raise ValueError("Camera resolution differs from calibrated image_size")
        annotated = frame.copy()
        detection_frame = native_frame if native_frame is not None else frame
        if native_frame is not None and low_to_native is None:
            raise ValueError("Native marker detection requires the calibrated pixel mapping")
        native_to_low = (cv2.invertAffineTransform(np.asarray(low_to_native, dtype=float))
                         if native_frame is not None else None)

        def detect(detector):
            detected, values, _ = detector.detectMarkers(detection_frame)
            if native_to_low is not None:
                detected = [cv2.transform(corner, native_to_low) for corner in detected]
            return detected, values

        # The calibrated low-resolution view may crop a complete native marker.
        # Transform its corners, including coordinates outside that crop, before PnP.
        corners, ids = detect(self.detector)
        secondary = {}
        all_corners = list(corners)
        detected_markers = [] if ids is None else [
            {"dictionary": self.config["dictionary"], "id": int(value),
             "corners": corner[0].tolist()}
            for corner, value in zip(corners, ids.flatten(), strict=True)]
        for marker, detector in self.additional_detectors:
            extra_corners, extra_ids = detect(detector)
            if extra_ids is None:
                continue
            all_corners.extend(extra_corners)
            cv2.aruco.drawDetectedMarkers(annotated, extra_corners, extra_ids)
            for corner, value in zip(extra_corners, extra_ids.flatten(), strict=True):
                detected_markers.append({**marker, "id": int(value),
                                         "corners": corner[0].tolist()})
                if int(value) == marker["id"]:
                    secondary[marker["dictionary"] + ":" + str(marker["id"])] = corner
        state = {"type": "state", "version": 1, "sequence": sequence,
                 "captured_unix_ns": captured_ns, "units": self.units,
                 "marker_visible": False, "objects": {}, "models": self.models,
                 "detected_markers": detected_markers,
                 "wood_height_measured": self.config["wood_height_m"] is not None}
        if marker_only:
            state['observation_scope'] = 'table and tool markers only; no base registration'
        candidates = [] if ids is None else [i for i, value in enumerate(ids.flatten())
                                            if int(value) == self.config["marker_id"]]
        object_points, image_points = [], []
        if candidates:
            object_points.extend(self.tag_points)
            image_points.extend(corners[candidates[0]][0])
        for key, tag in secondary.items():
            if key in self.secondary_layouts:
                object_points.extend(self.secondary_layouts[key])
                image_points.extend(tag[0])
        if object_points:
            # A learned second tag keeps the original world frame when the anchor is hidden.
            solution = self.estimate_pose(object_points, image_points,
                                          square=bool(candidates) and len(object_points) == 4)
            state["pose_source"] = ("both" if candidates and len(object_points) > 4
                                    else "anchor" if candidates else "secondary")
            if candidates and (solution is None or solution[0] > (
                    self.config["max_reprojection_error_px"])):
                state["fusion_error_px"] = solution[0] if solution else None
                solution = self.estimate_pose(self.tag_points, corners[candidates[0]], square=True)
                state["pose_source"] = "anchor_fallback"
            if solution:
                error, rotation, translation, rvec, center = solution
                state["reprojection_error_px"] = error
                if error <= self.config["max_reprojection_error_px"]:
                    state.update(marker_visible=True, camera_position=center.tolist(),
                                 camera_rotation=rotation.tolist(),
                                 anchor_visible=bool(candidates))
                    if candidates:
                        tag = corners[candidates[0]]
                        state["tag_corners"] = tag[0].tolist()
                        cv2.aruco.drawDetectedMarkers(annotated, [tag],
                                                     np.array([[self.config["marker_id"]]]))
                    # A stationary registered secondary also defines the original
                    # frame when the primary is outside the camera view.
                    for key, extra in secondary.items():
                        if key in self.secondary_layouts:
                            continue
                        measured = pixels_on_plane(extra, self.matrix, self.distortion,
                                                   rotation, translation)[:, :2]
                        # All configured table tags share the confirmed black edge.
                        source = self.tag_points[:, :2]
                        covariance = source.T @ (measured - measured.mean(axis=0))
                        left, _, right = np.linalg.svd(covariance)
                        planar_rotation = left @ right
                        aligned = source @ planar_rotation + measured.mean(axis=0)
                        residual = np.sqrt(np.mean((aligned-measured)**2)) / self.scale
                        if residual < .08 and np.linalg.det(planar_rotation) > 0:
                            self.secondary_layouts[key] = np.column_stack((
                                aligned, np.zeros(4))).tolist()
                    state["secondary_layouts"] = self.secondary_layouts
                    cv2.drawFrameAxes(annotated, self.matrix, self.distortion,
                                      rvec, translation, self.scale * .5)
                    hsv = None if marker_only else cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    for name, height in (() if marker_only else
                                         (("black_t", 0), ("wood_t", self.wood_height))):
                        limits = self.config["black_hsv" if name == "black_t" else "wood_hsv"]
                        mask = cv2.inRange(hsv, np.array(limits[0]), np.array(limits[1]))
                        # Remove all complete tags before fitting T shapes.
                        for detected in all_corners:
                            cv2.fillConvexPoly(mask, np.int32(detected[0]), 0)
                        detections = []
                        for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                        cv2.CHAIN_APPROX_SIMPLE)[0]:
                            x, y, width, length = cv2.boundingRect(contour)
                            if (cv2.contourArea(contour) < 100 or x < 2 or y < 2
                                    or x + width >= frame.shape[1] - 2
                                    or y + length >= frame.shape[0] - 2):
                                continue
                            try:
                                points = pixels_on_plane(contour, self.matrix, self.distortion,
                                                         rotation, translation, height)[:, :2]
                            except ValueError:
                                continue
                            area = cv2.contourArea(points.astype(np.float32)) / self.scale**2
                            if not .01 < area < 1.5:
                                continue
                            fitted = fit_t(points)
                            if fitted and fitted["iou"] >= self.config["min_t_iou"]:
                                fitted["contour_xy"] = cv2.approxPolyDP(
                                    points.astype(np.float32), .01*self.scale, True
                                ).reshape(-1, 2).tolist()
                                detections.append((fitted, contour, points))
                        if name in self.trackers:
                            tracked = self.trackers[name].update(
                                [item[0] for item in detections], mask, self.matrix,
                                self.distortion, rotation, translation, time.monotonic())
                            if tracked is not None:
                                state["objects"][name] = tracked
                                points_3d = np.column_stack((tracked["complete_outline_xy"],
                                                            np.full(8, height)))
                                outline = cv2.projectPoints(points_3d, rvec, translation,
                                                           self.matrix, self.distortion)[0]
                                color = (0, 200, 0) if tracked["observed"] else (0, 170, 255)
                                cv2.polylines(annotated, [np.int32(outline)], True, color, 1)
                                x, y = np.int32(outline).reshape(-1, 2).min(axis=0)
                                cv2.putText(annotated, f'{name}: {tracked["tracking_status"]}',
                                            (max(0, x-10), max(35, y-5)),
                                            cv2.FONT_HERSHEY_SIMPLEX, .35, color, 1)
                            continue
                        if not detections:
                            continue
                        fitted, contour, points = max(detections, key=lambda x: x[0]["iou"])
                        if name not in self.models:
                            self.models[name] = {key: fitted[key] for key in (
                                "width", "length", "cap_depth", "stem_width")}
                            self.models[name].update(height=height, height_measured=(
                                name == "black_t" or self.config["wood_height_m"] is not None))
                        dimensions = self.models[name]
                        if any(not .7 < fitted[key] / dimensions[key] < 1.3
                               for key in ("width", "length")):
                            continue
                        outline = cv2.approxPolyDP(points.astype(np.float32),
                                                  .01 * self.scale, True).reshape(-1, 2)
                        state["objects"][name] = {**fitted, "z": height / 2,
                                                  "contour_xy": outline.tolist()}
                        cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 1)
                        cv2.putText(annotated, f'{name} {fitted["iou"]:.2f}',
                                    (max(0, x - 20), max(12, y - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 180, 0), 1)
        if self.base_registrations and not marker_only:
            from lerobot_arm_control.perception.tabletop_registration import base_distances

            poses = {}
            for index, (name, registration) in enumerate(self.base_registrations.items()):
                poses[name] = registration.process(frame, state, native_frame, low_to_native,
                                                   robot)
                for marker in poses[name]['markers']:
                    cv2.polylines(annotated, [np.int32(marker['corners'])], True,
                                  (255, 180, 0), 1)
                cv2.putText(annotated, name+' base: '+poses[name]['status'],
                            (10, 40+index*16), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 180, 0), 1)
            state['base_registrations'] = poses
            state['base_distances'] = base_distances(poses)
            if self.robot_registration:
                state['robot_registration'] = poses[self.config['robot']['arm']]
        if self.tool_marker:
            measured = self.tool_marker.process(frame, state, native_frame, low_to_native)
            state['tool_marker'] = measured
            if measured.get('corners'):
                cv2.polylines(annotated, [np.int32(measured['corners'])], True, (180, 0, 255), 1)
            cv2.putText(annotated, 'tool marker: '+measured['status'], (10, 88),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (180, 0, 255), 1)
        state["processing_ms"] = (time.monotonic() - started) * 1000
        cv2.putText(annotated, f'AprilTag world: {"OK" if state["marker_visible"] else "LOST"}'
                    f' | {self.units}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (0, 180, 0) if state["marker_visible"] else (0, 0, 255), 1)
        for key, picture in (("jpeg", frame), ("annotated_jpeg", annotated)):
            ok, encoded = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            state[key] = base64.b64encode(encoded).decode("ascii")
        return state


def read_config(path):
    config = json.loads(Path(path).read_text())
    for key in ("marker_size_m", "wood_height_m"):
        value = config[key]
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"{key} must be positive or null")
    known = config.get("t_model")
    if known:
        if not config["marker_size_m"]:
            raise ValueError("Known metric T dimensions require a measured marker size")
        for key in ("width", "length", "cap_depth", "stem_width"):
            if known[key] is not None and (not math.isfinite(known[key]) or known[key] <= 0):
                raise ValueError(f"T {key} must be positive")
        if known["cap_depth"] is not None and known["cap_depth"] >= known["length"]:
            raise ValueError("T cap depth must be less than length")
        if known["stem_width"] is not None and known["stem_width"] >= known["width"]:
            raise ValueError("T stem width must be less than width")
    calibration = config.get('camera_calibration')
    if calibration:
        source = workspace_root() / calibration['file']
        payload = source.read_bytes()
        if hashlib.sha256(payload).hexdigest() != calibration['sha256']:
            raise ValueError('Camera calibration source hash differs from the selected record')
        native = json.loads(payload)
        mapping = np.eye(3)
        mapping[:2] = config['low_to_capture_affine']
        if (config['capture_size'] != native['image_size']
                or not np.allclose(mapping @ np.asarray(config['camera_matrix']),
                                   native['camera_matrix'], rtol=0, atol=1e-9)
                or config['dist_coeffs'] != native['dist_coeffs']):
            raise ValueError(
                'Runtime camera parameters differ from the selected native calibration')
    return config
