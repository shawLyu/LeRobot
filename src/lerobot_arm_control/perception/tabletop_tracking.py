"""Remote known-shape tracking from partially visible T boundaries."""

import math

import cv2
import numpy as np


def t_outline(dimensions):
    w, length = dimensions["width"], dimensions["length"]
    cap, stem = dimensions["cap_depth"], dimensions["stem_width"]
    return np.array([[-w/2, length/2], [w/2, length/2], [w/2, length/2-cap],
                     [stem/2, length/2-cap], [stem/2, -length/2], [-stem/2, -length/2],
                     [-stem/2, length/2-cap], [-w/2, length/2-cap]], dtype=float)


def world_outline(dimensions, xy, yaw):
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return t_outline(dimensions) @ rotation.T + xy


def boundary_samples(dimensions):
    polygon = t_outline(dimensions)
    points, groups = [], []
    for index, start in enumerate(polygon):
        end = polygon[(index+1) % len(polygon)]
        count = max(3, round(np.linalg.norm(end-start)/.001))
        points.extend(start + np.arange(count)[:, None]/count * (end-start))
        groups.extend([index]*count)
    return np.array(points), np.array(groups)


def match_partial(mask, origin, resolution, dimensions, previous):
    """Search a metric ROI; use both cap and stem edges to constrain pose.

    Missing template edges are tolerated. An entirely filled mask has no boundary
    evidence, and a stem alone is ambiguous along its long axis: neither updates pose.
    """
    edges = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
    if not edges.any():
        return None
    distance = cv2.distanceTransform(255-edges, cv2.DIST_L2, 3) * resolution
    samples, groups = boundary_samples(dimensions)
    polygon = t_outline(dimensions)
    xs = np.arange(-dimensions["width"]/2, dimensions["width"]/2, .002)
    ys = np.arange(-dimensions["length"]/2, dimensions["length"]/2, .002)
    xx, yy = np.meshgrid(xs, ys)
    inside = ((yy > dimensions["length"]/2-dimensions["cap_depth"])
              | (np.abs(xx) < dimensions["stem_width"]/2))
    interior = np.column_stack((xx[inside], yy[inside]))
    origin = np.array(origin)

    def evaluate(poses):
        poses = np.array(poses)
        cosine, sine = np.cos(poses[:, 2]), np.sin(poses[:, 2])

        def transform(points):
            x = poses[:, 0, None] + cosine[:, None]*points[:, 0] - sine[:, None]*points[:, 1]
            y = poses[:, 1, None] + sine[:, None]*points[:, 0] + cosine[:, None]*points[:, 1]
            return np.round((np.stack((x, y), axis=2)-origin)/resolution).astype(int)

        coordinates = transform(samples)
        x = coordinates[:, :, 0].clip(0, mask.shape[1]-1)
        y = coordinates[:, :, 1].clip(0, mask.shape[0]-1)
        distances = distance[y, x]
        supported = distances < .0018
        support = supported.mean(axis=1)
        cap_support = np.max(np.column_stack([
            supported[:, groups == i].mean(axis=1) for i in (0, 1, 2, 6, 7)]), axis=1)
        stem_support = np.max(np.column_stack([
            supported[:, groups == i].mean(axis=1) for i in (3, 4, 5)]), axis=1)
        coordinates = transform(interior)
        coverage = (mask[coordinates[:, :, 1].clip(0, mask.shape[0]-1),
                         coordinates[:, :, 0].clip(0, mask.shape[1]-1)] > 0).mean(axis=1)
        motion = np.linalg.norm(poses[:, :2]-previous["xy"], axis=1)
        turn = np.abs(np.arctan2(np.sin(poses[:, 2]-previous["yaw"]),
                                np.cos(poses[:, 2]-previous["yaw"])))
        score = (.7*support + .3*np.exp(-(distances/.002)**2).mean(axis=1)
                 + .1*coverage - .04*motion/.02 - .015*turn/.5)
        valid = (support > .42) & (cap_support > .45) & (stem_support > .45) & (coverage > .32)
        score[~valid] = -np.inf
        return score, support, coverage

    poses = [[previous["xy"][0]+dx, previous["xy"][1]+dy, previous["yaw"]+angle]
             for angle in np.deg2rad(np.arange(-25, 26, 5))
             for dx in np.arange(-.02, .0201, .004)
             for dy in np.arange(-.02, .0201, .004)]
    score, _, _ = evaluate(poses)
    if not np.isfinite(score).any():
        return None
    coarse = np.array(poses)[np.argmax(score)]
    fine = [[coarse[0]+dx, coarse[1]+dy, coarse[2]+angle]
            for angle in np.deg2rad(np.arange(-4, 4.1, 1))
            for dx in np.arange(-.003, .0031, .001)
            for dy in np.arange(-.003, .0031, .001)]
    score, support, coverage = evaluate(fine)
    if not np.isfinite(score).any():
        return None
    index = int(np.argmax(score))
    best = np.array(fine[index])
    # Very different poses with nearly equal edge evidence are not a new observation.
    alternative = (np.linalg.norm(np.array(fine)[:, :2]-best[:2], axis=1) > .004)
    if np.any(alternative & (score > score[index]-.015)):
        return None
    return {"xy": best[:2].tolist(), "yaw": float(best[2]),
            "edge_support": float(support[index]), "visible_fraction": float(coverage[index]),
            "iou": float(coverage[index]), "contour_xy": [],
            "complete_outline_xy": (polygon @ np.array([
                [math.cos(best[2]), math.sin(best[2])],
                [-math.sin(best[2]), math.cos(best[2])]]) + best[:2]).tolist()}


class TTracker:
    def __init__(self, dimensions, *, static=False, hold_seconds=2.0):
        self.dimensions = dimensions
        self.static, self.hold_seconds = static, hold_seconds
        self.last = None
        self.last_observed = 0.0
        self.undistort_maps = None

    def cached(self, now):
        return self._state('static_cached' if self.static else (
            'held' if now-self.last_observed <= self.hold_seconds else 'lost'), now, False)

    def update(self, detections, mask, matrix, distortion, rotation, translation, now):
        matches = [item for item in detections if all(
            .85 < item[key]/self.dimensions[key] < 1.2 for key in ("width", "length"))]
        if matches:
            if self.last:
                observed = min(matches, key=lambda x: np.linalg.norm(
                    np.array(x["xy"])-self.last["xy"]))
            else:
                observed = max(matches, key=lambda x: x["iou"])
            # A clear observation can reacquire a moved paper target; otherwise the
            # static target keeps its last measured pose during occlusion.
            self.last = dict(observed)
            self.last_observed = now
            return self._state("observed", now, True)
        if self.last is None:
            return None
        if self.static or now-self.last_observed <= self.hold_seconds:
            resolution = .00075
            origin = np.array(self.last["xy"]) - .08
            if self.undistort_maps is None:
                self.undistort_maps = cv2.initUndistortRectifyMap(
                    matrix, distortion, None, matrix, mask.shape[::-1], cv2.CV_32FC1)
            undistorted = cv2.remap(mask, *self.undistort_maps, cv2.INTER_NEAREST)
            height = self.dimensions["height"]
            plane_to_image = matrix @ np.column_stack((
                rotation[:, 0], rotation[:, 1], translation.reshape(3)+rotation[:, 2]*height))
            world_to_roi = np.array([[1/resolution, 0, -origin[0]/resolution],
                                     [0, 1/resolution, -origin[1]/resolution], [0, 0, 1]])
            rectified = cv2.warpPerspective(undistorted,
                                            world_to_roi @ np.linalg.inv(plane_to_image),
                                            (214, 214), flags=cv2.INTER_NEAREST)
            match = match_partial(rectified, origin, resolution, self.dimensions, self.last)
            if match is not None:
                self.last = match
                self.last_observed = now
                state = self._state("partial", now, True)
                state.update(edge_support=match["edge_support"],
                             visible_fraction=match["visible_fraction"])
                return state
        return self._state("static_cached" if self.static else (
            "held" if now-self.last_observed <= self.hold_seconds else "lost"), now, False)

    def _state(self, status, now, observed):
        state = {**self.last, **self.dimensions, "tracking_status": status,
                 "observed": observed, "pose_valid": status != "lost",
                 "observation_age_s": max(0., now-self.last_observed),
                 "z": self.dimensions["height"]/2}
        state["complete_outline_xy"] = world_outline(
            self.dimensions, state["xy"], state["yaw"]).tolist()
        return state
