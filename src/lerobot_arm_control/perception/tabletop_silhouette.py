"""Known-height T prism fitting for oblique cameras that also see the side walls."""

import cv2
import numpy as np

from .tabletop_tracking import t_outline, world_outline


def fit_extruded_t(contour, dimensions, matrix, distortion, rotation, translation, initial,
                   previous=None):
    """Fit planar position/yaw without changing measured dimensions or camera geometry."""
    x, y, width, height = cv2.boundingRect(contour)
    origin = np.array([x-80, y-80])
    shape = (height+160, width+160)
    observed = np.zeros(shape, np.uint8)
    cv2.fillPoly(observed, [contour.reshape(-1, 2)-origin], 1)
    polygon = t_outline(dimensions)
    rvec = cv2.Rodrigues(rotation)[0]

    def evaluate(poses):
        poses = np.asarray(poses)
        cosine, sine = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        xy = np.stack((poses[:, 0, None]+cosine*polygon[:, 0]-sine*polygon[:, 1],
                       poses[:, 1, None]+sine*polygon[:, 0]+cosine*polygon[:, 1]), axis=2)
        lower = np.concatenate((xy, np.zeros((*xy.shape[:2], 1))), axis=2)
        upper = lower.copy()
        upper[:, :, 2] = dimensions['height']
        points = np.concatenate((lower, upper), axis=1)
        camera_points = points@rotation.T+translation.reshape(3)
        pixels = cv2.projectPoints(points.reshape(-1, 3), rvec, translation,
                                    matrix, distortion)[0].reshape(-1, 16, 2)-origin
        scores = np.zeros(len(poses))
        for index, corners in enumerate(np.round(pixels).astype(np.int32)):
            if (np.any(camera_points[index, :, 2] <= 0) or corners.min() < 0
                    or corners[:, 0].max() >= shape[1] or corners[:, 1].max() >= shape[0]):
                continue
            rendered = np.zeros(shape, np.uint8)
            cv2.fillPoly(rendered, [corners[8:]], 1)
            for i in range(8):
                j = (i+1) % 8
                cv2.fillConvexPoly(rendered, corners[[i, j, j+8, i+8]], 1)
            scores[index] = np.count_nonzero(observed & rendered)/max(
                1, np.count_nonzero(observed | rendered))
        return scores

    if previous is not None:
        seed = np.r_[previous['xy'], previous['yaw']]
        nearby = [seed+[dx, dy, turn]
                  for dx in np.arange(-.004, .0041, .002)
                  for dy in np.arange(-.004, .0041, .002)
                  for turn in np.deg2rad(np.arange(-6, 6.1, 3))]
        nearby_scores = evaluate(nearby)
        if nearby_scores.max() >= .85:
            best = nearby[int(nearby_scores.argmax())]
            fine = [best+[dx, dy, turn]
                    for dx in np.arange(-.001, .0011, .0005)
                    for dy in np.arange(-.001, .0011, .0005)
                    for turn in np.deg2rad(np.arange(-2, 2.1, 1))]
        else:
            previous = None
    if previous is None:
        seed = np.r_[initial['xy'], initial['yaw']]
        coarse = [seed+[dx, dy, turn]
                  for dx in np.arange(-.016, .0161, .004)
                  for dy in np.arange(-.016, .0161, .004)
                  for turn in np.deg2rad(np.arange(-18, 18.1, 6))]
        scores = evaluate(coarse)
        best = coarse[int(scores.argmax())]
        fine = [best+[dx, dy, turn]
                for dx in np.arange(-.002, .0021, .0005)
                for dy in np.arange(-.002, .0021, .0005)
                for turn in np.deg2rad(np.arange(-3, 3.1, 1))]
    scores = evaluate(fine)
    index = int(scores.argmax())
    if scores[index] < .85:
        return None
    best = np.asarray(fine[index])
    # A flat score across different positions cannot establish an unambiguous pose.
    alternatives = np.linalg.norm(np.asarray(fine)[:, :2]-best[:2], axis=1) > .003
    if np.any(alternatives & (scores > scores[index]-.01)):
        return None
    return {**dimensions, 'xy': best[:2].tolist(), 'yaw': float(best[2]),
            'iou': float(scores[index]), 'silhouette_iou': float(scores[index]),
            'observation_method': 'known_height_3d_silhouette',
            'contour_xy': world_outline(dimensions, best[:2], best[2]).tolist()}
