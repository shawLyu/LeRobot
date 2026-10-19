"""Oblique views must fit the known prism, including side walls, in image space."""

import cv2
import numpy as np

from lerobot_arm_control.perception.tabletop_silhouette import fit_extruded_t
from lerobot_arm_control.perception.tabletop_tracking import t_outline

DIMENSIONS = {'width': .048, 'length': .06, 'cap_depth': .012,
              'stem_width': .012, 'height': .006}


def scene(xy=(.03, .02), yaw=.6):
    center = np.array([.1, -.45, .25])
    forward = -center/np.linalg.norm(center)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    rotation = np.vstack((right, np.cross(forward, right), forward))
    translation = -rotation@center
    matrix = np.array([[800., 0, 640], [0, 800., 360], [0, 0, 1]])
    xy = np.array(xy)
    planar_rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                                [np.sin(yaw), np.cos(yaw)]])
    points = t_outline(DIMENSIONS)@planar_rotation.T+xy
    points = np.concatenate((np.column_stack((points, np.zeros(8))),
                              np.column_stack((points, np.full(8, DIMENSIONS['height'])))))
    pixels = cv2.projectPoints(points, cv2.Rodrigues(rotation)[0], translation,
                                matrix, np.zeros(5))[0].reshape(16, 2).round().astype(np.int32)
    mask = np.zeros((720, 1280), np.uint8)
    cv2.fillPoly(mask, [pixels[8:]], 255)
    for i in range(8):
        j = (i+1) % 8
        cv2.fillConvexPoly(mask, pixels[[i, j, j+8, i+8]], 255)
    contour = max(cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    return contour, matrix, rotation, translation, xy, yaw


def test_known_extruded_geometry_recovers_pose_without_resizing_the_t():
    contour, matrix, rotation, translation, xy, yaw = scene()
    result = fit_extruded_t(contour, DIMENSIONS, matrix, np.zeros(5), rotation, translation,
                            {'xy': xy+[.006, -.004], 'yaw': yaw+.1})
    assert result is not None
    np.testing.assert_allclose(result['xy'], xy, atol=.002)
    assert abs(result['yaw']-yaw) < .04
    assert result['silhouette_iou'] > .95
    assert all(result[key] == value for key, value in DIMENSIONS.items())


def test_a_rectangle_cannot_initialize_a_t_observation():
    contour, matrix, rotation, translation, xy, yaw = scene()
    x, y, width, height = cv2.boundingRect(contour)
    rectangle = np.array([[x, y], [x+width, y], [x+width, y+height], [x, y+height]], np.int32)
    assert fit_extruded_t(rectangle, DIMENSIONS, matrix, np.zeros(5), rotation, translation,
                           {'xy': xy, 'yaw': yaw}) is None


def test_tracking_uses_the_new_image_instead_of_republishing_previous_pose():
    xy = np.array([.032, .019])
    contour, matrix, rotation, translation, _, yaw = scene(xy)
    previous = {'xy': [.03, .02], 'yaw': .6}
    result = fit_extruded_t(contour, DIMENSIONS, matrix, np.zeros(5), rotation, translation,
                            {'xy': [.4, .4], 'yaw': 0.}, previous)
    assert result is not None
    np.testing.assert_allclose(result['xy'], xy, atol=.001)
    assert np.linalg.norm(np.subtract(result['xy'], previous['xy'])) > .001
