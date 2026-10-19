"""Treat OpenCV's degenerate-contour assertion as a missed detection, never a pose."""

import cv2


def _detect(operation, image, missing):
    try:
        return operation(image)
    except cv2.error as exc:
        # OpenCV AprilTag corner refinement can assert on an unusable candidate
        # in noisy/dark frames. Preserve all other errors as real failures.
        if 'nContours.size() >= 2' not in str(exc):
            raise
        return missing


def detect_markers(detector, image):
    return _detect(detector.detectMarkers, image, ([], None, []))


def detect_board(detector, image):
    return _detect(detector.detectBoard, image, (None, None, [], None))
