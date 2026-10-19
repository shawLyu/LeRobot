"""Degenerate noisy marker candidates must not crash a live control session."""

from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from lerobot_arm_control.perception.marker_detection import detect_board, detect_markers


def test_degenerate_candidate_is_missing_and_next_frame_can_recover():
    image = np.zeros((100, 100, 3), np.uint8)
    detector = Mock()
    valid = ([np.zeros((1, 4, 2))], np.array([[3]]), [])
    detector.detectMarkers.side_effect = [
        cv2.error('(-215:Assertion failed) nContours.size() >= 2 in _interpolate2Dline'), valid]
    assert detect_markers(detector, image) == ([], None, [])
    assert detect_markers(detector, image) is valid
    detector.detectBoard.side_effect = cv2.error('nContours.size() >= 2')
    assert detect_board(detector, image) == (None, None, [], None)


def test_other_opencv_errors_are_preserved():
    detector = Mock()
    detector.detectMarkers.side_effect = cv2.error('invalid input image shape')
    with pytest.raises(cv2.error, match='invalid input image shape'):
        detect_markers(detector, np.zeros((1, 1), np.uint8))
