"""Stationary acquisition and browser commands must preserve raw calibration evidence."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import cv2
import numpy as np
import pytest

from lerobot_arm_control.calibration.camera_live import (
    CalibrationServer,
    LiveCapture,
    PoseSelection,
)
from lerobot_arm_control.perception.camera_controls import apply_v4l2_controls


def points(offset=0, count=24):
    return {i: np.array([i*8.+offset, (i % 4)*20.]) for i in range(count)}


def test_requires_stationarity_and_different_pose():
    selector = PoseSelection()
    assert not selector.update(points(), 0)[0]
    assert not selector.update(points(), .1)[0]
    assert selector.update(points(), .8)[0]
    selector.accepted = points()
    assert not selector.update(points(.3), 1)[0]
    assert not selector.update(points(30), 1.1)[0]
    assert not selector.update(points(30), 1.2)[0]
    assert selector.update(points(30), 1.9)[0]


def test_incomplete_board_and_motion_reset_hold():
    selector = PoseSelection()
    selector.update(points(), 0)
    selector.update(points(), .1)
    assert selector.update(points(), .8)[0]
    assert not selector.update(points(count=10), 1)[0]
    assert not selector.update(points(), 2)[0]
    assert not selector.update(points(), 2.1)[0]
    assert not selector.update(points(10), 2.8)[0]
    assert selector.still_since is None


def test_exposure_controls_require_actual_matching_readback(monkeypatch):
    calls = []

    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='auto_exposure: 1\nexposure_time_absolute: 80\ngain: 32\n')

    monkeypatch.setattr('lerobot_arm_control.perception.camera_controls.subprocess.run', run)
    controls = {'auto_exposure': 1, 'exposure_time_absolute': 80, 'gain': 32}
    assert 'exposure_time_absolute: 80' in apply_v4l2_controls('/dev/video2', controls)
    assert calls[0][-2] == '--set-ctrl' and calls[1][-2] == '--get-ctrl'
    with pytest.raises(ValueError, match='readback differs'):
        apply_v4l2_controls('/dev/video2', {**controls, 'gain': 31})
    with pytest.raises(ValueError, match='Unsupported'):
        apply_v4l2_controls('/dev/video2', {'focus': 'not an exposure setting'})


def test_capture_publishes_annotated_preview_and_saves_one_stationary_native_view(
        tmp_path, monkeypatch):
    directory = tmp_path/'capture'
    directory.mkdir()
    (directory/'calibration.json').write_text(json.dumps({
        'device': 'synthetic', 'image_size': [1280, 720], 'measured_square_mm': 30,
        'camera_controls': {'verified': True}}))
    board = cv2.aruco.CharucoBoard((5, 7), .03, .022,
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
    image = np.full((720, 1280, 3), 255, np.uint8)
    image[150:570, 490:790] = board.generateImage((300, 420))[:, :, None]
    camera = Mock()
    camera.isOpened.return_value = True
    camera.read.side_effect = [(True, image)]*12+[(False, None)]
    module = 'lerobot_arm_control.calibration.camera_live.'
    monkeypatch.setattr(module+'cv2.VideoCapture', Mock(return_value=camera))
    monkeypatch.setattr(module+'threading.Thread', Mock())
    clock = iter(np.arange(0, 5., .1))
    monkeypatch.setattr(module+'time.monotonic', lambda: float(next(clock)))
    capture = LiveCapture(directory, 'synthetic', 30, False, tmp_path/'candidate.json')
    try:
        capture.command('start')
        capture.run()
        snapshot = capture.snapshot()
        assert snapshot['count'] == 1 and snapshot['corners'] == 24 and snapshot['image']
        saved = [json.loads(line) for line in (directory/'states.jsonl').read_text().splitlines()]
        assert len(saved) == 1
        native = cv2.imread(str(directory/saved[0]['native_frame']))
        assert native.shape == image.shape
        assert saved[0]['hold_seconds'] >= .6
    finally:
        capture.close()
    camera.release.assert_called_once()


def test_http_page_state_and_capture_commands_require_same_session():
    class Capture:
        action = None

        def snapshot(self):
            return {'count': 16, 'paused': True}

        def command(self, action):
            self.action = action

    capture = Capture()
    server = CalibrationServer(('127.0.0.1', 0), capture)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    address = f'http://127.0.0.1:{server.server_port}'
    urlopen = build_opener(ProxyHandler({})).open
    try:
        with urlopen(address, timeout=2) as response:
            page = response.read().decode()
            assert '已采集覆盖图' in page and '__TOKEN__' not in page
            assert server.token in page
        with pytest.raises(HTTPError) as error:
            urlopen(address+'/api/state', timeout=2)
        assert error.value.code == 403
        headers = {'X-Calibration-Token': server.token, 'Content-Type': 'application/json'}
        with urlopen(Request(address+'/api/state', headers=headers), timeout=2) as response:
            assert json.load(response)['count'] == 16
        with urlopen(Request(address+'/api/command', headers=headers,
                             data=b'{"action":"start"}'), timeout=2):
            assert capture.action == 'start'
        headers['Origin'] = 'https://untrusted.example'
        with pytest.raises(HTTPError) as error:
            urlopen(Request(address+'/api/command', headers=headers,
                            data=b'{"action":"start"}'), timeout=2)
        assert error.value.code == 403
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)
