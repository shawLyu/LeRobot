"""Loopback browser guidance for fixed-focus ChArUco acquisition; no motor access."""

import json
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from lerobot_arm_control.perception.camera_controls import apply_v4l2_controls

from .camera_intrinsics import fit_capture


def displacement(a, b):
    common = set(a) & set(b)
    return float(np.median([np.linalg.norm(a[i]-b[i]) for i in common])) \
        if len(common) >= 12 else float('inf')


class PoseSelection:
    """Accept stationary, distinct views using the same distance as the offline fitter."""
    def __init__(self):
        self.previous = {}
        self.accepted = {}
        self.still_since = None

    def update(self, corners, now):
        motion = displacement(corners, self.previous)
        self.previous = corners
        if len(corners) < 16 or motion >= 2:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        held = 0. if self.still_since is None else now-self.still_since
        distinct = displacement(corners, self.accepted) >= 25
        return held >= .6 and distinct, held, distinct


class LiveCapture:
    def __init__(self, directory, device, square_mm, fixed_focus, output):
        self.directory, self.output = Path(directory), Path(output)
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory/'native').mkdir(exist_ok=True)
        metadata_path = self.directory/'calibration.json'
        if metadata_path.exists():
            self.metadata = json.loads(metadata_path.read_text())
            if (self.metadata['device'] != device
                    or self.metadata['measured_square_mm'] != square_mm):
                raise ValueError('Device or square size differs from saved session')
        else:
            self.metadata = {'version': 1, 'device': device, 'image_size': [1280, 720],
                             'measured_square_mm': square_mm, 'camera_controls': {
                                 'verified': fixed_focus, 'focus_mode': 'manual_fixed',
                                 'v4l2_controls': {'auto_exposure': 1,
                                                  'exposure_time_absolute': 80, 'gain': 32}}}
        if not self.metadata['camera_controls'].get('verified'):
            raise ValueError('Confirm fixed focus before collecting calibration images')
        controls = self.metadata['camera_controls'].get('v4l2_controls', {})
        self.board = cv2.aruco.CharucoBoard(
            (5, 7), square_mm/1000, square_mm/1000*22/30,
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.selection = PoseSelection()
        self.centers, self.count = [], 0
        states = self.directory/'states.jsonl'
        self.sequence = 0
        for line in states.read_text().splitlines() if states.exists() else []:
            state = json.loads(line)
            self.sequence = max(self.sequence, state['sequence'])
            image = cv2.imread(str(self.directory/state['native_frame']))
            if image is None:
                raise ValueError('Saved calibration image is missing')
            corners, ids, _, _ = self.detector.detectBoard(image)
            if ids is not None and len(ids) >= 16:
                self.selection.accepted = self.points(corners, ids)
                self.centers.append((corners[:, 0].mean(0)/image.shape[1::-1]).tolist())
                self.count += 1
        self.device, self.square_mm = device, square_mm
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.state = {'paused': True, 'fitting': False, 'corners': 0, 'hold': 0.,
                      'message': '点击开始采集；每个姿态停稳 1–2 秒', 'validation': None}
        path = Path(device).resolve()
        index = int(path.name[5:]) if path.name.startswith('video') else device
        self.camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.camera.release()
            raise RuntimeError('Cannot open calibration camera')
        width, height = self.metadata['image_size']
        for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                            (cv2.CAP_PROP_FRAME_WIDTH, width),
                            (cv2.CAP_PROP_FRAME_HEIGHT, height),
                            (cv2.CAP_PROP_FPS, 15), (cv2.CAP_PROP_BUFFERSIZE, 1)):
            self.camera.set(prop, value)
        try:
            if controls:
                self.metadata['camera_controls']['exposure_readback'] = apply_v4l2_controls(
                    device, controls)
            metadata_path.write_text(json.dumps(self.metadata, indent=2)+'\n')
        except Exception:
            self.camera.release()
            raise
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    @staticmethod
    def points(corners, ids):
        return {int(i): p for i, p in zip(ids[:, 0], corners[:, 0], strict=True)}

    def snapshot(self):
        with self.lock:
            return {**self.state, 'count': self.count, 'centers': list(self.centers),
                    'directory': str(self.directory), 'device': self.device,
                    'frame_age_ms': (time.monotonic()-self.frame_at)*1000
                    if hasattr(self, 'frame_at') else None}

    def command(self, action):
        with self.lock:
            if self.state['fitting']:
                raise ValueError('正在验证内参，请稍候')
            if action in ('start', 'pause'):
                self.state['paused'] = action == 'pause'
                self.selection.still_since = None
            elif action == 'fit':
                self.state.update(paused=True, fitting=True)
                threading.Thread(target=self.fit, daemon=True).start()
            else:
                raise ValueError('Unknown calibration action')

    def fit(self):
        try:
            result = fit_capture(self.directory, self.square_mm)
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            with self.lock:
                self.state['validation'] = result['validation']
                self.state['fit_error'] = None
        except Exception as exc:
            with self.lock:
                self.state['fit_error'] = str(exc)
        finally:
            with self.lock:
                self.state['fitting'] = False

    def run(self):
        try:
            import base64
            while not self.stopped.is_set():
                ok, image = self.camera.read()
                captured_ns, now = time.time_ns(), time.monotonic()
                if not ok:
                    raise RuntimeError('Camera stopped returning frames')
                if list(image.shape[1::-1]) != self.metadata['image_size']:
                    raise ValueError('Camera resolution differs from saved calibration')
                corners, ids, _, _ = self.detector.detectBoard(image)
                points = {} if ids is None else self.points(corners, ids)
                with self.lock:
                    ready, held, distinct = self.selection.update(points, now)
                    message = ('移动到新位置，或向另一个方向倾斜' if not distinct else
                               '请停稳 1–2 秒' if len(points) >= 16 else
                               '请让整块棋盘进入画面，避免遮挡和反光')
                    if ready and not self.state['paused']:
                        self.sequence += 1
                        name = f'native/{self.sequence:06d}.jpg'
                        if not cv2.imwrite(str(self.directory/name), image,
                                           [cv2.IMWRITE_JPEG_QUALITY, 98]):
                            raise OSError('Could not save calibration image')
                        with (self.directory/'states.jsonl').open('a') as stream:
                            stream.write(json.dumps({'sequence': self.sequence,
                                'captured_unix_ns': captured_ns, 'native_frame': name,
                                'charuco_corners': len(points), 'hold_seconds': held})+'\n')
                        self.selection.accepted = points
                        self.centers.append((corners[:, 0].mean(0)/image.shape[1::-1]).tolist())
                        self.count += 1
                        self.state.update(validation=None, fit_error=None)
                        message = '已保存！请换一个位置或倾角'
                    self.state.update(corners=len(points), hold=min(1., held/.6), message=message)
                annotated = image.copy()
                if ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(
                        annotated, corners, cornerColor=(60, 220, 80))
                preview = cv2.resize(annotated, (960, 540))
                ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if not ok:
                    raise RuntimeError('Could not encode camera preview')
                with self.lock:
                    self.state['image'] = base64.b64encode(encoded).decode()
                    self.frame_at = now
        except Exception as exc:
            with self.lock:
                self.state.update(error=str(exc), paused=True)

    def close(self):
        self.stopped.set()
        self.worker.join(timeout=2)
        self.camera.release()


class CalibrationServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, capture):
        self.capture, self.token = capture, secrets.token_urlsafe(32)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def respond(self, status, body, content_type='application/json; charset=utf-8'):
        if not isinstance(body, bytes):
            body = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def check(self, token=False):
        hosts = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
        if self.headers.get('Host') not in hosts:
            raise PermissionError('Loopback access only')
        origin = self.headers.get('Origin')
        if origin and origin not in {f'http://{host}' for host in hosts}:
            raise PermissionError('Cross-origin request rejected')
        if token and not secrets.compare_digest(
                self.headers.get('X-Calibration-Token', ''), self.server.token):
            raise PermissionError('Invalid calibration session')

    def do_GET(self):
        try:
            self.check()
            if self.path == '/':
                html = Path(__file__).with_name('camera_live.html').read_text()
                return self.respond(200, html.replace('__TOKEN__', self.server.token).encode(),
                                    'text/html; charset=utf-8')
            if self.path == '/api/state':
                self.check(token=True)
                return self.respond(200, self.server.capture.snapshot())
            if self.path == '/favicon.ico':
                return self.respond(204, b'')
            self.respond(404, {'error': 'Not found'})
        except PermissionError as exc:
            self.respond(403, {'error': str(exc)})

    def do_POST(self):
        try:
            self.check(token=True)
            if self.path != '/api/command':
                return self.respond(404, {'error': 'Not found'})
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 1024:
                raise ValueError('Invalid request size')
            body = json.loads(self.rfile.read(length))
            self.server.capture.command(body['action'])
            self.respond(200, {'ok': True})
        except PermissionError as exc:
            self.respond(403, {'error': str(exc)})
        except (ValueError, TypeError, KeyError) as exc:
            self.respond(400, {'error': str(exc)})


def serve(args):
    capture = LiveCapture(args.capture, args.device, args.square_mm, args.fixed_focus, args.output)
    server = CalibrationServer(('127.0.0.1', args.port), capture)
    print(f'Camera calibration: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        capture.close()
