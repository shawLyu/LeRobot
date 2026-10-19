"""Independent camera observations in the primary table frame; no motor access."""

import copy
import json
import multiprocessing
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.perception.camera_controls import apply_v4l2_controls
from lerobot_arm_control.perception.tabletop_perception import Perception, read_config


def current_object(state, name, age):
    obj = state.get('objects', {}).get(name, {})
    return (0 <= age <= .5 and state.get('marker_visible') and state.get('units') == 'm'
            and obj.get('observed') is True and obj.get('pose_valid') is True
            and obj.get('tracking_status') in ('observed', 'partial')
            and np.isfinite([*obj.get('xy', [np.inf]*2), obj.get('yaw', np.inf)]).all())


def fuse_objects(primary, views):
    """Use a fresh calibrated second view during occlusion; disagreement holds motion."""
    reference = primary.get('table_reference')
    references = [('primary', primary)]
    references.extend((name, view) for name, view in views.items()
                      if reference is not None and view.get('table_reference') == reference
                      and view.get('calibrated') is True)
    localized = [(name, view) for name, view in references
                 if view.get('marker_visible') and view.get('units') == 'm'
                 and type(view.get('captured_unix_ns')) is int
                 and 0 <= view.get('capture_age_at_send_ms', np.inf) <= 500]
    primary.pop('control_table_observation', None)
    if localized:
        name, view = min(localized, key=lambda item: item[1]['capture_age_at_send_ms'])
        primary['control_table_observation'] = {
            'source': name, 'captured_unix_ns': view['captured_unix_ns'],
            'capture_age_at_send_ms': view['capture_age_at_send_ms'], 'marker_visible': True}
    primary['observation_sources'] = {}
    primary['camera_disagreements'] = {}
    for name in ('wood_t', 'black_t'):
        a = primary.get('objects', {}).get(name, {})
        source = a.get('observation_source')
        primary_source = source in (None, 'primary', 'primary_previous_frame')
        own = primary_source and current_object(
            primary, name, primary.get('capture_age_at_send_ms', np.inf)/1000)
        stamp = a.get('observation_captured_unix_ns')
        if type(stamp) is int:
            age = max(0., (primary['captured_unix_ns']-stamp)/1e9+primary.get(
                'capture_age_at_send_ms', np.inf)/1000)
            a['observation_age_s'] = age
            if age > .5 and a.get('tracking_status') != 'static_cached':
                a.update(observed=False, tracking_status='held')
                own = False
        if (not own and primary_source and type(stamp) is int and a.get('pose_valid')
                and not (name == 'black_t' and a.get('tracking_status') == 'static_cached')
                and a.get('tracking_status') in ('held', 'static_cached', 'recent')):
            age = (primary['captured_unix_ns']-stamp)/1e9+primary.get(
                'capture_age_at_send_ms', np.inf)/1000
            if 0 <= age <= .5:
                a = {**a, 'tracking_status': 'recent', 'observed': False,
                     'observation_source': 'primary_previous_frame', 'observation_age_s': age}
                primary['objects'][name] = a
                own = True
        candidates = []
        for camera, view in views.items():
            if reference is not None and view.get('table_reference') != reference:
                continue
            age = view.get('capture_age_at_send_ms', np.inf)/1000
            if view.get('calibrated') and current_object(view, name, age):
                candidates.append((camera, view, age))
        primary_measurement = own
        if name == 'black_t' and own and a.get('tracking_status') == 'partial':
            complete = [item for item in candidates
                        if item[1]['objects'][name]['tracking_status'] == 'observed']
            if complete:
                # Dark arm geometry can pull a partially visible paper-template fit.
                # A current complete silhouette is stronger evidence of its pose.
                own, candidates = False, complete
        if own:
            primary['observation_sources'][name] = a.get('observation_source', 'primary')
            a = primary['objects'][name]
            for camera, view, _ in candidates:
                b = view['objects'][name]
                # Do not compare different instants as if they were simultaneous.
                if abs(view['captured_unix_ns']-a.get(
                        'observation_captured_unix_ns', primary['captured_unix_ns'])) > 150_000_000:
                    continue
                turn = np.arctan2(np.sin(a['yaw']-b['yaw']), np.cos(a['yaw']-b['yaw']))
                if np.linalg.norm(np.subtract(a['xy'], b['xy'])) > .005 or abs(turn) > .15:
                    if view.get('intrinsics_quality') == 'provisional':
                        primary['camera_disagreements'][name] = {
                            'rejected_source': camera, 'reason': 'provisional intrinsics disagree',
                            'distance_m': float(np.linalg.norm(np.subtract(a['xy'], b['xy']))),
                            'yaw_rad': float(abs(turn))}
                        continue
                    primary['objects'][name] = {**a, 'pose_valid': False, 'observed': False,
                                                'tracking_status': 'camera_disagreement'}
                    primary['observation_sources'][name] = 'camera_disagreement'
                    break
        elif candidates:
            camera, view, age = min(candidates, key=lambda item: item[2])
            obj = copy.deepcopy(view['objects'][name])
            obj.update(observation_source=camera,
                       observation_captured_unix_ns=view['captured_unix_ns'],
                       observation_age_s=age+obj.get('observation_age_s', 0.))
            primary['objects'][name] = obj
            primary['observation_sources'][name] = camera
        # Movement checks compare a camera to its own initial measurement, rather
        # than treating the fixed bias between two cameras as object motion.
        measurements = {}
        cached_reference = (name == 'black_t' and primary_source
                            and a.get('tracking_status') == 'static_cached'
                            and a.get('pose_valid'))
        if primary_measurement or cached_reference:
            measurements['primary'] = {'xy': list(a['xy']), 'yaw': a['yaw'],
                'observed': a.get('observed', False),
                'tracking_status': a.get('tracking_status'),
                'captured_unix_ns': a.get('observation_captured_unix_ns',
                                         primary['captured_unix_ns'])}
            if cached_reference:
                measurements['primary']['static_cached'] = True
        rejected = primary['camera_disagreements'].get(name, {}).get('rejected_source')
        for camera, view, _ in candidates:
            if camera != rejected:
                obj = view['objects'][name]
                measurements[camera] = {'xy': list(obj['xy']), 'yaw': obj['yaw'],
                                        'observed': obj['observed'],
                                        'tracking_status': obj['tracking_status'],
                                        'captured_unix_ns': view['captured_unix_ns']}
        if name in primary.get('objects', {}):
            primary['objects'][name]['camera_observations'] = measurements
    return primary


def refresh_observation(primary, elapsed_ms, views, robot=None):
    """A newly arrived side view can refresh control without resetting the primary frame age."""
    state = copy.deepcopy(primary)
    state['capture_age_at_send_ms'] += max(0., elapsed_ms)
    if robot is not None:
        state['robot'] = {**robot, 'registration': state['robot']['registration']}
    elif 'sample_age_at_send_ms' in state.get('robot', {}):
        state['robot']['sample_age_at_send_ms'] += max(0., elapsed_ms)
    state['camera_views'] = views
    return fuse_objects(state, views)


class CameraWorker:
    """Capture and perception have separate latest-frame slots, never a growing queue."""
    def __init__(self, entry):
        self.name = entry['name']
        if not self.name.replace('_', '').replace('-', '').isalnum():
            raise ValueError('observer name must contain only letters, numbers, _ or -')
        self.config = read_config(workspace_root()/entry['tabletop_config'])
        source = workspace_root()/self.config['camera_calibration']['file']
        record = json.loads(source.read_text())
        passed = record.get('validation', {}).get('passed') is True
        provisional = (record.get('quality') == 'provisional'
                       and entry.get('allow_provisional') is True)
        if not record.get('activated') or not (passed or provisional):
            raise ValueError('observer requires accepted intrinsics or explicit provisional use')
        self.quality = 'validated' if passed else 'provisional'
        if (self.config.get('robot', {}).get('enabled')
                or self.config.get('additional_robots')):
            raise ValueError('observer cameras cannot own robot base calibration')
        self.perception = Perception(self.config)
        self.device = entry['device']
        if record.get('camera_device') != self.device:
            raise ValueError('observer device differs from the accepted intrinsics record')
        self.stopped = threading.Event()
        self.changed = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.layouts = None
        self.latest = {'status': 'connecting', 'calibrated': True, 'device': self.device,
                       'intrinsics_quality': self.quality}
        self.published_at = 0.
        path = Path(self.device).resolve()
        index = int(path.name[5:]) if path.name.startswith('video') else self.device
        self.camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.camera.release()
            raise RuntimeError(f'{self.name}: cannot open camera')
        size = self.config.get('capture_size', self.config['image_size'])
        for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                            (cv2.CAP_PROP_FRAME_WIDTH, size[0]),
                            (cv2.CAP_PROP_FRAME_HEIGHT, size[1]),
                            (cv2.CAP_PROP_FPS, entry.get('fps', 15)),
                            (cv2.CAP_PROP_BUFFERSIZE, 1)):
            self.camera.set(prop, value)
        try:
            self.controls = apply_v4l2_controls(
                self.device, entry.get('exposure_controls', record.get(
                    'camera_controls', {}).get('v4l2_controls', {})))
        except Exception:
            self.camera.release()
            raise
        self.capture = threading.Thread(target=self._capture, daemon=True)
        self.worker = threading.Thread(target=self._process, daemon=True)
        self.capture.start()
        self.worker.start()

    def _capture(self):
        try:
            sequence = 0
            while not self.stopped.is_set():
                ok, frame = self.camera.read()
                if not ok:
                    raise RuntimeError('camera stopped returning frames')
                sequence += 1
                with self.lock:
                    self.frame = frame, sequence, time.time_ns(), time.monotonic()
                self.changed.set()
        except Exception as exc:
            self._failed(exc)

    def _failed(self, error):
        with self.lock:
            self.latest = {'status': 'error', 'calibrated': False, 'error': str(error),
                           'device': self.device}
        self.stopped.set()
        self.changed.set()

    def _process(self):
        sequence = 0
        try:
            while not self.stopped.is_set():
                self.changed.wait(.1)
                self.changed.clear()
                with self.lock:
                    packet = self.frame
                    layouts, self.layouts = self.layouts, None
                if packet is None or packet[1] == sequence:
                    continue
                native, sequence, captured_ns, captured_at = packet
                if list(native.shape[1::-1]) != self.config['capture_size']:
                    raise ValueError('observer capture resolution differs from its calibration')
                if layouts is not None:
                    self.perception.secondary_layouts.update(layouts)
                mapping = np.array(self.config['low_to_capture_affine'])
                frame = cv2.warpAffine(native, cv2.invertAffineTransform(mapping),
                                       tuple(self.config['image_size']), flags=cv2.INTER_AREA)
                state = self.perception.process(frame, sequence, captured_ns, native, mapping)
                now = time.monotonic()
                state.update(status='live', calibrated=True, device=self.device,
                             intrinsics_quality=self.quality,
                             exposure_readback=self.controls,
                             capture_age_at_send_ms=(now-captured_at)*1000)
                with self.lock:
                    self.latest, self.published_at = state, now
        except Exception as exc:
            self._failed(exc)

    def snapshot(self, layouts):
        with self.lock:
            self.layouts = copy.deepcopy(layouts)
            state, published = dict(self.latest), self.published_at
        if state.get('status') == 'live':
            state['capture_age_at_send_ms'] += (time.monotonic()-published)*1000
            if state['capture_age_at_send_ms'] > 500:
                state['status'] = 'stale'
        return state

    def close(self):
        self.stopped.set()
        self.changed.set()
        self.capture.join(timeout=2)
        self.camera.release()
        self.worker.join(timeout=2)


def _observe_camera(entry, states, layouts, stopped):
    """Each camera gets a Python process so silhouette searches cannot block another view."""
    worker, current_layouts = None, {}
    try:
        cv2.setNumThreads(entry.get('opencv_threads', 2))
        worker = CameraWorker(entry)
        sequence = None
        while not stopped.is_set():
            try:
                current_layouts = layouts.get_nowait()
            except queue.Empty:
                pass
            state = worker.snapshot(current_layouts)
            key = state.get('sequence'), state.get('status')
            if key != sequence:
                try:
                    states.put((state, time.monotonic()), timeout=.1)
                    sequence = key
                except queue.Full:
                    pass
            if state.get('status') == 'error':
                break
            stopped.wait(.01)
    except Exception as exc:
        states.put(({'status': 'error', 'calibrated': False, 'error': str(exc)},
                    time.monotonic()), timeout=1)
    finally:
        if worker:
            worker.close()


class CameraObserver:
    """Read the independent process continuously, retaining only its latest observation."""
    def __init__(self, entry):
        self.name, self.device = entry['name'], entry['device']
        self.config = read_config(workspace_root()/entry['tabletop_config'])
        self.latest = {'status': 'connecting', 'calibrated': False, 'device': self.device}
        self.published_at = 0.
        self.lock = threading.Lock()
        self.last_layouts = None
        self.on_update = None
        context = multiprocessing.get_context('spawn')
        self.states, self.layouts = context.Queue(maxsize=2), context.Queue(maxsize=1)
        self.stopped = context.Event()
        self.process = context.Process(target=_observe_camera,
            args=(entry, self.states, self.layouts, self.stopped), daemon=True)
        self.process.start()
        self.reader = threading.Thread(target=self._receive, daemon=True)
        self.reader.start()

    def _receive(self):
        while not self.stopped.is_set():
            try:
                state, published = self.states.get(timeout=.1)
            except queue.Empty:
                if not self.process.is_alive():
                    with self.lock:
                        if self.latest.get('status') != 'error':
                            self.latest = {'status': 'error', 'calibrated': False,
                                           'error': 'observer process exited'}
                    break
                continue
            with self.lock:
                self.latest, self.published_at = state, published
            if self.on_update:
                self.on_update()

    def snapshot(self, layouts):
        if layouts != self.last_layouts:
            try:
                self.layouts.put_nowait(copy.deepcopy(layouts))
                self.last_layouts = copy.deepcopy(layouts)
            except queue.Full:
                pass
        with self.lock:
            state, published = dict(self.latest), self.published_at
        if 'capture_age_at_send_ms' in state:
            state['capture_age_at_send_ms'] += (time.monotonic()-published)*1000
            if state.get('status') == 'live' and state['capture_age_at_send_ms'] > 500:
                state['status'] = 'stale'
        return state

    def close(self):
        self.stopped.set()
        self.reader.join(timeout=1)
        self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1)
        for channel in (self.states, self.layouts):
            channel.cancel_join_thread()
            channel.close()
