"""Exercise the client/planner boundary and real JSON submission format without SSH/hardware."""

import base64
import io
import json
from unittest.mock import Mock

import pytest

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.transport.push_t_link import MotorLink


def test_motor_proxy_waits_for_acceptance_and_matching_job_completion():
    link = Mock()
    link.send.return_value = 7
    motor = MotorLink(link, 'model-hash')
    reference = {'captured_unix_ns': 1, 'robot': {}, 'objects': {}}
    motor.submit(reference, [{'start': [0]*6, 'end': [.1]*6, 'seconds': 1}])
    assert motor.status()['phase'] == 'pending'
    assert link.send.call_args.kwargs['model_sha256'] == 'model-hash'
    motor.update({'job_id': 6, 'motor': {'phase': 'completed'}})
    assert motor.status()['phase'] == 'pending'
    motor.acknowledge({'id': 7, 'ok': True, 'result': {
        'motor': {'phase': 'pending', 'writes_enabled': False}}})
    assert motor.status()['phase'] == 'pending'
    motor.update({'job_id': 7, 'motor': {'phase': 'completed', 'writes_enabled': True}})
    assert motor.status()['phase'] == 'completed'
    motor.submit(reference, [])
    motor.acknowledge({'id': 7, 'ok': False, 'error': 'scene changed'})
    assert motor.status()['phase'] == 'stopped'
    assert motor.status()['reason'] == 'scene changed'


def test_closing_planner_after_disconnect_does_not_raise_or_send_new_goals():
    link = Mock()
    link.send.side_effect = BrokenPipeError('SSH closed')
    motor = MotorLink(link, 'model-hash')
    motor.cancel('session closed')
    assert motor.status()['phase'] == 'stopped'
    assert 'connection lost' in motor.status()['reason']
    link.send.assert_called_once_with('push_stop', reason='session closed')


def test_stop_after_rejected_job_tracks_the_remote_idle_job_id():
    link = Mock()
    link.send.side_effect = [7, 8]
    motor = MotorLink(link, 'model')
    motor.submit({'captured_unix_ns': 1, 'robot': {}, 'objects': {}}, [])
    motor.acknowledge({'id': 7, 'ok': False, 'error': 'observation stale'})
    motor.cancel('hold')
    motor.acknowledge({'id': 8, 'ok': True, 'result': {
        'job_id': None, 'motor': {'phase': 'stopping', 'writes_enabled': False}}})
    motor.update({'job_id': None, 'motor': {'phase': 'stopped', 'writes_enabled': False,
                                          'reason': 'hold'}})
    assert motor.status()['phase'] == 'stopped'
    assert motor.status()['reason'] == 'hold'


def test_table_link_starts_local_planner_and_sends_results_only(monkeypatch, tmp_path):
    pytest.importorskip('mujoco')
    pytest.importorskip('cv2')
    from lerobot_arm_control.runtime.tabletop_client import TableLink

    marker = base64.b64encode(b'marker').decode()
    hello = {'version': 1, 'type': 'hello', 'tag_png': marker, 'config': {},
             'control_config_sha256': CONTROL_CONFIG_SHA256,
             'planning_location': 'local', 'execution_protocol': 1}
    state = {'version': 1, 'type': 'state', 'sequence': 1, 'jpeg': marker,
             'annotated_jpeg': marker, 'models': {}, 'objects': {}}
    process = Mock(stdin=io.StringIO(), stdout=io.StringIO(
        json.dumps(hello)+'\n'+json.dumps(state)+'\n'))
    monkeypatch.setattr('lerobot_arm_control.runtime.tabletop_client.subprocess.Popen',
        Mock(return_value=process))
    planner = Mock()
    planner.status.return_value = {'phase': 'preview ready', 'planning_location': 'local'}
    factory = Mock(return_value=planner)
    monkeypatch.setattr('lerobot_arm_control.tasks.push_t.push_t_local.LocalPush', factory)
    link = TableLink('robot-host', '/remote/robot', tmp_path, server_args=('--push-t',))
    link.reader.join(timeout=3)
    assert link.error is None
    assert factory.call_args.kwargs['execute'] is False
    assert planner.observe.call_count == 1
    assert link.latest[0]['push_t']['planning_location'] == 'local'
    link.send('push_start')
    planner.command.assert_called_with('push_start')
    assert not process.stdin.getvalue()
    factory.call_args.kwargs['publish']({'passed': True})
    command = json.loads(process.stdin.getvalue())
    assert command['op'] == 'push_result'
    assert command['version'] == 1
    assert command['control_config_sha256'] == CONTROL_CONFIG_SHA256
    assert command['result']['passed'] is True


@pytest.mark.parametrize('condition', ['ready', 'missing_identity', 'feedback_error', 'read_only'])
def test_hardware_handshake_requires_verified_identity_and_an_available_executor(
        monkeypatch, tmp_path, condition):
    pytest.importorskip('mujoco')
    pytest.importorskip('cv2')
    import hashlib

    from lerobot_arm_control.runtime.tabletop_client import ROOT, TableLink

    model_hash = hashlib.sha256((ROOT/'assets/so101/so101.xml').read_bytes()).hexdigest()
    robot = {'fingerprint': {'mjcf_sha256': model_hash}, 'motion_available': True,
             'status': 'live'}
    if condition == 'missing_identity':
        robot = {'status': 'error', 'error': 'There is no status packet!'}
    elif condition == 'feedback_error':
        robot.update(status='error', error='Encoder read failed')
    elif condition == 'read_only':
        robot['motion_available'] = False
    marker = base64.b64encode(b'marker').decode()
    hello = {'version': 1, 'type': 'hello', 'tag_png': marker, 'config': {},
             'control_config_sha256': CONTROL_CONFIG_SHA256, 'robot': robot,
             'planning_location': 'local', 'execution_protocol': 1}
    state = {'version': 1, 'type': 'state', 'sequence': 1, 'jpeg': marker,
             'annotated_jpeg': marker, 'models': {}, 'objects': {}}
    process = Mock(stdin=io.StringIO(), stdout=io.StringIO(
        json.dumps(hello)+'\n'+json.dumps(state)+'\n'))
    process.poll.return_value = 0
    monkeypatch.setattr('lerobot_arm_control.runtime.tabletop_client.subprocess.Popen',
        Mock(return_value=process))
    planner = Mock()
    planner.status.return_value = {'phase': 'waiting for observation'}
    factory = Mock(return_value=planner)
    monkeypatch.setattr('lerobot_arm_control.tasks.push_t.push_t_local.LocalPush', factory)
    link = TableLink('unused', '/unused', tmp_path,
                     server_args=('--push-t', '--hardware-control'))
    try:
        link.reader.join(timeout=3)
        assert not link.reader.is_alive()
        if condition == 'ready':
            assert link.error is None
            assert link.motor.model_hash == model_hash
            assert factory.call_args.kwargs['execute'] is True
            assert planner.observe.call_count == 1
        else:
            assert 'not ready for hardware control' in link.error
            if robot.get('error'):
                assert robot['error'] in link.error
            factory.assert_not_called()
            assert link.motor is None
            assert link.planner is None
            assert link.count == 0
        assert not process.stdin.getvalue()
    finally:
        link.close()


def test_local_texture_worker_does_not_block_feedback_and_delivers_textures_once(monkeypatch):
    pytest.importorskip('mujoco')
    pytest.importorskip('cv2')
    import threading
    import time

    from lerobot_arm_control.visualization.appearance_worker import AppearanceWorker

    release, began = threading.Event(), threading.Event()

    class Extractor:
        def __init__(self, config):
            pass

        def extract(self, frame, state, native):
            began.set()
            release.wait(2)
            return {'metadata': {'version': 1}, 'pngs': {'tabletop.png': 'texture'}}

    monkeypatch.setattr('lerobot_arm_control.visualization.appearance_worker.ObservedAppearance',
        Extractor)
    worker = AppearanceWorker({})
    try:
        worker.offer({'jpeg': base64.b64encode(b'frame').decode()})
        assert began.wait(1)
        started = time.monotonic()
        for _ in range(50):
            worker.offer({'jpeg': base64.b64encode(b'frame').decode()})
            assert worker.take() == {}
        assert time.monotonic()-started < .1
        release.set()
        worker.worker.join(timeout=1)
        assert worker.take()['appearance_pngs'] == {'tabletop.png': 'texture'}
        assert worker.take() == {'appearance': {'version': 1}}
    finally:
        release.set()
        worker.close()


def test_push_stream_rate_is_independent_of_display_and_omits_bulk_native_images(
        tmp_path, monkeypatch):
    from types import SimpleNamespace

    from lerobot_arm_control.runtime.tabletop_client import run

    captured = []

    def link(*args, server_args):
        captured.extend(server_args)
        raise RuntimeError('stop before connecting')

    monkeypatch.setattr('lerobot_arm_control.runtime.tabletop_client.TableLink', link)
    args = SimpleNamespace(output=tmp_path, push_t=True, fps=5, hardware_control=True,
                           recalibrate_base=False, recalibrate_table=False, camera_only=False,
                           host='unused', remote_root='/unused')
    with pytest.raises(RuntimeError, match='stop before connecting'):
        run(args)
    assert float(captured[captured.index('--stream-fps')+1]) >= 10
    assert '--omit-native-images' in captured and '--hardware-control' in captured
