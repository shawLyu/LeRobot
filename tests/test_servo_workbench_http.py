"""HTTP request validation, exports and the actual offline browser assets."""

import http.client
import json
import threading
from types import SimpleNamespace

import pytest

from lerobot_arm_control.workbench.server import WorkbenchServer, export_csv


class Link:
    def __init__(self):
        self.calls = []

    def state(self):
        return {'connected': False, 'state': None}

    def connect(self, *args):
        self.calls.append(('connect', args))
        return {'arm': args[0]}

    def request(self, operation, payload=None):
        self.calls.append((operation, payload))
        return {'completed': True}

    def close(self):
        self.calls.append(('close', None))


@pytest.fixture
def server():
    arm = SimpleNamespace(name='follower', role='follower', port='/dev/serial/by-id/test',
                          motors={str(i): i for i in range(1, 7)})
    app = WorkbenchServer(('127.0.0.1', 0), Link(), {'follower': arm})
    thread = threading.Thread(target=app.serve_forever, daemon=True)
    thread.start()
    yield app
    app.shutdown()
    app.server_close()
    thread.join(timeout=2)


def request(server, path, body=None, *, token=True, origin=None, host=None, content_type=None):
    connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
    headers = {}
    if token:
        headers['X-Workbench-Token'] = server.token
    if origin:
        headers['Origin'] = origin
    if host:
        headers['Host'] = host
    if body is not None:
        headers['Content-Type'] = content_type or 'application/json'
    connection.request('GET' if body is None else 'POST', path,
                       None if body is None else json.dumps(body), headers)
    result = connection.getresponse()
    response = result.status, result.read(), dict(result.getheaders())
    connection.close()
    return response


def test_browser_assets_are_self_contained_and_session_token_is_inserted(server):
    status, body, headers = request(server, '/')
    assert status == 200 and server.token.encode() in body and b'__TOKEN__' not in body
    assert 'frame-ancestors' in headers['Content-Security-Policy']
    assert b'cdn.' not in body
    for path in ('/app.js', '/style.css'):
        status, body, _ = request(server, path)
        assert status == 200 and len(body) > 1000
    assert request(server, '/api/info')[0] == 200
    assert len(json.loads(request(server, '/api/info')[1])['result']['schema']) > 50


@pytest.mark.parametrize('token,origin,host', [
    (False, None, None), (True, 'https://example.com', None),
    (True, None, 'attacker.example'),
])
def test_cross_site_or_unauthenticated_calls_cannot_drive_motors(server, token, origin, host):
    status, _, _ = request(server, '/api/command', {'operation': 'enable'},
                           token=token, origin=origin, host=host)
    assert status == 403 and not server.link.calls


@pytest.mark.parametrize('body', [[], {'operation': 'shell'},
                                   {'operation': 'enable', 'payload': []}])
def test_unknown_operations_and_invalid_payloads_never_reach_the_worker(server, body):
    assert request(server, '/api/command', body)[0] == 400
    assert not server.link.calls


def test_known_command_is_forwarded_with_exact_payload(server):
    payload = {'targets': {'shoulder_pan': 2070}, 'speed_deg_s': 30}
    assert request(server, '/api/command', {'operation': 'move', 'payload': payload})[0] == 200
    assert server.link.calls == [('move', payload)]


def test_csv_contains_all_six_channels_and_raw_response_time_without_browser_timestamps():
    joints = {f'j{i}': {'position': 2060+i, 'goal': 2070, 'velocity': 4, 'load': 6,
                        'current_raw': 3, 'voltage': 5.3, 'temperature': 25,
                        'status': 0, 'torque': 1} for i in range(6)}
    record = {'frames': [{'seq': 1, 't': .01, 'motors': joints}],
              'response': {'segments': [{'joint': 'j1', 'kind': 'step', 'target': 2070,
                                        'frames': [{'t': .02, 'position': 2065}]}]}}
    live = export_csv(record, 'live').decode().splitlines()
    response = export_csv(record, 'response').decode().splitlines()
    assert len(live) == 7 and 'j5' in live[-1]
    assert response[-1] == 'j1,0,step,0.02,2065,2070,5'
