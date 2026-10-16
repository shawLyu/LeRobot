"""Loopback-only, dependency-free browser UI for the remote servo workbench."""

import argparse
import csv
import io
import json
import secrets
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.workbench.link import WorkbenchLink
from lerobot_arm_control.workbench.registers import BAUDRATES, schema

WEB = Path(__file__).with_name('web')
OPERATIONS = {'parameters', 'write', 'bounds', 'enable', 'hold', 'release', 'clear_fault',
              'move', 'step', 'result'}


def export_csv(record, scope):
    output = io.StringIO(newline='')
    writer = csv.writer(output, lineterminator='\n')
    if scope == 'response':
        writer.writerow(['joint', 'segment', 'kind', 't_seconds', 'position_ticks',
                         'target_ticks', 'error_ticks'])
        for index, segment in enumerate((record.get('response') or {}).get('segments', [])):
            for frame in segment['frames']:
                writer.writerow([segment['joint'], index, segment['kind'], frame['t'],
                                 frame['position'], segment['target'],
                                 segment['target'] - frame['position']])
    else:
        writer.writerow(['seq', 't_seconds', 'joint', 'position_ticks', 'goal_ticks',
                         'error_ticks', 'velocity_raw', 'load_raw', 'current_raw',
                         'voltage_V', 'temperature_C', 'status', 'torque'])
        for frame in record.get('frames', []):
            for joint, m in frame['motors'].items():
                writer.writerow([frame['seq'], frame['t'], joint, m['position'], m['goal'],
                    m['goal'] - m['position'], m['velocity'], m['load'], m['current_raw'],
                    m['voltage'], m['temperature'], m['status'], m['torque']])
    return output.getvalue().encode()


class WorkbenchServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, link, arms):
        self.link, self.arms, self.token = link, arms, secrets.token_urlsafe(32)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def respond(self, status, body, content_type='application/json; charset=utf-8', **headers):
        if not isinstance(body, bytes):
            body = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; img-src 'self' data: blob:; "
                         "frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        for key, value in headers.items():
            self.send_header(key.replace('_', '-'), value)
        self.end_headers()
        self.wfile.write(body)

    def check_host(self):
        hosts = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
        if self.headers.get('Host') not in hosts:
            raise PermissionError('仅允许本机工作台地址')
        origin = self.headers.get('Origin')
        if origin and origin not in {f'http://{host}' for host in hosts}:
            raise PermissionError('拒绝跨站请求')

    def check_api(self):
        self.check_host()
        if not secrets.compare_digest(self.headers.get('X-Workbench-Token', ''),
                                      self.server.token):
            raise PermissionError('无效的工作台会话')

    def do_GET(self):
        try:
            self.check_host()
            parsed = urlsplit(self.path)
            if parsed.path == '/':
                body = (WEB / 'index.html').read_text().replace('__TOKEN__', self.server.token)
                return self.respond(200, body.encode(), 'text/html; charset=utf-8')
            assets = {'/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                      '/measurement.js': ('measurement.js', 'text/javascript; charset=utf-8'),
                      '/style.css': ('style.css', 'text/css; charset=utf-8')}
            if parsed.path in assets:
                filename, content_type = assets[parsed.path]
                return self.respond(200, (WEB / filename).read_bytes(), content_type)
            if parsed.path == '/favicon.ico':
                return self.respond(204, b'')
            self.check_api()
            if parsed.path == '/api/info':
                result = {'arms': [{'name': arm.name, 'role': arm.role, 'port': arm.port,
                                    'ids': list(arm.motors.values())}
                                   for arm in self.server.arms.values()],
                          'schema': schema(), 'baudrates': BAUDRATES}
            elif parsed.path == '/api/state':
                result = self.server.link.state()
            elif parsed.path == '/api/result':
                result = self.server.link.request('result')
            elif parsed.path == '/api/export':
                record = self.server.link.request('export')
                options = parse_qs(parsed.query)
                if options.get('format') == ['csv']:
                    body = export_csv(record, options.get('scope', ['live'])[0])
                    return self.respond(200, body, 'text/csv; charset=utf-8',
                                        Content_Disposition='attachment; filename="servo-data.csv"')
                result = record
            else:
                return self.respond(404, {'ok': False, 'error': '未找到该地址'})
            self.respond(200, {'ok': True, 'result': result})
        except PermissionError as exc:
            self.respond(403, {'ok': False, 'error': str(exc)})
        except (ValueError, OSError, RuntimeError) as exc:
            self.respond(400, {'ok': False, 'error': str(exc)})

    def do_POST(self):
        try:
            self.check_api()
            if not self.headers.get('Content-Type', '').startswith('application/json'):
                raise ValueError('请使用 JSON 请求')
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 65536:
                raise ValueError('无效的请求大小')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('请求必须是 JSON 对象')
            if self.path == '/api/connect':
                result = self.server.link.connect(body['arm'], body['ids'], body['baudrate'])
            elif self.path == '/api/disconnect':
                self.server.link.close()
                result = {'disconnected': True}
            elif self.path == '/api/command':
                operation, payload = body['operation'], body.get('payload', {})
                if operation not in OPERATIONS or not isinstance(payload, dict):
                    raise ValueError('未知操作或无效参数')
                result = self.server.link.request(operation, payload)
            else:
                return self.respond(404, {'ok': False, 'error': '未找到该地址'})
            self.respond(200, {'ok': True, 'result': result})
        except PermissionError as exc:
            self.respond(403, {'ok': False, 'error': str(exc)})
        except (KeyError, TypeError, ValueError, OSError, RuntimeError) as exc:
            self.respond(400, {'ok': False, 'error': str(exc)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    add_remote_arguments(parser)
    args = parser.parse_args()
    arms = load_arms()
    link = WorkbenchLink(arms, host=args.host, root=args.remote_root)
    server = WorkbenchServer(('127.0.0.1', args.port), link, arms)
    print(f'舵机工作台：http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        server.server_close()


if __name__ == '__main__':
    main()
