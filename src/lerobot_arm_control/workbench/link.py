"""One SSH worker per selected arm; display polling never clocks motor motion."""

import json
import shlex
import subprocess
import threading
import time
from collections import deque

from lerobot_arm_control.core.runtime_config import remote_runtime
from lerobot_arm_control.workbench.registers import BAUDRATES


class WorkbenchLink:
    def __init__(self, arms, *, host=None, root=None):
        self.arms = arms
        self.remote = remote_runtime(host, root)
        self.process = None
        self.hello = self.latest = None
        self.last_update = 0
        self.error = None
        self.stderr = deque(maxlen=20)
        self.condition = threading.Condition()
        self.operations = threading.RLock()
        self.writer = threading.Lock()
        self.pending = {}
        self.counter = 0
        self.last_heartbeat = 0

    def connect(self, name, ids, baudrate):
        if name not in self.arms:
            raise ValueError('未知机械臂')
        if (not isinstance(ids, list) or len(ids) != 6 or len(set(ids)) != 6
                or any(type(v) is not int or not 1 <= v <= 253 for v in ids)
                or type(baudrate) is not int or baudrate not in BAUDRATES):
            raise ValueError('请选择有效波特率和六个互不重复的 ID')
        with self.operations:
            self.close()
            command = ['.venv/bin/python', '-u', '-m',
                       'lerobot_arm_control.workbench.worker', '--arm', name,
                       '--ids', ','.join(map(str, ids)), '--baudrate', str(baudrate)]
            remote_command = ('cd ' + shlex.quote(self.remote.root)
                              + ' && exec env PYTHONPATH=src ' + shlex.join(command))
            self.hello = self.latest = None
            self.error = None
            self.stderr.clear()
            process = subprocess.Popen(['ssh', '-T', '-o', 'BatchMode=yes',
                                        '-o', 'ConnectTimeout=8', self.remote.host, remote_command],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True, bufsize=1)
            self.process = process
            threading.Thread(target=self.read_stdout, args=(process,), daemon=True).start()
            threading.Thread(target=self.read_stderr, args=(process,), daemon=True).start()
            with self.condition:
                ready = self.condition.wait_for(lambda: self.hello is not None or self.error
                                                 or process.poll() is not None, timeout=15)
            if not ready or self.hello is None:
                error = self.error or '\n'.join(self.stderr) or '远端连接超时'
                self.close()
                raise ConnectionError(error)
            return self.hello

    def read_stderr(self, process):
        for line in process.stderr:
            if process is self.process:
                self.stderr.append(line.rstrip())

    def read_stdout(self, process):
        try:
            for line in process.stdout:
                message = json.loads(line)
                with self.condition:
                    if process is not self.process:
                        return
                    kind = message.get('type')
                    if kind == 'hello':
                        self.hello = message
                        self.latest = message['state']
                    elif kind == 'state':
                        self.latest = message
                    elif kind == 'ack' and message.get('id') in self.pending:
                        self.pending[message['id']] = message
                    elif kind == 'error':
                        self.error = message['error']
                    self.last_update = time.monotonic()
                    self.condition.notify_all()
        except (ValueError, OSError) as exc:
            if process is self.process:
                self.error = str(exc)
        finally:
            with self.condition:
                if process is self.process:
                    self.error = self.error or '远端连接已关闭；已请求保持'
                    self.condition.notify_all()

    def send(self, message):
        with self.writer:
            process = self.process
            if process is None or process.poll() is not None or process.stdin.closed:
                raise ConnectionError(self.error or '请先连接机械臂')
            process.stdin.write(json.dumps(message, allow_nan=False) + '\n')
            process.stdin.flush()

    def request(self, operation, payload=None):
        with self.operations:
            with self.condition:
                self.counter += 1
                identifier = self.counter
                self.pending[identifier] = None
            try:
                self.send({'id': identifier, 'operation': operation, 'payload': payload or {}})
                with self.condition:
                    ready = self.condition.wait_for(
                        lambda: self.pending[identifier] is not None or self.error, timeout=12)
                    response = self.pending[identifier]
                if not ready or response is None:
                    raise ConnectionError(self.error or '远端指令超时')
                if not response['ok']:
                    raise ValueError(response.get('error', '指令被拒绝'))
                result = response.get('result')
                if operation == 'write' and isinstance(result, dict) and result.get('baudrate'):
                    self.hello['baudrate'] = result['baudrate']
                return result
            finally:
                with self.condition:
                    self.pending.pop(identifier, None)

    def state(self):
        connected = (self.process is not None and self.process.poll() is None
                     and self.hello is not None and self.error is None)
        if connected and time.monotonic() - self.last_heartbeat >= .25:
            try:
                self.send({'id': 'heartbeat', 'operation': 'heartbeat'})
                self.last_heartbeat = time.monotonic()
            except (OSError, ValueError) as exc:
                self.error = str(exc)
                connected = False
        with self.condition:
            return {'connected': connected, 'hello': self.hello, 'state': self.latest,
                    'age_seconds': (time.monotonic() - self.last_update
                                    if self.last_update else None),
                    'error': self.error, 'stderr': list(self.stderr)}

    def close(self):
        with self.operations:
            process = self.process
            if process is None:
                return
            try:
                if process.poll() is None and self.hello is not None:
                    self.request('disconnect')
            except (ValueError, OSError):
                pass
            try:
                if not process.stdin.closed:
                    process.stdin.close()
                process.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            for stream in (process.stdout, process.stderr):
                stream.close()
            self.process = None
            self.error = None
            self.hello = self.latest = None
