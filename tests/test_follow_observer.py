import json
import socket
import tempfile
import threading
from pathlib import Path

import pytest

from lerobot_arm_control.transport.follow_observer import FollowBroadcast


@pytest.fixture
def socket_path():
    with tempfile.TemporaryDirectory(dir="/tmp", prefix="follow-") as directory:
        yield Path(directory)/"follow.sock"


def test_viewers_are_optional_read_only_and_reconnect_to_latest_state(socket_path):
    path = socket_path
    with FollowBroadcast(path) as publish:
        publish({'type': 'hello', 'remote_autonomous': True})
        for index in range(1000):
            publish({'type': 'state', 'seq': index})
        for _ in range(2):
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(2)
                client.connect(str(path))
                # Even a command written to the socket is never interpreted.
                client.sendall(b'{"op":"hold"}\n')
                with client.makefile('r') as reader:
                    assert json.loads(reader.readline())['remote_autonomous']
                    assert json.loads(reader.readline())['seq'] == 999
        with pytest.raises(RuntimeError, match='already running'):
            with FollowBroadcast(path):
                pass
        assert path.exists()
    assert not path.exists()


def test_stalled_observer_cannot_block_publishing(socket_path):
    path = socket_path
    with FollowBroadcast(path) as publish:
        publish({'type': 'hello'})
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(path))
            completed = threading.Event()

            def write():
                for index in range(1000):
                    publish({'type': 'state', 'seq': index, 'data': 'x'*10000})
                completed.set()

            thread = threading.Thread(target=write)
            thread.start()
            assert completed.wait(1)
            thread.join()
