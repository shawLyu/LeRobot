import threading

from lerobot_arm_control.transport.latest_output import LatestOutput


def test_slow_display_never_blocks_control_and_receives_latest_state():
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    received = []

    def send(message):
        if not received:
            entered.set()
            assert release.wait(2)
        received.append(message)
        if message.get('seq') == 99:
            finished.set()

    disconnected = threading.Event()
    with LatestOutput(send, disconnected) as publish:
        publish({'type': 'hello'})
        assert entered.wait(2)
        for seq in range(100):
            publish({'type': 'state', 'seq': seq})
        publish({'type': 'ack', 'id': 1})
        publish({'type': 'ack', 'id': 2})
        release.set()
        assert finished.wait(2)
    assert received == [{'type': 'hello'}, {'type': 'ack', 'id': 1},
                        {'type': 'ack', 'id': 2}, {'type': 'state', 'seq': 99}]
    assert not disconnected.is_set()


def test_broken_output_requests_hardware_disconnect():
    disconnected = threading.Event()

    def broken(message):
        raise BrokenPipeError('closed')

    with LatestOutput(broken, disconnected) as publish:
        publish({'type': 'state'})
        assert disconnected.wait(2)
