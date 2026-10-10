"""A successful command acknowledgement must not interrupt a hardware trial."""

import time
from types import SimpleNamespace

import pytest

pytest.importorskip('mujoco')
from lerobot_arm_control.diagnostics import response as verification  # noqa: E402


def test_successful_ack_is_not_a_worker_error():
    state = {'control': {'last_fault': None}}
    link = SimpleNamespace(error=None, process=SimpleNamespace(poll=lambda: None),
                           snapshot=lambda: (None, state, time.monotonic(), 1, {'ok': True}))
    assert verification.motor_observation(link) is state
    link.error = 'disconnected'
    with pytest.raises(RuntimeError, match='disconnected'):
        verification.motor_observation(link)
