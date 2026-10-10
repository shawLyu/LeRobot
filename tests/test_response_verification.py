"""A successful command acknowledgement must not interrupt a hardware trial."""

import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip('mujoco')
spec = importlib.util.spec_from_file_location(
    'verify_response', Path(__file__).parents[1]/'scripts/verify_response.py')
verification = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verification)


def test_successful_ack_is_not_a_worker_error():
    state = {'control': {'last_fault': None}}
    link = SimpleNamespace(error=None, process=SimpleNamespace(poll=lambda: None),
                           snapshot=lambda: (None, state, time.monotonic(), 1, {'ok': True}))
    assert verification.motor_observation(link) is state
    link.error = 'disconnected'
    with pytest.raises(RuntimeError, match='disconnected'):
        verification.motor_observation(link)
