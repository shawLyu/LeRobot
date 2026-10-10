"""A failed record update must preserve the last valid robot state."""

from pathlib import Path

import pytest

from lerobot_arm_control.core.files import atomic_json


def test_nonfinite_record_cannot_replace_last_valid_state(tmp_path):
    target = tmp_path / "state.json"
    target.write_text('{"sequence": 1}\n')
    with pytest.raises(ValueError):
        atomic_json(target, {"sequence": 2, "position": float("nan")})
    assert target.read_text() == '{"sequence": 1}\n'


def test_failed_atomic_replace_keeps_last_valid_state(tmp_path, monkeypatch):
    target = tmp_path / "state.json"
    target.write_text('{"sequence": 1}\n')

    def fail_replace(*args):
        raise OSError("record volume unavailable")

    monkeypatch.setattr(Path, "replace", fail_replace)
    with pytest.raises(OSError, match="volume unavailable"):
        atomic_json(target, {"sequence": 2})
    assert target.read_text() == '{"sequence": 1}\n'
