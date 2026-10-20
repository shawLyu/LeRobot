"""Bind saved table circles to the observed tool without enabling hardware."""

import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot_arm_control.core.files import file_digest
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.planning.circle_simulation import (
    bind_held_joints,
    validate_hardware_entry,
)
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle


@pytest.mark.parametrize('name', ['table-circle120-h38cm', 'table-circle120-tilt30'])
def test_rebound_circle_preserves_cartesian_path_pitch_and_observed_tool(tmp_path, name):
    source = resource_path(f'data/trajectories/{name}.json')
    digest = file_digest(source)
    original = Circle(source)
    initial = list(original.points[0])
    initial[4:] = [0., -1.614196225307055]
    with pytest.raises(ValueError, match='hold the observed'):
        validate_hardware_entry(original, initial)
    path = bind_held_joints(source, initial, tmp_path/'bound.json')
    bound = Circle(path)
    proof = validate_hardware_entry(bound, initial)
    assert proof['checked'] and proof['plan_sha256'] == file_digest(path)
    assert file_digest(source) == digest
    assert bound.plan_hash != original.plan_hash
    assert bound.center == original.center and bound.radius == original.radius
    assert bound.points[0] == bound.points[-1]
    assert all(q[4:] == initial[4:] for q in bound.points)
    np.testing.assert_allclose(np.sum(np.array(bound.points)[:, 1:4], axis=1),
                               np.sum(np.array(original.points)[:, 1:4], axis=1), atol=1e-6)
    assert not list(tmp_path.glob('*.candidate.json'))


def test_matching_pose_reuses_original_plan_without_writing_copy(tmp_path):
    source = resource_path('data/trajectories/table-circle-8cm.json')
    assert bind_held_joints(source, Circle(source).points[0], tmp_path/'unused.json') == source
    assert not list(tmp_path.iterdir())


def test_failed_binding_validation_leaves_no_executable_plan(tmp_path, monkeypatch):
    from lerobot_arm_control.planning import circle_simulation

    source = resource_path('data/trajectories/table-circle120-h38cm.json')
    initial = list(Circle(source).points[0])
    initial[4] = 0.

    def reject(*args):
        raise ValueError('trajectory collision')

    monkeypatch.setattr(circle_simulation, 'validate_hardware_entry', reject)
    with pytest.raises(ValueError, match='collision'):
        bind_held_joints(source, initial, tmp_path/'bound.json')
    assert not list(tmp_path.iterdir())


def test_entry_preparation_uploads_bound_plan_and_matching_proof_without_control(
        tmp_path, monkeypatch):
    from lerobot_arm_control.diagnostics import circle_tracking

    source = resource_path('data/trajectories/table-circle120-h38cm.json')
    initial = list(Circle(source).points[0])
    initial[4:] = [0., -1.614196225307055]
    closed, copies = [], []

    class Reader:
        error = None
        process = SimpleNamespace(poll=lambda: None)

        def __init__(self, host, root, backend, **options):
            assert backend == 'hardware'
            assert not options.get('hardware_control')
            assert not options.get('circle')

        def snapshot(self):
            return {'fingerprint': {}}, {'raw': {}}, time.monotonic(), 1, None

        def close(self):
            closed.append(True)

    def copy(command, **options):
        assert closed
        assert command[0] == 'scp'
        copies.append(command)

    monkeypatch.setattr(circle_tracking, 'ROOT', tmp_path)
    monkeypatch.setattr(circle_tracking, 'Link', Reader)
    monkeypatch.setattr(circle_tracking, 'AngleCalibration',
                        lambda *args: SimpleNamespace(angles=lambda *args: initial))
    monkeypatch.setattr(circle_tracking.subprocess, 'run', copy)
    proof_path, plan_path = circle_tracking.prepare_remote_entry(
        SimpleNamespace(host='robot-host', root='/home/robot/lerobot'), source, 'test')
    proof = json.loads(proof_path.read_text())
    assert proof['plan_sha256'] == file_digest(plan_path)
    assert proof['initial_qpos'] == initial
    assert len(copies) == 2
    assert copies[0][1] == str(plan_path) and copies[1][1] == str(proof_path)
