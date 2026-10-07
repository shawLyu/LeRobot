"""Deployment keeps the hardware runtime independent of all simulation/client modules."""

import ast
import hashlib
import json
import math
from pathlib import Path

import pytest

from arm_control.circle_trajectory import Circle
from arm_control.control_defaults import POSITION_FEEDBACK, TRAJECTORY
from arm_control.model_metadata import MODEL, joint_ranges

ROOT = Path(__file__).resolve().parents[1]


def test_remote_manifest_is_complete_and_cannot_import_local_simulation_modules():
    paths = [line for line in (ROOT/'configs/remote-runtime.txt').read_text().splitlines()
             if line and not line.startswith('#')]
    assert len(paths) == len(set(paths))
    modules = {Path(path).stem for path in paths if path.startswith('src/')}
    for path in paths:
        assert (ROOT/path).is_file(), path
        if not path.startswith('src/'):
            continue
        for node in ast.walk(ast.parse((ROOT/path).read_text())):
            if isinstance(node, ast.Import):
                assert all(name.name != 'mujoco' for name in node.names), path
            if isinstance(node, ast.ImportFrom):
                assert node.module != 'mujoco', path
                if node.level:
                    assert node.module.split('.')[0] in modules, (path, node.module)


def test_metadata_ranges_match_the_canonical_explicit_joint_limits():
    assert joint_ranges()[0] == [-1.91986, 1.91986]
    assert len(joint_ranges()) == 6
    assert Circle().model_hash == hashlib.sha256(MODEL.read_bytes()).hexdigest()
    assert hashlib.sha256(json.dumps(POSITION_FEEDBACK, sort_keys=True).encode()).hexdigest() == (
        '62effcc919c99dbc0e7a32f5af3dd465a05f1ced419d0e065a4aa1e34af77f95')
    assert TRAJECTORY['control_hz'] == 60
    assert TRAJECTORY['period_seconds'] == 4


@pytest.mark.parametrize('axis', [0, 1, 2])
@pytest.mark.parametrize('angle', [0, .5, math.pi-1e-6, math.pi, -2.5])
def test_camera_registration_quaternion_without_mujoco(axis, angle):
    import numpy as np

    from arm_control.tabletop_registration import rotation_quaternion

    cv2 = pytest.importorskip("cv2")

    vector = np.zeros(3)
    vector[axis] = angle
    rotation = cv2.Rodrigues(vector)[0]
    q = np.array(rotation_quaternion(rotation))
    assert np.linalg.norm(q) == pytest.approx(1)
    expected = np.zeros(4)
    expected[0], expected[axis+1] = math.cos(angle/2), math.sin(angle/2)
    assert abs(np.dot(q, expected)) == pytest.approx(1, abs=1e-10)
