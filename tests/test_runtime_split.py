"""Deployment keeps the hardware runtime independent of all simulation/client modules."""

import ast
import hashlib
import importlib.util
import math
from pathlib import Path

import pytest

from lerobot_arm_control.robot.so101.model_metadata import MODEL, joint_ranges
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle

ROOT = Path(__file__).resolve().parents[1]


def test_remote_manifest_is_complete_and_cannot_import_local_simulation_modules():
    paths = [line for line in (ROOT/'configs/remote-runtime.txt').read_text().splitlines()
             if line and not line.startswith('#')]
    assert len(paths) == len(set(paths))
    assert "scripts/lerobot-arm" in paths
    modules = {str(Path(path).with_suffix('')).removeprefix('src/').replace('/', '.')
               .removesuffix('.__init__') for path in paths
               if path.startswith('src/') and Path(path).suffix == '.py'}
    for path in paths:
        assert (ROOT/path).is_file(), path
        if not path.startswith('src/') or Path(path).suffix != '.py':
            continue
        module = str(Path(path).with_suffix('')).removeprefix('src/').replace('/', '.')
        package = module.rsplit('.', 1)[0]
        for node in ast.walk(ast.parse((ROOT/path).read_text())):
            if isinstance(node, ast.Import):
                for name in node.names:
                    assert name.name.split('.')[0] != 'mujoco', path
                    if name.name.startswith('lerobot_arm_control'):
                        assert name.name in modules, (path, name.name)
            if isinstance(node, ast.ImportFrom):
                assert node.module != 'mujoco', path
                imported = (importlib.util.resolve_name('.'*node.level + (node.module or ''),
                                                       package)
                            if node.level else node.module)
                if imported.startswith('lerobot_arm_control'):
                    assert imported in modules, (path, imported)
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == 'import_module' and node.args
                    and isinstance(node.args[0], ast.Constant)):
                imported = node.args[0].value
                if imported.startswith('lerobot_arm_control'):
                    assert imported in modules, (path, imported)


def test_metadata_ranges_match_the_canonical_explicit_joint_limits():
    assert joint_ranges()[0] == [-1.91986, 1.91986]
    assert len(joint_ranges()) == 6
    assert Circle().model_hash == hashlib.sha256(MODEL.read_bytes()).hexdigest()


@pytest.mark.parametrize('axis', [0, 1, 2])
@pytest.mark.parametrize('angle', [0, .5, math.pi-1e-6, math.pi, -2.5])
def test_camera_registration_quaternion_without_mujoco(axis, angle):
    import numpy as np

    from lerobot_arm_control.perception.tabletop_registration import rotation_quaternion

    cv2 = pytest.importorskip("cv2")

    vector = np.zeros(3)
    vector[axis] = angle
    rotation = cv2.Rodrigues(vector)[0]
    q = np.array(rotation_quaternion(rotation))
    assert np.linalg.norm(q) == pytest.approx(1)
    expected = np.zeros(4)
    expected[0], expected[axis+1] = math.cos(angle/2), math.sin(angle/2)
    assert abs(np.dot(q, expected)) == pytest.approx(1, abs=1e-10)
