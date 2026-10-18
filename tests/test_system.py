"""Source execution, configuration isolation and deployment boundaries."""

import importlib
import json
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from lerobot_arm_control import load_arms, resource_path, workspace_root
from lerobot_arm_control.cli import main
from lerobot_arm_control.core.paths import source_environment
from lerobot_arm_control.core.processes import remote_python

ROOT = Path(__file__).resolve().parents[1]


def test_source_import_and_help_from_another_directory_need_no_optional_drivers(tmp_path):
    code = """
import sys
import lerobot_arm_control
from lerobot_arm_control.cli import main
from lerobot_arm_control.robot.so101.model_metadata import joint_ranges
assert len(joint_ranges()) == 6
try:
    main(['hardware', '--help'])
except SystemExit as error:
    assert error.code == 0
assert not {'lerobot', 'torch', 'mujoco', 'cv2', 'serial'} & set(sys.modules)
"""
    environment = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
    environment.pop("LEROBOT_ARM_WORKSPACE", None)
    result = subprocess.run([sys.executable, "-c", code], cwd=tmp_path, env=environment,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert not list(tmp_path.iterdir())


def test_workspace_override_never_borrows_another_robots_resources(tmp_path, monkeypatch):
    monkeypatch.setenv("LEROBOT_ARM_WORKSPACE", str(tmp_path))
    assert workspace_root() == tmp_path
    (tmp_path / "configs").mkdir()
    path = tmp_path / "configs/arms.toml"
    path.write_text('''[arms.test]
port = "/dev/serial/by-id/test-device"
role = "follower"
model = "sts3215"
baudrate = 1000000
[arms.test.motors]
gripper = 6
''')
    assert load_arms()["test"].port == "/dev/serial/by-id/test-device"
    shared = tmp_path / "configs/control.json"
    with pytest.raises(FileNotFoundError, match="control.json"):
        resource_path("configs/control.json")
    shared.write_text("custom-shared-config")
    assert resource_path("configs/control.json") == shared
    with pytest.raises(FileNotFoundError, match="so101.xml"):
        resource_path("assets/so101/so101.xml")
    assert resource_path("configs") == tmp_path / "configs"


@pytest.mark.parametrize("relative", ["../outside", "/absolute", "configs/../../outside"])
def test_resource_path_rejects_paths_outside_workspace(relative):
    with pytest.raises(ValueError, match="relative"):
        resource_path(relative)


def test_nonexistent_workspace_is_not_created(tmp_path, monkeypatch):
    missing = tmp_path / "missing"
    monkeypatch.setenv("LEROBOT_ARM_WORKSPACE", str(missing))
    with pytest.raises(FileNotFoundError, match="workspace does not exist"):
        workspace_root()
    assert not missing.exists()


def test_local_worker_gets_source_even_with_no_pythonpath(monkeypatch, tmp_path):
    monkeypatch.delenv("PYTHONPATH", raising=False)
    environment = source_environment()
    result = subprocess.run(
        [sys.executable, "-c",
         "from lerobot_arm_control.core.paths import source_root; print(source_root())"],
        cwd=tmp_path, env=environment, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(ROOT)


def test_remote_worker_quotes_paths_and_uses_remote_workspace():
    root = "/remote robot/project's files; ignored"
    argument = "log path; ignored"
    command = remote_python(root, "lerobot_arm_control.runtime.realtime_server",
                            ["--control-log", argument])
    assert command.startswith("cd " + shlex.quote(root) + " && exec ")
    assert shlex.split(command.split(" && exec ", 1)[1]) == [
        "env", "PYTHONPATH=src", "LEROBOT_ARM_WORKSPACE=.", ".venv/bin/python",
        "-u", "-m", "lerobot_arm_control.runtime.realtime_server", "--control-log", argument,
    ]
    with pytest.raises(ValueError, match="absolute"):
        remote_python("relative", "unused")


def test_unified_entry_forwards_execution_options_and_restores_argv(monkeypatch):
    seen = []
    original = sys.argv
    module = SimpleNamespace(main=lambda: seen.append(sys.argv[1:]))
    monkeypatch.setattr("lerobot_arm_control.cli.importlib.import_module", lambda name: module)
    main(["view", "arm", "--backend", "hardware", "--hardware-control"])
    assert seen == [["--backend", "mujoco", "--backend", "hardware", "--hardware-control"]]
    assert sys.argv is original


def test_public_domain_exports_use_canonical_implementations():
    from lerobot_arm_control.calibration import AngleCalibration
    from lerobot_arm_control.control import JointPath, ManualControl, PositionFeedback
    from lerobot_arm_control.robot.so101 import connected, joint_ranges
    from lerobot_arm_control.tasks.circle import Circle

    expected = [
        (ManualControl, "control.manual_control", "ManualControl"),
        (PositionFeedback, "control.position_feedback", "PositionFeedback"),
        (JointPath, "control.joint_path", "JointPath"),
        (Circle, "tasks.circle.circle_trajectory", "Circle"),
        (AngleCalibration, "calibration.angle_calibration", "AngleCalibration"),
        (connected, "robot.connection", "connected"),
    ]
    for value, module, attribute in expected:
        assert value is getattr(importlib.import_module("lerobot_arm_control." + module), attribute)
    assert len(joint_ranges()) == 6


def test_remote_source_subset_imports_without_simulation_or_hardware_drivers(tmp_path):
    paths = (ROOT / "configs/remote-runtime.txt").read_text().splitlines()
    modules = []
    for relative in paths:
        if not relative or relative.startswith("#"):
            continue
        target = tmp_path / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, target)
        if relative.startswith("src/lerobot_arm_control/") and relative.endswith(".py"):
            modules.append(relative.removeprefix("src/").removesuffix(".py")
                           .replace("/", ".").removesuffix(".__init__"))
    shutil.copytree(ROOT / "configs", tmp_path / "configs", dirs_exist_ok=True)
    code = """
import importlib, json, sys
for module in json.loads(sys.argv[1]):
    importlib.import_module(module)
assert not {'mujoco', 'lerobot', 'torch', 'serial'} & set(sys.modules)
"""
    environment = dict(os.environ, PYTHONPATH=str(tmp_path / "src"),
                       LEROBOT_ARM_WORKSPACE=str(tmp_path))
    result = subprocess.run([sys.executable, "-c", code, json.dumps(modules)], cwd=tmp_path,
                            env=environment, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
