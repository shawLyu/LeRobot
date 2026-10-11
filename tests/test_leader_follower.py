"""Teleoperation preview, hardware execution boundaries and upstream arguments."""

import json
import os
import subprocess
import sys
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from lerobot_arm_control.cli import main as cli_main
from lerobot_arm_control.core.config import ArmConfig
from lerobot_arm_control.runtime import leader_follower as teleop


@pytest.fixture
def plan(tmp_path, monkeypatch):
    arms = {
        name: ArmConfig(name, str(tmp_path / name), name, "sts3215", 1_000_000,
                        dict(teleop.MOTORS))
        for name in ("leader", "follower")
    }
    monkeypatch.setattr(teleop, "load_arms", lambda path: arms)
    args = teleop.parser().parse_args([
        "--leader", "leader", "--leader-id", "test_leader", "--follower-id", "test_follower",
        "--calibration-root", str(tmp_path / "calibration"), "--duration", "10",
    ])
    return args, arms, teleop.build_plan(args)


def calibrated(plan):
    for path in plan.calibration_files:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({
            name: dict(id=motor_id, drive_mode=0, homing_offset=0, range_min=100, range_max=3900)
            for name, motor_id in teleop.MOTORS.items()
        }), encoding="utf-8")
    for arm in (plan.leader, plan.follower):
        Path(arm.port).touch()


def test_upstream_command_uses_same_interpreter_and_explicit_units(plan):
    _, _, built = plan
    assert built.command[:3] == [sys.executable, "-m", "lerobot.scripts.lerobot_teleoperate"]
    assert "--teleop.use_degrees=true" in built.command
    assert "--robot.use_degrees=true" in built.command
    assert "--robot.max_relative_target=1.0" in built.command
    assert "--robot.disable_torque_on_disconnect=true" in built.command
    assert "--display_data=false" in built.command
    assert "--teleop_time_s=10.0" in built.command
    for prefix, path in zip(("teleop", "robot"), built.calibration_files, strict=True):
        assert f"--{prefix}.calibration_dir={path.parent}" in built.command


@pytest.mark.parametrize("change, message", [
    ("same_entry", "different arm entries"),
    ("unassigned", "confirm hardware"),
    ("bad_map", "standard joint IDs"),
    ("same_device", "same USB device"),
    ("same_id", "distinct"),
    ("unknown", "Unknown leader"),
    ("fps", "fps"),
])
def test_reject_invalid_pair(plan, change, message):
    args, arms, _ = plan
    if change == "same_entry":
        args.leader = args.follower
    elif change == "unassigned":
        arms["leader"] = replace(arms["leader"], role="unassigned")
    elif change == "bad_map":
        arms["leader"] = replace(arms["leader"], motors={"gripper": 6})
    elif change == "same_device":
        arms["leader"] = replace(arms["leader"], port=arms["follower"].port + "/../follower")
    elif change == "same_id":
        args.leader_id = args.follower_id
    elif change == "unknown":
        args.leader = "missing"
    else:
        args.fps = 0
    with pytest.raises(ValueError, match=message):
        teleop.build_plan(args)


@pytest.mark.parametrize("option, value", [
    ("--leader-id", "../other"), ("--duration", "nan"), ("--duration", "-1"),
    ("--max-relative-target", "inf"), ("--max-relative-target", "0"),
])
def test_reject_invalid_cli_values(option, value):
    with pytest.raises(SystemExit) as error:
        teleop.parser().parse_args([
            "--leader", "leader", "--leader-id", "leader", "--follower-id", "follower",
            option, value,
        ])
    assert error.value.code == 2


def test_calibration_rejects_missing_empty_wrong_ids_and_zero_ranges(plan):
    _, _, built = plan
    path = built.calibration_files[0]
    with pytest.raises(ValueError, match="Missing upstream"):
        teleop.validate_calibration(path)
    calibrated(built)
    teleop.validate_calibration(path)
    original = json.loads(path.read_text())
    for invalid, message in (({}, "all six"),
                             ({**original, "gripper": {**original["gripper"], "id": 5}}, "ID"),
                             ({**original, "gripper": {**original["gripper"], "range_max": 100}},
                              "encoder range")):
        path.write_text(json.dumps(invalid))
        with pytest.raises(ValueError, match=message):
            teleop.validate_calibration(path)


def test_execute_reserves_both_ports_and_releases_on_child_failure(plan, monkeypatch):
    _, _, built = plan
    calibrated(built)
    monkeypatch.setattr(teleop.sys, "platform", "linux")
    monkeypatch.setattr(teleop, "version", lambda name: "0.6.1")
    owned = set()

    @contextmanager
    def owner(port):
        owned.add(port)
        try:
            yield
        finally:
            owned.remove(port)

    monkeypatch.setitem(sys.modules, "lerobot_arm_control.robot.so101.driver",
                        SimpleNamespace(serial_owner=owner))

    def child(command):
        assert command == built.command
        assert owned == {built.leader.port, built.follower.port}
        raise RuntimeError("upstream failed")

    monkeypatch.setattr(teleop, "run_child", child)
    with pytest.raises(RuntimeError, match="upstream failed"):
        teleop.execute(built)
    assert not owned


def test_execute_does_not_spawn_without_connected_devices_and_calibration(plan, monkeypatch):
    _, _, built = plan
    monkeypatch.setattr(teleop.sys, "platform", "linux")
    monkeypatch.setattr(teleop, "version", lambda name: "0.6.1")
    monkeypatch.setattr(teleop, "run_child", lambda command: pytest.fail("must not spawn"))
    with pytest.raises(ValueError, match="not connected"):
        teleop.execute(built)
    for arm in (built.leader, built.follower):
        Path(arm.port).touch()
    with pytest.raises(ValueError, match="Missing upstream"):
        teleop.execute(built)


def test_preview_and_help_from_cli_need_no_hardware_packages(tmp_path):
    config = tmp_path / "arms.toml"
    config.write_text("\n".join(
        f'''[arms.{role}]
port = "/dev/serial/by-id/{role}"
role = "{role}"
model = "sts3215"
baudrate = 1000000
[arms.{role}.motors]
''' + "\n".join(f"{name} = {motor_id}" for name, motor_id in teleop.MOTORS.items())
        for role in ("leader", "follower")
    ))
    code = '''
import sys
from lerobot_arm_control.cli import main
try:
    main(['teleoperate', '--help'])
except SystemExit as error:
    assert error.code == 0
main(['teleoperate', '--leader', 'leader', '--leader-id', 'leader',
      '--follower-id', 'follower', '--config', sys.argv[1]])
assert not {'lerobot', 'serial', 'torch', 'mujoco', 'cv2', 'fcntl'} & set(sys.modules)
'''
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([sys.executable, "-c", code, str(config)], cwd=tmp_path,
                            env=dict(os.environ, PYTHONPATH=str(root / "src")),
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "Preview only" in result.stdout
    assert not (tmp_path / "calibration").exists()


def test_cli_forwarding(monkeypatch):
    seen = []
    monkeypatch.setattr("lerobot_arm_control.cli.importlib.import_module",
                        lambda name: SimpleNamespace(
                            main=lambda: seen.append((name, sys.argv[1:]))))
    cli_main(["teleoperate", "--leader", "arm1", "--execute"])
    assert seen == [("lerobot_arm_control.runtime.leader_follower",
                     ["--leader", "arm1", "--execute"])]


def test_ctrl_c_waits_for_child_before_releasing_reservations(monkeypatch):
    events = []

    class Child:
        pid = 123

        def poll(self):
            return None

        def wait(self):
            events.append("wait")
            if len(events) == 1:
                raise KeyboardInterrupt
            return 0

    monkeypatch.setattr(teleop.subprocess, "Popen", lambda command, **kwargs: Child())
    monkeypatch.setattr(teleop.os, "killpg", lambda pid, sig: events.append("interrupt"),
                        raising=False)
    assert teleop.run_child(["unused"]) == 0
    assert events == ["wait", "interrupt", "wait"]


def test_termination_is_relayed_and_signal_handlers_restored(monkeypatch):
    handlers = {}
    events = []

    class Child:
        pid = 123

        def poll(self):
            return None

        def wait(self):
            events.append("wait")
            if len(events) == 1:
                handlers[teleop.signal.SIGTERM](teleop.signal.SIGTERM, None)
            return 0

    monkeypatch.setattr(teleop.subprocess, "Popen", lambda command, **kwargs: Child())
    monkeypatch.setattr(teleop.signal, "getsignal", lambda sig: "previous")
    monkeypatch.setattr(teleop.signal, "signal",
                        lambda sig, handler: handlers.update({sig: handler}))
    monkeypatch.setattr(teleop.os, "killpg", lambda pid, sig: events.append("interrupt"),
                        raising=False)
    assert teleop.run_child(["unused"]) == 0
    assert events == ["wait", "interrupt", "wait"]
    assert set(handlers.values()) == {"previous"}


def test_calibration_environment_precedence(tmp_path, monkeypatch):
    for key in ("HF_LEROBOT_CALIBRATION", "HF_LEROBOT_HOME", "HF_HOME", "XDG_CACHE_HOME"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    assert teleop.calibration_root() == tmp_path / "cache/huggingface/lerobot/calibration"
    monkeypatch.setenv("HF_HOME", str(tmp_path / "hf"))
    assert teleop.calibration_root() == tmp_path / "hf/lerobot/calibration"
    monkeypatch.setenv("HF_LEROBOT_HOME", str(tmp_path / "robot"))
    assert teleop.calibration_root() == tmp_path / "robot/calibration"
    monkeypatch.setenv("HF_LEROBOT_CALIBRATION", str(tmp_path / "chosen"))
    assert teleop.calibration_root() == tmp_path / "chosen"


@pytest.mark.parametrize("platform, installed, message", [
    ("win32", "0.6.1", "Linux host"), ("linux", "0.6.0", "locked LeRobot"),
    ("linux", None, "LeRobot is missing"),
])
def test_execution_requires_linux_and_locked_hardware_environment(
    plan, monkeypatch, platform, installed, message,
):
    monkeypatch.setattr(teleop.sys, "platform", platform)

    def package_version(name):
        if installed is None:
            raise teleop.PackageNotFoundError(name)
        return installed

    monkeypatch.setattr(teleop, "version", package_version)
    monkeypatch.setattr(teleop, "run_child", lambda command: pytest.fail("must not spawn"))
    with pytest.raises(ValueError, match=message):
        teleop.execute(plan[2])
