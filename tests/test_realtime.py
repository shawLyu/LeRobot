import io
import json
import math
import shlex
import sys
import time
from pathlib import Path
from unittest.mock import Mock

import pytest

from lerobot_arm_control.runtime.realtime_client import ROOT, Link
from lerobot_arm_control.transport.realtime import JOINTS, raw_to_angles, validate_target


@pytest.mark.parametrize("invalid", [
    None, [0] * 5, [0] * 7, [math.nan] * 6, [math.inf] * 6, [True] * 6,
    ["0"] * 6,
])
def test_remote_target_rejects_invalid_data(invalid):
    with pytest.raises(ValueError):
        validate_target(invalid, [0] * 6, [[-1, 1]] * 6)


def test_remote_target_honors_model_limits_even_with_small_delta():
    assert validate_target([1.01] * 6, [0.99] * 6, [[-1, 1]] * 6) == [1] * 6


def test_bounded_target_is_accepted():
    assert validate_target([0.03] * 6, [0] * 6, [[-1, 1]] * 6) == [0.03] * 6


def test_estimated_encoder_mapping_has_explicit_endpoints():
    limits = {name: [100, 200] for name in JOINTS}
    raw = {name: 150 for name in JOINTS}
    assert raw_to_angles(raw, limits, [[-1, 1]] * 6) == [0] * 6
    raw["gripper"] = 200
    assert raw_to_angles(raw, limits, [[-1, 1]] * 6)[5] == 1
    limits["gripper"] = [100, 100]
    with pytest.raises(ValueError, match="encoder limits"):
        raw_to_angles(raw, limits, [[-1, 1]] * 6)


@pytest.mark.parametrize("independent", [True, False])
def test_simulation_state_can_maintain_heartbeat_without_render_loop(monkeypatch, independent):
    class Process:
        stdin = io.StringIO()
        stdout = io.StringIO(json.dumps({"version": 1, "type": "state", "seq": 0}) + "\n")

    process = Process()
    monkeypatch.setattr("lerobot_arm_control.runtime.realtime_client.subprocess.Popen",
        lambda *a, **k: process)
    link = Link("unused", "/unused", "mujoco", keep_simulation_alive=independent)
    link.reader.join(timeout=1)
    assert not link.reader.is_alive()
    assert link.received_count == 1
    assert link.error is None
    writes = process.stdin.getvalue()
    if independent:
        assert json.loads(writes)["op"] == "ping"
    else:
        assert writes == ""


def test_hardware_rejects_independent_simulation_heartbeat_before_connecting():
    with pytest.raises(ValueError, match="simulation-only"):
        Link("unused", "/unused", "hardware", keep_simulation_alive=True)


def test_hardware_keeps_ssh_routing_and_control_flags(monkeypatch):
    process = Mock(stdin=io.StringIO(), stdout=io.StringIO())
    popen = Mock(return_value=process)
    monkeypatch.setattr("lerobot_arm_control.runtime.realtime_client.subprocess.Popen", popen)
    link = Link("robot-host", "/remote project/robot", "hardware", hardware_control=True,
                circle=True, simulation_model="/local-only/model.xml")
    link.reader.join(timeout=1)
    launch = popen.call_args.args[0]
    assert launch[0] == "ssh"
    assert launch[-2] == "robot-host"
    assert launch[-1] == (
        f"cd {shlex.quote('/remote project/robot')} && exec env PYTHONPATH=src "
        "LEROBOT_ARM_WORKSPACE=. .venv/bin/python -u "
        "-m lerobot_arm_control.runtime.realtime_server --hardware-control "
        "--circle"
    )
    assert link.worker_location == "remote"
    assert not process.stdin.getvalue()


@pytest.mark.parametrize("matches", [False, True])
def test_hardware_control_rejects_configuration_drift_before_commands(monkeypatch, matches):
    from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256

    hello = {"version": 1, "type": "hello", "joints": list(JOINTS),
             "control_config_sha256": CONTROL_CONFIG_SHA256 if matches else "outdated"}
    process = Mock(stdin=io.StringIO(), stdout=io.StringIO(json.dumps(hello) + "\n"))
    monkeypatch.setattr("lerobot_arm_control.runtime.realtime_client.subprocess.Popen",
        Mock(return_value=process))
    link = Link("robot-host", "/remote/robot", "hardware", hardware_control=True)
    link.reader.join(timeout=1)
    if matches:
        assert link.error is None
        assert link.hello == hello
    else:
        assert "configuration differs" in link.error
        assert link.hello is None
    assert not process.stdin.getvalue()


def test_simulation_runs_local_physics_with_no_remote_host():
    pytest.importorskip("mujoco")
    model_path = ROOT / "assets/so101/scene_mujoco.xml"
    link = Link("invalid-simulation-host", "/nonexistent/remote/project", "mujoco",
                simulation_model=model_path, keep_simulation_alive=True)
    try:
        assert link.process.args[0] == sys.executable
        assert Path(link.process.args[-1]) == model_path
        assert link.worker_location == "local"
        deadline = time.monotonic() + 5
        while link.snapshot()[1] is None and time.monotonic() < deadline:
            assert link.error is None
            assert link.process.poll() is None
            time.sleep(0.02)
        hello, state, *_ = link.snapshot()
        assert hello["backend"] == "mujoco"
        assert not hello["hardware_motion_enabled"]
        assert not hello["hardware_control"]
        initial = state["qpos"][4]
        target = list(state["target"])
        target[4] += 0.06
        assert link.wait_ack(link.send("target", qpos=target))["ok"]
        while time.monotonic() < deadline:
            state = link.snapshot()[1]
            if abs(state["qpos"][4] - initial) > 0.02:
                break
            time.sleep(0.02)
        assert abs(state["qpos"][4] - initial) > 0.02
        assert state["sim_time"] > 0
        assert link.received_count >= 2
    finally:
        link.close()
    assert link.process.poll() == 0


def test_simulation_accepts_position_steps_without_relative_action_cap():
    assert validate_target([0.9] * 6, [0] * 6, [[-1, 1]] * 6) == [0.9] * 6
