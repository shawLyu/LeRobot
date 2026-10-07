import hashlib
import io
import json
from dataclasses import fields
from unittest.mock import Mock

import pytest

from arm_control.control_defaults import (
    CONTROL_HZ,
    DEFAULTS,
    POSITION_FEEDBACK,
    READ_RETRIES,
    TRAJECTORY,
)


def test_profile_has_no_custom_relative_target_limit():
    assert CONTROL_HZ == 60
    assert READ_RETRIES == 2
    assert DEFAULTS["max_relative_target"] is None


def test_recorded_defaults_match_installed_official_lerobot():
    pytest.importorskip("lerobot")
    from lerobot.robots.so_follower.config_so_follower import SOFollowerConfig
    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

    official = SOFollowerConfig(port="unused")
    for key in ("max_relative_target", "num_read_retries", "position_p_coefficient",
                "position_i_coefficient", "position_d_coefficient"):
        assert DEFAULTS[key] == getattr(official, key)
    assert CONTROL_HZ == next(field.default for field in fields(TeleoperateConfig)
                              if field.name == "fps")


def test_shared_configuration_preserves_latest_measured_values():
    # Fingerprint of the accepted V14 feedback values before this cleanup.
    assert hashlib.sha256(json.dumps(POSITION_FEEDBACK, sort_keys=True).encode()).hexdigest() == (
        "62effcc919c99dbc0e7a32f5af3dd465a05f1ced419d0e065a4aa1e34af77f95")
    assert TRAJECTORY == {"version": 1, "entry_speed_rad_s": .5, "min_entry_seconds": .5,
                          "ramp_seconds": 1., "control_hz": 60,
                          "reset_speed_rad_s": .5, "period_seconds": 4.}
    assert TRAJECTORY["control_hz"] == CONTROL_HZ


def test_manual_reset_circle_and_push_use_shared_defaults_without_overrides():
    from test_push_t_remote import Calibration, FakeBus

    from arm_control.circle_trajectory import Circle
    from arm_control.manual_control import ManualControl
    from arm_control.push_t_motor import ENTRY_PROFILE, PushMotor
    from arm_control.realtime import JOINTS

    limits = {joint: [1000, 3000] for joint in JOINTS}
    bus = FakeBus(JOINTS)
    manual = ManualControl(bus, Calibration(), [[-1, 1]] * 6, limits)
    push = PushMotor(FakeBus(JOINTS), Calibration(), [[-1, 1]] * 6, limits)
    assert manual.position_feedback.profile == push.control.position_feedback.profile
    assert manual.position_feedback.profile == POSITION_FEEDBACK
    assert manual.parking.planned_speed == push.control.parking.planned_speed == .5
    circle = Circle()
    assert circle.entry_speed == ENTRY_PROFILE["entry_speed_rad_s"] == .5
    assert circle.period == 4
    # The actual default enable path applies V14 and restores the previous RAM values.
    manual.enable_all()
    assert set(bus.sync_read("Acceleration").values()) == {20}
    assert set(bus.sync_read("Goal_Velocity").values()) == {600}
    manual.close()
    assert set(bus.sync_read("Acceleration").values()) == {0}
    assert set(bus.sync_read("Goal_Velocity").values()) == {0}
    assert set(bus.sync_read("Torque_Enable").values()) == {1}


def test_controller_instances_do_not_share_mutable_compensation_parameters():
    from arm_control.position_feedback import PositionFeedback
    from arm_control.realtime import JOINTS

    limits = {joint: [-1, 1] for joint in JOINTS}
    first = PositionFeedback(POSITION_FEEDBACK, limits)
    second = PositionFeedback(POSITION_FEEDBACK, limits)
    first.profile["joints"]["elbow_flex"]["kp"] = 0
    assert second.profile["joints"]["elbow_flex"]["kp"] == 2
    assert POSITION_FEEDBACK["joints"]["elbow_flex"]["kp"] == 2


@pytest.mark.parametrize("matches", [False, True])
def test_push_t_connection_checks_shared_configuration(monkeypatch, tmp_path, matches):
    import base64

    from arm_control.control_defaults import CONTROL_CONFIG_SHA256
    from arm_control.tabletop_client import TableLink

    hello = {"version": 1, "type": "hello", "config": {},
             "tag_png": base64.b64encode(b"test-marker").decode(),
             "control_config_sha256": CONTROL_CONFIG_SHA256 if matches else "outdated"}
    process = Mock(stdin=io.StringIO(), stdout=io.StringIO(json.dumps(hello) + "\n"))
    monkeypatch.setattr("arm_control.tabletop_client.subprocess.Popen", Mock(return_value=process))
    link = TableLink("unused", "/unused", tmp_path, server_args=("--hardware-control",))
    link.reader.join(timeout=1)
    if matches:
        assert link.error is None
        assert link.hello["control_config_sha256"] == CONTROL_CONFIG_SHA256
    else:
        assert "configuration differs" in link.error
        assert link.hello is None
    assert not process.stdin.getvalue()


@pytest.mark.parametrize("module_name", ["realtime_client", "realtime_server"])
@pytest.mark.parametrize("option", ["--baseline", "--tracking-profile", "--circle-profile",
                                    "--circle-period"])
def test_retired_cli_options_fail_before_starting_a_worker(monkeypatch, module_name, option):
    import importlib

    module = importlib.import_module("arm_control." + module_name)
    monkeypatch.setattr("sys.argv", [module_name, option])
    startup = Mock()
    monkeypatch.setattr(module, "run" if module_name == "realtime_client" else "serve", startup)
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    startup.assert_not_called()


@pytest.mark.parametrize("constructor,kwargs", [
    ("ManualControl", {"tracking_profile": None}),
    ("PushMotor", {"tracking_profile": None}),
    ("Circle", {"profile_path": "obsolete.json"}),
    ("Circle", {"period": 6}),
    ("Link", {"baseline": True}),
    ("Link", {"tracking_profile": "obsolete.json"}),
    ("Link", {"circle_profile": "obsolete.json"}),
    ("Link", {"circle_period": 6}),
])
def test_retired_python_parameters_cannot_bypass_shared_config(constructor, kwargs):
    from arm_control.circle_trajectory import Circle
    from arm_control.manual_control import ManualControl
    from arm_control.push_t_motor import PushMotor
    from arm_control.realtime_client import Link

    constructors = {"ManualControl": (ManualControl, [None] * 4),
                    "PushMotor": (PushMotor, [None] * 4),
                    "Circle": (Circle, []), "Link": (Link, ["unused", "/unused", "hardware"])}
    factory, args = constructors[constructor]
    with pytest.raises(TypeError, match="unexpected keyword"):
        factory(*args, **kwargs)
