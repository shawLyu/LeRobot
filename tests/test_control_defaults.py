import io
import json
import math
from dataclasses import fields
from unittest.mock import Mock

import pytest

from lerobot_arm_control.control.control_defaults import (
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
    assert POSITION_FEEDBACK['name'] == 'shared-joint-pid-v1'
    assert all(gains['kd'] > 0 for gains in POSITION_FEEDBACK['joints'].values())
    assert 'locked_p_coefficients' not in POSITION_FEEDBACK['native_stream']
    assert POSITION_FEEDBACK['native_stream']['locked_i_coefficients'] == {
        'shoulder_lift': 0, 'elbow_flex': 0}
    assert TRAJECTORY == {"version": 1, "entry_speed_rad_s": .8, "min_entry_seconds": .35,
                          "ramp_seconds": 1., "control_hz": 60,
                          "reset_speed_rad_s": .5, "period_seconds": 16.,
                          'max_circle_angular_speed_deg_s': 120.,
                          'max_circle_joint_speed_rad_s': math.radians(90),
                          'max_circle_joint_acceleration_rad_s2': math.radians(300)}
    assert TRAJECTORY["control_hz"] == CONTROL_HZ


def test_manual_reset_circle_and_push_use_shared_defaults_without_overrides():
    from fakes import Bus, Calibration

    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
    from lerobot_arm_control.tasks.push_t.push_t_motor import ENTRY_PROFILE, PushMotor
    from lerobot_arm_control.transport.realtime import JOINTS

    limits = {joint: [1000, 3000] for joint in JOINTS}
    bus = Bus()
    manual = ManualControl(bus, Calibration(), limits)
    push = PushMotor(Bus(), Calibration(), [[-1, 1]] * 6, limits)
    assert manual.position_feedback.profile == push.control.position_feedback.profile
    assert manual.position_feedback.profile == POSITION_FEEDBACK
    assert manual.parking.planned_speed == push.control.parking.planned_speed == .5
    circle = Circle()
    assert circle.entry_speed == ENTRY_PROFILE["entry_speed_rad_s"] == .8
    assert circle.period == 16
    # The shared enable path restores the previous RAM values.
    manual.enable_all()
    assert set(bus.sync_read("Acceleration").values()) == {
        POSITION_FEEDBACK['native_stream']['acceleration']}
    assert set(bus.sync_read("Goal_Velocity").values()) == {1000}
    assert set(bus.sync_read('P_Coefficient').values()) == {16}
    manual.close()
    assert set(bus.sync_read("Acceleration").values()) == {0}
    assert set(bus.sync_read("Goal_Velocity").values()) == {0}
    assert set(bus.sync_read("Torque_Enable").values()) == {1}
    assert set(bus.sync_read('P_Coefficient').values()) == {16}


def test_controller_instances_do_not_share_mutable_compensation_parameters():
    from lerobot_arm_control.control.position_feedback import PositionFeedback
    from lerobot_arm_control.transport.realtime import JOINTS

    limits = {joint: [-1, 1] for joint in JOINTS}
    first = PositionFeedback(POSITION_FEEDBACK, limits)
    second = PositionFeedback(POSITION_FEEDBACK, limits)
    first.profile["joints"]["elbow_flex"]["kp"] = 0
    assert second.profile["joints"]["elbow_flex"]["kp"] == .25
    assert POSITION_FEEDBACK["joints"]["elbow_flex"]["kp"] == .25


@pytest.mark.parametrize("matches", [False, True])
def test_push_t_connection_checks_shared_configuration(monkeypatch, tmp_path, matches):
    import base64

    from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
    from lerobot_arm_control.runtime.tabletop_client import TableLink

    hello = {"version": 1, "type": "hello", "config": {},
             "tag_png": base64.b64encode(b"test-marker").decode(),
             "control_config_sha256": CONTROL_CONFIG_SHA256 if matches else "outdated"}
    process = Mock(stdin=io.StringIO(), stdout=io.StringIO(json.dumps(hello) + "\n"))
    monkeypatch.setattr("lerobot_arm_control.runtime.tabletop_client.subprocess.Popen",
        Mock(return_value=process))
    link = TableLink("unused", "/unused", tmp_path, server_args=("--hardware-control",))
    link.reader.join(timeout=1)
    if matches:
        assert link.error is None
        assert link.hello["control_config_sha256"] == CONTROL_CONFIG_SHA256
    else:
        assert "configuration differs" in link.error
        assert link.hello is None
    assert not process.stdin.getvalue()
