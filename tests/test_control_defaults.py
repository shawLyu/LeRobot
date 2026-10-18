import io
import json
from unittest.mock import Mock

import pytest

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, POSITION_FEEDBACK


def test_manual_reset_circle_and_push_use_shared_defaults_without_overrides():
    from fakes import Bus, Calibration

    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
    from lerobot_arm_control.tasks.push_t.push_t_motor import ENTRY_PROFILE, PushMotor
    from lerobot_arm_control.transport.realtime import JOINTS

    limits = {joint: [1000, 3000] for joint in JOINTS}
    bus = Bus()
    bus.registers['shoulder_lift']['I_Coefficient'] = 4
    bus.registers['elbow_flex']['I_Coefficient'] = 8
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
    assert bus.read('I_Coefficient', 'shoulder_lift') == 0
    assert bus.read('I_Coefficient', 'elbow_flex') == 0
    manual.close()
    assert set(bus.sync_read("Acceleration").values()) == {0}
    assert set(bus.sync_read("Goal_Velocity").values()) == {0}
    assert set(bus.sync_read("Torque_Enable").values()) == {1}
    assert set(bus.sync_read('P_Coefficient').values()) == {16}
    assert bus.read('I_Coefficient', 'shoulder_lift') == 4
    assert bus.read('I_Coefficient', 'elbow_flex') == 8


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
