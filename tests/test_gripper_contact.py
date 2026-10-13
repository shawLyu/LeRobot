import pytest

from lerobot_arm_control.control.control_defaults import CONTROL
from lerobot_arm_control.control.gripper_contact import GripperContact


def test_loaded_stalled_closure_holds_until_user_requests_opening():
    guard = GripperContact(CONTROL['gripper_contact'])
    assert guard.update(.1, .5, .05, 200, 0) is None
    assert guard.update(.1, .501, .05, 200, .1) is None
    assert guard.update(.1, .5, .05, 200, .16) == .5
    # Additional closure and load relaxation cannot restart squeezing.
    assert guard.update(0, .48, -.1, 0, .3) == .5
    assert guard.update(.51, .5, .5, 0, .4) == .5
    assert guard.update(.6, .5, .55, 0, .5) is None
    assert not guard.status()['active']


@pytest.mark.parametrize('reference,command,load', [(.1, .05, 0), (.1, .05, None),
                                                  (.6, .55, 300), (.1, .49, 300)])
def test_motion_without_loaded_closing_error_does_not_trigger(reference, command, load):
    guard = GripperContact(CONTROL['gripper_contact'])
    for index in range(100):
        assert guard.update(reference, .5, command, load, index/60) is None


def test_loaded_jaw_still_moving_does_not_look_like_contact():
    guard = GripperContact(CONTROL['gripper_contact'])
    for index in range(30):
        assert guard.update(0, 1-index*.01, 0, 300, index/60) is None


def test_contact_reset_removes_latch():
    guard = GripperContact(CONTROL['gripper_contact'])
    guard.update(0, .5, 0, 300, 0)
    assert guard.update(0, .5, 0, 300, .2) == .5
    guard.reset()
    assert guard.update(0, .5, 0, 300, .21) is None


def test_shared_hardware_pid_freezes_at_contact_and_releases_on_opening():
    from test_manual_control import Bus, Calibration

    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.transport.realtime import JOINTS

    bus, now = Bus(), [0.]
    control = ManualControl(bus, Calibration(), [(-1, 1)]*6,
                            {j: [1000, 3000] for j in JOINTS}, clock=lambda: now[0])
    control.enable_all()
    control.target('gripper', 0)
    for _ in range(40):
        now[0] += 1/60
        control.heartbeat()
        telemetry = {key: bus.sync_read(key) for key in ('Status', 'Torque_Enable')}
        telemetry['Present_Load'] = {'gripper': 200}
        control.step(bus.sync_read('Present_Position'), telemetry)
    assert control.gripper_contact.status()['active']
    assert bus.registers['gripper']['Goal_Position'] == 2200
    assert control.position_feedback.integral['gripper'] == 0
    assert not control.stopped
    control.target('gripper', .6)
    now[0] += 1/60
    control.step(bus.sync_read('Present_Position'), telemetry)
    assert not control.gripper_contact.status()['active']
    assert bus.registers['gripper']['Goal_Position'] > 2200
