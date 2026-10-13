import pytest

from lerobot_arm_control.control.control_defaults import CONTROL
from lerobot_arm_control.control.motion_stall import MotionStall


@pytest.mark.parametrize('direction', [-1, 1])
def test_loaded_joint_stalled_in_either_direction_trips(direction):
    guard = MotionStall(CONTROL['motion_stall'])
    assert not guard.update('wrist_flex', .05*direction, 0, .1*direction, -200, 0)
    assert not guard.update('wrist_flex', .05*direction, 0, .1*direction, -200, .2)
    assert guard.update('wrist_flex', .05*direction, 0, .1*direction, -200, .26)
    guard.reset()
    assert not guard.update('wrist_flex', .05*direction, 0, .1*direction, -200, .27)


@pytest.mark.parametrize('reference,command,load', [(0, .1, 200), (.05, .1, 0),
                                                  (.05, .1, None), (.05, .02, 200)])
def test_normal_hold_or_small_errors_do_not_trip(reference, command, load):
    guard = MotionStall(CONTROL['motion_stall'])
    for index in range(100):
        assert not guard.update('elbow_flex', reference, 0, command, load, index/60)


def test_moving_loaded_joint_resets_stall_timer():
    guard = MotionStall(CONTROL['motion_stall'])
    for index in range(50):
        measured = index*.01
        assert not guard.update('wrist_flex', measured+.05, measured, measured+.1, 200, index/60)


def test_shared_hardware_envelope_is_applied_without_writes_and_cannot_be_widened():
    import json

    from fakes import Bus

    from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.core.paths import workspace_root
    from lerobot_arm_control.transport.realtime import JOINTS

    path = workspace_root()/'data/calibration/follower_mujoco.json'
    saved = json.loads(path.read_text())
    calibration = AngleCalibration(path, saved['fingerprint'])
    bus = Bus()
    control = ManualControl(bus, calibration, saved['fingerprint']['raw_limits'])
    assert control.raw_limits['wrist_flex'] == (862, 2920)
    assert control.raw_limits['gripper'] == (1479, 2982)
    assert all(control.raw_limits[j] == tuple(saved['fingerprint']['raw_limits'][j])
               for j in JOINTS if j not in ('wrist_flex', 'gripper'))
    assert not bus.writes
    with pytest.raises(ValueError):
        control.set_command_limits('wrist_flex', control.rest_limits['wrist_flex'])


def test_loaded_stall_holds_all_axes_and_removes_stale_goal():
    from fakes import Bus, Calibration

    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.transport.realtime import JOINTS

    bus, now = Bus(), [0.]
    control = ManualControl(bus, Calibration(), {j: [1000, 3000] for j in JOINTS},
                            clock=lambda: now[0])
    control.enable_all()
    control.target('wrist_flex', .5)
    for _ in range(60):
        now[0] += 1/60
        control.heartbeat()
        telemetry = {key: bus.sync_read(key) for key in ('Status', 'Torque_Enable')}
        telemetry['Present_Load'] = {'wrist_flex': -200}
        control.step(bus.sync_read('Present_Position'), telemetry)
    assert set(control.stopped) == set(JOINTS)
    assert 'loaded motion stall' in control.last_fault['reason']
    assert bus.sync_read('Goal_Position') == bus.sync_read('Present_Position')
