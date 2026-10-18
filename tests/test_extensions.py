"""Alternate robot shapes, controller factories and buses without connected hardware."""

from types import SimpleNamespace

import pytest

from lerobot_arm_control.control import factory
from lerobot_arm_control.control.position_feedback import PositionFeedback
from lerobot_arm_control.core.config import ArmConfig, load_arms
from lerobot_arm_control.robot import backends, connection
from lerobot_arm_control.robot.model import RobotModel


@pytest.fixture(autouse=True)
def isolated_registries(monkeypatch):
    monkeypatch.setattr(backends, '_BACKENDS', dict(backends._BACKENDS))
    monkeypatch.setattr(factory, '_CONTROLLERS', dict(factory._CONTROLLERS))


def two_joint_profile():
    return {'version': 1, 'joints': {j: {'kp': .4, 'ki': .5, 'max_correction_rad': .1,
            'correction_rate_rad_s': .1} for j in ('base', 'wrist')}}


def test_pid_state_is_per_instance_and_not_bound_to_six_joint_names():
    profile = two_joint_profile()
    limits = {'base': (-1., 1.), 'wrist': (-.5, .5)}
    a, b = factory.create_controller(profile, limits), factory.create_controller(profile, limits)
    assert isinstance(a, PositionFeedback) and a.joints == ('base', 'wrist')
    a.target('base', .1, 0, 1/60)
    assert a.integral['base'] > 0 and b.integral['base'] == 0
    a.reset(['base'])
    assert a.integral['base'] == 0
    assert profile == two_joint_profile()
    for _ in range(100):
        assert a.target('wrist', 4., 0, 1/60) == .5
    a.reset()
    assert all(v == 0 for v in a.integral.values())


@pytest.mark.parametrize('limits', [{}, {'base': (0, 0), 'wrist': (-1, 1)},
                                   {'base': (0, float('nan')), 'wrist': (-1, 1)},
                                   {'base': (-1, 1)}])
def test_pid_rejects_bad_or_mismatched_joint_limits(limits):
    with pytest.raises(ValueError):
        factory.create_controller(two_joint_profile(), limits)


class AlternativeController:
    def __init__(self, profile, limits):
        self.profile, self.limits = profile, limits
        self.last = {}

    def reset(self, joints=None):
        for j in self.limits if joints is None else joints:
            self.last.pop(j, None)

    def finish_reference(self, joints):
        self.reset(joints)

    def target(self, joint, reference, measured, elapsed):
        self.last[joint] = reference
        low, high = self.limits[joint]
        return max(low, min(high, reference))

    def status(self):
        return {'last': dict(self.last)}


def test_alternative_controller_is_selected_explicitly_without_mutating_config():
    factory.register_controller('alternative', AlternativeController)
    profile = {**two_joint_profile(), 'controller': 'alternative'}
    c = factory.create_controller(profile, {'base': (-1, 1), 'wrist': (-.5, .5)})
    assert isinstance(c, AlternativeController)
    assert c.target('base', 2, 0, .01) == 1
    c.profile['changed'] = True
    assert 'changed' not in profile
    c.finish_reference(['base'])
    assert c.status() == {'last': {}}
    with pytest.raises(ValueError):
        factory.register_controller('alternative', PositionFeedback)
    with pytest.raises(ValueError, match='unknown'):
        factory.create_controller({'controller': 'missing'}, {})
    factory.register_controller('incomplete', lambda *_: SimpleNamespace(profile={}))
    with pytest.raises(TypeError, match='implement'):
        factory.create_controller({'controller': 'incomplete'}, {})


def test_hardware_adapter_uses_the_same_controller_factory_and_guards_native_limits():
    from fakes import Bus, Calibration

    from lerobot_arm_control.control.manual_control import ManualControl
    from lerobot_arm_control.robot.so101.joints import JOINTS

    factory.register_controller('alternative', AlternativeController)
    profile = {'controller': 'alternative'}
    bus = Bus()
    control = ManualControl(bus, Calibration(), {j: [1000, 3000] for j in JOINTS},
                            feedback_profile=profile)
    assert isinstance(control.position_feedback, AlternativeController)
    assert not bus.writes
    with pytest.raises(ValueError):
        control.target('gripper', .1)
    assert not bus.writes


def test_non_so101_mjcf_metadata_has_its_own_joint_order(tmp_path):
    path = tmp_path / 'other.xml'
    path.write_text('<mujoco><compiler angle="radian"/><worldbody><body>'
                    '<joint name="base" range="-1 1"/><body>'
                    '<joint name="wrist" range="-.5 .5"/></body></body></worldbody></mujoco>')
    model = RobotModel.from_mjcf(path)
    assert model.joints == ('base', 'wrist')
    assert model.limits == {'base': (-1, 1), 'wrist': (-.5, .5)}
    with pytest.raises(ValueError, match='joint map'):
        RobotModel.from_mjcf(path, expected_joints=('wrist', 'base'))
    path.write_text(path.read_text().replace('angle="radian"', 'angle="degree"'))
    with pytest.raises(ValueError, match='radians'):
        RobotModel.from_mjcf(path)


@pytest.mark.parametrize('model_number', [1234, 999])
def test_registered_bus_validates_its_own_identity_and_never_changes_torque(
        tmp_path, model_number):
    events = []
    class Bus:
        is_connected = False
        port_handler = SimpleNamespace(ser=SimpleNamespace(exclusive=False))

        def connect(self, handshake):
            assert handshake is False
            self.is_connected = True
            events.append('connect')

        def ping(self, name, raise_on_error):
            return model_number

        def disconnect(self, disable_torque):
            assert disable_torque is False
            self.is_connected = False
            events.append('disconnect')

    def validate(arm):
        if arm.model != 'other-servo' or arm.baudrate != 115200:
            raise ValueError('other backend settings')
    backends.register_backend('other', backends.MotorBackend(lambda _: Bus(), validate,
                                                           {'other-servo': 1234}))
    arm = ArmConfig('other', str(tmp_path/'bus'), 'follower', 'other-servo', 115200,
                    {'base': 1, 'wrist': 2}, backend='other')
    if model_number == 1234:
        with connection.connected(arm):
            assert events == ['connect']
    else:
        with pytest.raises(RuntimeError, match='1234'):
            with connection.connected(arm):
                pytest.fail('incorrect motor identity accepted')
    assert events == ['connect', 'disconnect']
    with pytest.raises(ValueError):
        backends.register_backend('other', backends.get_backend('other'))


def test_arm_config_dispatches_validation_without_constructing_a_bus(tmp_path):
    calls = []
    def validate(arm):
        calls.append(arm.name)
        if arm.baudrate != 115200:
            raise ValueError('unsupported other baudrate')
    backends.register_backend('other', backends.MotorBackend(
        lambda _: pytest.fail('config opened hardware'), validate, {'other-servo': 1234}))
    path = tmp_path / 'arms.toml'
    path.write_text('[arms.other]\nport="custom-port"\nrole="follower"\n'
                    'model="other-servo"\nbackend="other"\nbaudrate=115200\n'
                    '[arms.other.motors]\nbase=1\nwrist=2\n')
    assert load_arms(path)['other'].backend == 'other'
    assert calls == ['other']
    path.write_text(path.read_text().replace('backend="other"', 'backend="missing"'))
    with pytest.raises(ValueError, match='unknown motor backend'):
        load_arms(path)


def test_native_register_editor_refuses_other_motor_families_before_opening():
    from lerobot_arm_control.robot.feetech import connected_sts3215
    from lerobot_arm_control.workbench.hardware import open_hardware

    arm = ArmConfig('other', 'unused', 'follower', 'other-servo', 115200,
                    {'base': 1}, backend='other')
    with pytest.raises(ValueError, match='STS3215'):
        with open_hardware(arm, [1], 115200):
            pytest.fail('native STS registers used with another backend')
    with pytest.raises(ValueError, match='STS3215'):
        with connected_sts3215(arm):
            pytest.fail('SO101 native tool opened another backend')


@pytest.mark.parametrize('count', [1, 3, 7])
def test_pid_accepts_other_revolute_joint_counts(count):
    limits = {f'axis_{i}': (-1., 1.) for i in range(count)}
    gains = {'kp': .4, 'ki': .5, 'max_correction_rad': .1, 'correction_rate_rad_s': .1}
    c = factory.create_controller({'version': 1, 'joints': {j: gains for j in limits}}, limits)
    for _ in range(10):
        for j in limits:
            assert -.1 <= c.target(j, .05, 0, .01) <= .15
    assert set(c.integral) == set(limits)
    c.reset()
    assert all(v == 0 for v in c.integral.values())


@pytest.mark.parametrize('stream', [{'acceleration': 0, 'velocity_ticks_s': 100},
    {'acceleration': 20, 'velocity_ticks_s': 3001},
    {'acceleration': 20, 'velocity_ticks_s': 100, 'locked_p_coefficients': {'gripper': 32}}])
def test_native_register_stream_validation_remains_on_the_feetech_adapter(stream):
    from lerobot_arm_control.robot.feetech import validate_position_stream

    with pytest.raises(ValueError):
        validate_position_stream(stream, ('base', 'gripper'))
