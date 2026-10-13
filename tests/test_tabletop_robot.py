"""Hardware-free remote tests enforce calibration identity and read-only operation."""

import json
from types import SimpleNamespace

import pytest

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.runtime.tabletop_robot import (
    JOINTS,
    ROOT,
    calibration_identity,
)


class ReadOnlyBus:
    def __init__(self, identity):
        self.identity = identity
        self.reads = []

    def sync_read(self, register, normalize):
        assert normalize is False
        self.reads.append(register)
        if register == "Min_Position_Limit":
            return {k: v[0] for k, v in self.identity["raw_limits"].items()}
        if register == "Max_Position_Limit":
            return {k: v[1] for k, v in self.identity["raw_limits"].items()}
        return self.identity[
            {"Homing_Offset": "homing_offsets", "Phase": "phase", "Operating_Mode": "mode"}[
                register
            ]
        ]


def test_missing_white_mapping_is_explicit_and_does_not_open_a_bus(tmp_path, monkeypatch):
    from lerobot_arm_control.runtime import tabletop_robot

    monkeypatch.setattr(tabletop_robot, 'ROOT', tmp_path)
    config = {'additional_robots': [{'arm': 'arm1', 'enabled': True,
                                     'joint_calibration': 'data/calibration/arm1_mujoco.json'}]}
    feedback = tabletop_robot.start_additional_feedback(config)
    state = feedback['arm1'].snapshot()
    assert state['status'] == 'uncalibrated' and state['read_only'] is True
    assert 'qpos' not in state
    assert feedback['arm1'].worker.ident is None
    feedback['arm1'].close()
    assert not tabletop_robot.start_additional_feedback(config, camera_only=True)


def test_hardware_identity_reuses_all_six_existing_fits():
    path = ROOT / "data/calibration/follower_mujoco.json"
    saved = json.loads(path.read_text())
    identity = saved["fingerprint"]
    bus = ReadOnlyBus(identity)
    arm = SimpleNamespace(port=identity["port"], motors=identity["motors"])
    actual = calibration_identity(bus, arm, ROOT / "assets/so101/so101.xml")
    assert actual == identity
    assert bus.reads == [
        "Min_Position_Limit",
        "Max_Position_Limit",
        "Homing_Offset",
        "Phase",
        "Operating_Mode",
    ]
    calibration = AngleCalibration(path, actual, use_kinematic=False)
    assert calibration.count() == 6
    raw = {k: saved["points"][k]["30"]["raw"] for k in JOINTS}
    assert calibration.angles(raw, [0] * 6) == pytest.approx([0.5235987756] * 6)


def test_changed_eeprom_is_rejected():
    path = ROOT / "data/calibration/follower_mujoco.json"
    identity = json.loads(path.read_text())["fingerprint"]
    identity["homing_offsets"]["gripper"] += 1
    with pytest.raises(ValueError, match="identity/EEPROM changed"):
        AngleCalibration(path, identity)


@pytest.mark.parametrize('own_mapping', [True, 'wrong_identity'])
def test_white_feedback_validates_its_independent_mapping(
        tmp_path, monkeypatch, own_mapping):
    from contextlib import contextmanager
    from copy import deepcopy

    from lerobot_arm_control.runtime import tabletop_robot

    source_path = ROOT/'data/calibration/follower_mujoco.json'
    saved = json.loads(source_path.read_text())
    source = AngleCalibration(source_path, saved['fingerprint'], use_kinematic=False)
    identity = deepcopy(saved['fingerprint'])
    identity['port'] = '/dev/white-arm'
    identity['homing_offsets']['shoulder_pan'] += 300
    identity['phase']['shoulder_pan'] = 76
    own_path = tmp_path/'arm1.json'
    independent = deepcopy(saved)
    independent['fingerprint'] = identity if own_mapping is True else saved['fingerprint']
    for points in independent['points'].values():
        for point in points.values():
            point['raw'] += 100
    own_path.write_text(json.dumps(independent))
    monkeypatch.setattr(tabletop_robot, 'ROOT', tmp_path)
    monkeypatch.setattr(tabletop_robot, 'load_arms', lambda: {'arm1': SimpleNamespace(
        port=identity['port'], motors=identity['motors'])})
    raw = {joint: saved['points'][joint]['0']['raw']+110 for joint in JOINTS}

    class WhiteBus(ReadOnlyBus):
        def sync_read(self, register, normalize):
            if register == 'Present_Position':
                feedback['arm1'].stopped.set()
                return raw
            if register in ('Torque_Enable', 'Status', 'Present_Voltage', 'Present_Temperature'):
                return dict.fromkeys(JOINTS, 0)
            return super().sync_read(register, normalize)

    @contextmanager
    def connection(arm):
        yield WhiteBus(identity)

    monkeypatch.setattr(tabletop_robot, 'connected', connection)
    monkeypatch.setattr(tabletop_robot.RobotFeedback, 'start', lambda self: None)
    config = {'additional_robots': [{
        'enabled': True, 'arm': 'arm1', 'joint_calibration': 'arm1.json'}]}
    feedback = tabletop_robot.start_additional_feedback(config)
    feedback['arm1']._run()
    state = feedback['arm1'].snapshot()
    if own_mapping == 'wrong_identity':
        assert state['status'] == 'error' and 'identity/EEPROM' in state['error']
        assert 'qpos' not in state
        return
    actual = feedback['arm1'].metadata_snapshot()
    assert actual['fingerprint'] == identity
    assert state['read_only'] is True and actual['motion_available'] is False
    assert state['calibration_mode'] == 'independent'
    assert state['calibration_source_arm'] == 'arm1'
    assert state['independently_calibrated'] is True
    assert state['qpos'] == pytest.approx(
        [10*source.fit(j)['radians_per_tick'] for j in JOINTS])


@pytest.mark.parametrize('hardware_control', [False, True])
def test_feedback_worker_has_only_reads_and_saves_every_sample(monkeypatch, hardware_control):
    from contextlib import contextmanager

    from lerobot_arm_control.runtime.tabletop_robot import RobotFeedback

    path = ROOT / "data/calibration/follower_mujoco.json"
    saved = json.loads(path.read_text())
    identity = saved["fingerprint"]
    feedback = RobotFeedback(
        {
            "arm": "follower",
            "feedback_hz": 100,
            "joint_calibration": "data/calibration/follower_mujoco.json",
        }, hardware_control=hardware_control,
    )

    class FeedbackBus(ReadOnlyBus):
        samples = 0
        torque_samples = []

        def sync_read(self, register, normalize):
            assert normalize is False
            if register == "Present_Position":
                self.samples += 1
                if self.samples == 3:
                    feedback.stopped.set()
                return {k: saved["points"][k]["0"]["raw"] for k in JOINTS}
            if register in ("Torque_Enable", "Status", "Present_Voltage", "Present_Temperature"):
                if register == 'Torque_Enable':
                    self.torque_samples.append(self.samples)
                return {
                    k: {
                        "Torque_Enable": 0 if hardware_control and self.samples == 1 else 1,
                        "Status": 0,
                        "Present_Voltage": 52,
                        "Present_Temperature": 28,
                    }[register]
                    for k in JOINTS
                }
            return super().sync_read(register, normalize)

    bus = FeedbackBus(identity)
    arm = SimpleNamespace(port=identity["port"], motors=identity["motors"])

    @contextmanager
    def fake_connection(arm):
        yield bus

    monkeypatch.setattr("lerobot_arm_control.runtime.tabletop_robot.load_arms",
        lambda: {"follower": arm})
    monkeypatch.setattr("lerobot_arm_control.runtime.tabletop_robot.connected", fake_connection)
    if hardware_control:
        import gc

        from lerobot_arm_control.control.servo_feedback import FIELDS

        def packed_feedback(bus):
            assert not gc.isenabled()
            raw = bus.sync_read('Present_Position', normalize=False)
            values = {key: bus.sync_read(key, normalize=False)
                      for key in ('Torque_Enable', 'Status', 'Present_Voltage')}
            return {key: raw if key == 'Present_Position' else values.get(
                key, dict.fromkeys(JOINTS, 0)) for key in FIELDS}

        monkeypatch.setattr('lerobot_arm_control.runtime.tabletop_robot.read_control_feedback',
            packed_feedback)
        previously_enabled = gc.isenabled()
    feedback._run()
    assert feedback.snapshot()["status"] == "live"
    assert list(feedback.snapshot()["telemetry"]["Torque_Enable"].values()) == [1] * 6
    assert [s["sequence"] for s in feedback.drain_samples()] == [1, 2, 3]
    metadata = feedback.metadata_snapshot()
    assert metadata['status'] == 'live'
    assert metadata['fingerprint'] == identity
    assert metadata['motion_available'] is hardware_control
    assert feedback.drain_samples() == []
    if hardware_control:
        assert bus.torque_samples == [1, 2, 3]
        assert gc.isenabled() == previously_enabled
        assert feedback.metadata['control_hz'] == 60
        expected = json.loads((ROOT/'configs/control.json').read_text())['position_feedback']
        assert feedback.motor.control.position_feedback.profile == expected
    feedback.sampled_at -= 1
    assert feedback.snapshot()["status"] == "stale"


@pytest.mark.parametrize('hardware_control', [False, True])
def test_failed_bus_initialization_reports_error_without_a_fingerprint(
        monkeypatch, hardware_control):
    from unittest.mock import Mock

    from lerobot_arm_control.runtime.tabletop_robot import RobotFeedback

    feedback = RobotFeedback({'arm': 'follower', 'joint_calibration': 'unused'},
                             hardware_control=hardware_control)
    monkeypatch.setattr('lerobot_arm_control.runtime.tabletop_robot.load_arms',
                        lambda: {'follower': SimpleNamespace()})
    monkeypatch.setattr('lerobot_arm_control.runtime.tabletop_robot.connected',
                        Mock(side_effect=ConnectionError('There is no status packet!')))
    feedback._run()
    metadata = feedback.metadata_snapshot()
    assert feedback.ready.is_set()
    assert metadata['status'] == 'error'
    assert metadata['error'] == 'There is no status packet!'
    assert metadata['motion_available'] is False
    assert 'fingerprint' not in metadata
    assert feedback.motor is None
