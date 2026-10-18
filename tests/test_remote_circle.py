"""A validated finite circle owns its remote clock; a blocked observer owns none."""
import io
import json
import threading
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from fakes import Bus

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256
from lerobot_arm_control.control.manual_control import ManualControl
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.runtime import realtime_server
from lerobot_arm_control.tasks.circle.circle_trajectory import Circle
from lerobot_arm_control.transport.realtime import JOINTS


@pytest.mark.parametrize('case', ['completed', 'feedback_deadline', 'invalid_proof'])
@pytest.mark.parametrize('plan_name,laps', [('table-circle-8cm.json', 1),
                                          ('table-circle90-h34cm.json', 3),
                                          ('table-circle120-h38cm.json', 4)])
def test_finite_circle_is_independent_but_keeps_hardware_guards(monkeypatch, tmp_path, case,
                                                            plan_name, laps):
    path = resource_path('data/calibration/follower_mujoco.json')
    saved = json.loads(path.read_text())
    identity = saved['fingerprint']
    cal = AngleCalibration(path, identity)
    plan_path = resource_path('data/trajectories/'+plan_name)
    circle = Circle(plan_path)
    proof = {'checked': True, 'initial_qpos': circle.points[0],
             'model_sha256': circle.model_hash, 'plan_sha256': circle.plan_hash,
             'control_config_sha256': CONTROL_CONFIG_SHA256}
    if case == 'invalid_proof':
        proof['control_config_sha256'] = 'stale'
    validation = tmp_path/'proof.json'
    validation.write_text(json.dumps(proof))
    bus = Bus()
    for j, q in zip(JOINTS, circle.points[0]):
        r = bus.registers[j]
        fit = cal.fit(j)
        raw = round(q/fit['radians_per_tick']+fit['zero_raw'])
        r.update(Min_Position_Limit=identity['raw_limits'][j][0],
                 Max_Position_Limit=identity['raw_limits'][j][1],
                 Homing_Offset=identity['homing_offsets'][j], Phase=identity['phase'][j],
                 Torque_Enable=1, Present_Position=raw, Goal_Position=raw,
                 I_Coefficient=0, D_Coefficient=32)
    arm = SimpleNamespace(port=identity['port'], motors=identity['motors'])

    @contextmanager
    def connect(arm):
        yield bus

    clock = [0.]
    monkeypatch.setattr(realtime_server.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(realtime_server.time, 'sleep',
                        lambda _: clock.__setitem__(0, clock[0]+1/60))
    monkeypatch.setattr('lerobot_arm_control.core.config.load_arms', lambda: {'follower': arm})
    monkeypatch.setattr('lerobot_arm_control.robot.feetech.connected_sts3215', connect)
    monkeypatch.setattr('lerobot_arm_control.control.manual_control.ManualControl',
                        lambda *a, **k: ManualControl(*a, **k, clock=lambda: clock[0]))
    monkeypatch.setattr('sys.stdin', io.StringIO(''))  # Batch execution has no console heartbeat.

    def feedback(bus):
        if case == 'feedback_deadline' and clock[0] > 3:
            clock[0] += 1  # A complete but late native snapshot must not refresh the watchdog.
        for r in bus.registers.values():
            r['Present_Position'] = r['Goal_Position']
        return {name: bus.sync_read(name) for name in (
            'Present_Position', 'Torque_Enable', 'Acceleration', 'Goal_Position', 'Status')}

    monkeypatch.setattr('lerobot_arm_control.control.servo_feedback.read_control_feedback',
                        feedback)
    release = threading.Event()
    monkeypatch.setattr(realtime_server, 'send', lambda _: release.wait(timeout=2))
    trace = tmp_path/'trace.json'
    args = SimpleNamespace(arm='follower', hardware_control=True, allow_gripper=False,
                           circle=True,
                           circle_plan=plan_path,
                           model=resource_path('assets/so101/so101.xml'), hz=20,
                           standalone_circle_validation=validation, circle_laps=laps,
                           control_log=trace)
    try:
        if case == 'invalid_proof':
            with pytest.raises(ValueError, match='validation'):
                realtime_server.serve(args)
        else:
            realtime_server.serve(args)
    finally:
        release.set()
    if case == 'invalid_proof':
        assert not bus.writes
        return
    record = json.loads(trace.read_text())
    assert record['frames'][-1]['circle']['phase'] == (
        'completed' if case == 'completed' else 'stopped: control feedback deadline: hold')
    assert record['metadata']['autonomous_circle'] is True
    if (case == 'completed'
            and 'transition_acceleration' in circle.feedback_profile['native_stream']):
        acceleration = [f['telemetry']['Acceleration']['shoulder_pan'] for f in record['frames']]
        assert min(acceleration) == 16
        assert max(acceleration) == circle.feedback_profile['native_stream']['acceleration']
        assert acceleration[-1] == 16
    assert all(r['Torque_Enable'] == 1 and r['Acceleration'] == 0 for r in bus.registers.values())
    assert all(r['Goal_Position'] == r['Present_Position'] for r in bus.registers.values())
    assert all(r['D_Coefficient'] == 32 and r['Lock'] == 1 for r in bus.registers.values())
