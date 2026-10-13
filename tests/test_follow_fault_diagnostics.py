from lerobot_arm_control.runtime.follow_server import leader_fault_snapshot, read_leader_status


class FaultBus:
    def __init__(self):
        self.reads = []

    def sync_read(self, register, normalize):
        assert normalize is False
        self.reads.append(register)
        if register == 'Status':
            return {'gripper': 1}
        if register == 'Present_Current':
            raise ConnectionError('missing packet')
        return {'gripper': 52}

    def read(self, register, joint, normalize):
        assert (register, joint, normalize) == ('Status', 'gripper', False)
        raise RuntimeError('[RxPacketError] Input voltage error!')


def test_confirmed_device_error_is_preserved_with_original_driver_message():
    errors = {}
    status = read_leader_status(FaultBus(), errors)
    assert status == {'gripper': 1}
    assert 'Input voltage error' in errors['gripper']


def test_fault_snapshot_is_read_only_and_keeps_partial_evidence_on_read_failure():
    bus = FaultBus()
    result = leader_fault_snapshot(bus, {'gripper': 1}, {'gripper': 'voltage'})
    assert result['status'] == {'gripper': 1}
    assert result['packet_errors'] == {'gripper': 'voltage'}
    assert result['registers']['Present_Voltage'] == {'gripper': 52}
    assert result['registers']['Present_Current'] == {'error': 'missing packet'}
    assert bus.reads[-1] == 'Torque_Enable'
