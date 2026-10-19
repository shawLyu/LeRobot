"""Three calibration categories preserve their hardware boundaries and measurements."""

import json
import sys
from types import SimpleNamespace

import pytest

from lerobot_arm_control import cli
from lerobot_arm_control.calibration import angles, table


def test_calibration_help_contains_only_three_categories(capsys):
    with pytest.raises(SystemExit) as caught:
        cli.main(['calibrate', '--help'])
    assert caught.value.code == 0
    help_text = capsys.readouterr().out
    assert '{camera,table,angles}' in help_text
    assert 'sample-markers' not in help_text and 'height' not in help_text


def test_angles_opens_selected_arm_with_capture_mode_and_no_motion(monkeypatch):
    from lerobot_arm_control.runtime import realtime_client

    seen = []
    monkeypatch.setattr(sys, 'argv', ['angles', '--arm', 'arm1', '--duration', '2'])
    monkeypatch.setattr(realtime_client, 'main', lambda: seen.extend(sys.argv[1:]))
    angles.main()
    assert seen[:5] == ['--backend', 'hardware', '--calibration-mode', '--arm', 'arm1']
    assert '--duration' in seen
    assert not {'--hardware-control', '--verify-gripper', '--circle'} & set(seen)


@pytest.mark.parametrize('arguments', [
    ['--hardware-control'], ['--backend', 'mujoco'], ['--verify-gripper'], ['--circle']])
def test_angle_entry_does_not_accept_motion_modes(monkeypatch, arguments):
    monkeypatch.setattr(sys, 'argv', ['angles', *arguments])
    with pytest.raises(SystemExit) as caught:
        angles.main()
    assert caught.value.code == 2


@pytest.mark.parametrize('argument', ['--hardware-control', '--smoke-test', '--verify-gripper',
                                      '--circle', '--backend=mujoco'])
def test_capture_mode_cannot_be_combined_with_execution(monkeypatch, argument):
    from lerobot_arm_control.runtime import realtime_client

    monkeypatch.setattr(sys, 'argv', ['view', '--calibration-mode', argument])
    monkeypatch.setattr(realtime_client, 'run', lambda _: pytest.fail('unsafe mode executed'))
    with pytest.raises(SystemExit) as caught:
        realtime_client.main()
    assert caught.value.code == 2


@pytest.mark.parametrize('action,module', list(angles.REFINEMENTS.items()))
def test_precision_refinements_are_under_angles_and_keep_their_gates(monkeypatch, action, module):
    seen = []
    monkeypatch.setattr(sys, 'argv', ['angles', action, '--help'])
    def load(name):
        assert name == 'lerobot_arm_control.calibration.'+module
        return SimpleNamespace(main=lambda: seen.extend(sys.argv[1:]))
    monkeypatch.setattr(angles.importlib, 'import_module', load)
    angles.main()
    assert seen == ['--help']


def test_live_table_registration_uses_camera_only_and_recollects_both_bases(monkeypatch, tmp_path):
    from lerobot_arm_control.runtime import tabletop_client

    seen = []
    monkeypatch.setattr(sys, 'argv', ['table', '--output', str(tmp_path/'session'), '--headless'])
    monkeypatch.setattr(tabletop_client, 'run', lambda args: seen.append(args))
    table.main()
    args = seen[0]
    assert args.camera_only and args.recalibrate_base
    assert not args.recalibrate_table and not args.hardware_control and not args.push_t


def test_table_reference_reset_is_explicit(monkeypatch, tmp_path):
    from lerobot_arm_control.runtime import tabletop_client

    seen = []
    monkeypatch.setattr(sys, 'argv',
                        ['table', '--reset-table', '--output', str(tmp_path/'session')])
    monkeypatch.setattr(tabletop_client, 'run', lambda args: seen.append(args))
    table.main()
    assert seen[0].recalibrate_table and seen[0].camera_only


def test_offline_table_fit_preserves_quality_checks_and_never_opens_a_camera(monkeypatch, tmp_path):
    from lerobot_arm_control.calibration import table_reference_calibration as fitting
    from lerobot_arm_control.perception import tabletop_perception

    source, output = tmp_path/'observations', tmp_path/'result'
    seen = []
    result = {'quality': {'passed': True}, 'secondary_layouts': {'test': []}}
    def fit(path, config):
        seen.append((path, config))
        return result
    monkeypatch.setattr(fitting, 'fit_capture', fit)
    monkeypatch.setattr(tabletop_perception, 'read_config', lambda _: {'verified': True})
    monkeypatch.setattr(sys, 'argv', ['table', '--capture', str(source), '--output', str(output)])
    table.main()
    assert seen == [(source, {'verified': True})]
    assert json.loads((output/'table-reference.json').read_text()) == result


@pytest.mark.parametrize('arguments', [['--capture', 'data', '--reset-table'],
                                      ['--hardware-control'], ['--push-t'], ['--fps', '0']])
def test_table_entry_rejects_mixed_or_motion_operations(monkeypatch, arguments):
    monkeypatch.setattr(sys, 'argv', ['table', *arguments])
    with pytest.raises(SystemExit) as caught:
        table.main()
    assert caught.value.code == 2


def test_table_calibration_panel_separates_pending_from_measured_bases():
    config = {'marker_id': 1, 'robot': {'arm': 'follower', 'enabled': True},
              'additional_robots': [{'arm': 'arm1', 'enabled': True}]}
    state = {'marker_visible': True, 'camera_position': [0., 0., .7],
             'base_registrations': {
                 'follower': {'status': 'fixed', 'calibrated': True,
                              'position': [.1, .2, .0024], 'quaternion': [1, 0, 0, 0]},
                 'arm1': {'status': 'tracked', 'calibrated': False,
                          'fixed_calibration_samples': 8, 'fixed_calibration_required': 20}}}
    panel = table.calibration_text(config, state, .1)
    assert 'Camera in table frame (mm): 0.0, 0.0, 700.0' in panel
    assert 'follower: fixed' in panel and '100.0, 200.0, 2.4' in panel
    assert 'arm1: tracked | samples 8/20' in panel
    assert 'Joint angles are not sampled' in panel
    assert 'LOST / STALE' in table.calibration_text(config, state, 1.)
    config['additional_robots'][0]['markers'] = [{'dictionary': 'DICT_4X4_50', 'id': 6}]
    state['base_registrations']['arm1']['status'] = 'markers_lost'
    panel = table.calibration_text(config, state, .1)
    assert 'Primary anchor hidden: base calibration samples paused' in panel
    assert 'No verified base marker: DICT_4X4_50 ID 6' in panel
