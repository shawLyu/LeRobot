"""Recalibrating moved table tags must invalidate every pose in their old frame."""

import json

import pytest

from lerobot_arm_control.runtime import tabletop_server


def test_focus_lock_is_verified_and_unsupported_controls_fail():
    from unittest.mock import Mock

    import cv2

    camera = Mock()
    values = {cv2.CAP_PROP_AUTOFOCUS: 0, cv2.CAP_PROP_FOCUS: 68}
    camera.get.side_effect = values.__getitem__
    camera.set.return_value = True
    config = {'camera_controls': {'autofocus': False, 'focus': 68}}
    assert tabletop_server.configure_focus(camera, config) == {
        'autofocus': False, 'focus': 68, 'verified': True}
    camera.set.return_value = False
    with pytest.raises(RuntimeError, match='confirmed'):
        tabletop_server.configure_focus(camera, config)
    camera.set.return_value = True
    values[cv2.CAP_PROP_AUTOFOCUS] = 1
    with pytest.raises(RuntimeError, match='confirmed'):
        tabletop_server.configure_focus(camera, config)
    with pytest.raises(ValueError, match='autofocus=false'):
        tabletop_server.configure_focus(
            camera, {'camera_controls': {'autofocus': True, 'focus': 68}})


def test_table_reset_archives_both_bases_and_secondary_layout(tmp_path, monkeypatch):
    monkeypatch.setattr(tabletop_server, 'ROOT', tmp_path)
    markers = [
        {'dictionary': 'DICT_4X4_50', 'id': 7},
        {'dictionary': 'DICT_4X4_50', 'id': 6},
    ]
    entries = [{'enabled': True, 'arm': name, 'markers': [marker],
                'fixed_base': {'file': f'data/calibration/{name}_base.json'}}
               for name, marker in zip(('follower', 'arm1'), markers, strict=True)]
    config = {'robot': entries[0], 'additional_robots': entries[1:]}
    files = ['data/calibration/follower_base.json', 'data/calibration/arm1_base.json',
             'data/tabletop/static_map.json']
    for relative in files:
        path = tmp_path/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({'original': relative}))
    tabletop_server.archive_reference(config, include_table=True)
    for relative in files:
        path = tmp_path/relative
        assert not path.exists()
        saved = list(path.parent.glob(path.stem+'-before-*'+path.suffix))
        assert len(saved) == 1
        assert json.loads(saved[0].read_text()) == {'original': relative}


def test_base_reset_preserves_table_map(tmp_path, monkeypatch):
    monkeypatch.setattr(tabletop_server, 'ROOT', tmp_path)
    path = tmp_path/'data/tabletop/static_map.json'
    path.parent.mkdir(parents=True)
    path.write_text('original')
    tabletop_server.archive_reference({'robot': {'enabled': False}})
    assert path.read_text() == 'original'
