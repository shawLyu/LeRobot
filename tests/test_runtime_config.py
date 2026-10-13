"""Connection settings follow the selected workspace and explicit overrides."""

import argparse
import json

import pytest

from lerobot_arm_control.core import runtime_config


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    directory = tmp_path/'robot workspace'
    (directory/'configs').mkdir(parents=True)
    (directory/'configs/runtime.json').write_text(json.dumps({
        'version': 1, 'remote': {'host': 'other-robot', 'root': '/home/robot/project space'},
        'local': {'fallback_environment': '../python environment'}}))
    monkeypatch.setenv('LEROBOT_ARM_WORKSPACE', str(directory))
    monkeypatch.delenv('LEROBOT_HOST', raising=False)
    monkeypatch.delenv('LEROBOT_REMOTE_ROOT', raising=False)
    return directory


def test_cli_environment_and_workspace_precedence(workspace, monkeypatch):
    parser = argparse.ArgumentParser()
    runtime_config.add_remote_arguments(parser)
    args = parser.parse_args([])
    assert (args.host, args.remote_root) == ('other-robot', '/home/robot/project space')
    monkeypatch.setenv('LEROBOT_HOST', 'environment-robot')
    monkeypatch.setenv('LEROBOT_REMOTE_ROOT', '/home/environment/project')
    parser = argparse.ArgumentParser()
    runtime_config.add_remote_arguments(parser)
    args = parser.parse_args(['--host', 'cli-robot', '--remote-root', '/home/cli/project'])
    assert (args.host, args.remote_root) == ('cli-robot', '/home/cli/project')
    assert runtime_config.remote_runtime().host == 'environment-robot'
    assert runtime_config.remote_runtime('explicit-robot', '/home/explicit/project').root == \
        '/home/explicit/project'


def test_missing_workspace_config_never_borrows_another_robots_defaults(workspace):
    (workspace/'configs/runtime.json').unlink()
    with pytest.raises(FileNotFoundError, match='runtime.json'):
        runtime_config.remote_runtime()


@pytest.mark.parametrize('host,root', [('-oProxyCommand=x', '/home/robot'),
                                     ('robot host', '/home/robot'), ('robot', 'relative')])
def test_invalid_connection_settings_rejected(workspace, host, root):
    with pytest.raises(ValueError):
        runtime_config.remote_runtime(host, root)


def test_launcher_environment_resolves_against_workspace(workspace, monkeypatch, capsys):
    monkeypatch.setattr('sys.argv', ['runtime_config', 'local.fallback_environment'])
    runtime_config.main()
    assert capsys.readouterr().out.strip() == str((workspace/'../python environment').resolve())
