"""Physical following is the default; preview modes never start real control."""

import pytest


@pytest.mark.parametrize('arguments, execute', [([], True), (['--execute'], True),
    (['--simulate'], False), (['--source', 'synthetic'], False),
    (['--source', 'synthetic', '--auto', '--headless'], False),
    (['--simulate', '--headless'], False)])
def test_real_following_is_default_and_simulation_is_explicit(monkeypatch, arguments, execute):
    import sys

    from lerobot_arm_control.runtime import follow_client

    captured = []
    monkeypatch.setattr(sys, 'argv', ['follow', *arguments])
    monkeypatch.setattr(follow_client, 'run', captured.append)
    follow_client.main()
    assert captured[0].execute is execute


@pytest.mark.parametrize('arguments', [['--execute', '--simulate'], ['--headless'],
                                     ['--source', 'synthetic', '--execute']])
def test_invalid_follow_modes_fail_before_starting_a_worker(monkeypatch, arguments):
    import sys

    from lerobot_arm_control.runtime import follow_client

    monkeypatch.setattr(sys, 'argv', ['follow', *arguments])
    monkeypatch.setattr(follow_client, 'run', lambda args: pytest.fail('must not start hardware'))
    with pytest.raises(SystemExit) as error:
        follow_client.main()
    assert error.value.code == 2


@pytest.mark.parametrize('problem', ['delay', 'worker_exit', 'send_failure',
                                     'auto_start', 'start_held', 'space', 'unfocused'])
def test_interrupted_feedback_keeps_window_open_and_never_automatically_resumes(
        monkeypatch, tmp_path, problem):
    import json
    import time
    from types import SimpleNamespace

    from lerobot_arm_control.control.follow import load_follow_config
    from lerobot_arm_control.core.paths import workspace_root
    from lerobot_arm_control.runtime import follow_client
    from lerobot_arm_control.visualization import control_viewer

    commands, viewers, closed = [], [], []
    config = load_follow_config()
    packet = {'leader_qpos': [0]*6, 'qpos': [0]*6, 'target_qpos': [0]*6,
              'following': False, 'stage': 'held', 'notice': 'Held'}

    class Link:
        latest = packet
        process = SimpleNamespace(poll=lambda: None)

        @property
        def error(self):
            return 'serial read failed' if problem == 'worker_exit' and viewers else None

        def __init__(self, *args, **kwargs):
            assert kwargs['hardware_control'] is True

        def snapshot(self):
            age = (.4 if problem in ('delay', 'send_failure')
                   and viewers and viewers[0].frames < 2 else 0)
            return ({'config': config, 'leader': {'mode': 'independent'}},
                    packet, time.monotonic()-age, 1, None)

        def send(self, operation):
            commands.append(operation)
            if problem == 'send_failure':
                raise BrokenPipeError('worker stdin closed')

        def close(self):
            closed.append(True)

    class Viewer:
        def __init__(self, model, data, callback, *args, **kwargs):
            self.callback, self.frames = callback, 0
            self.cam = SimpleNamespace(lookat=[0]*3)
            viewers.append(self)

        def is_running(self):
            return self.frames < 3

        def is_focused(self):
            return problem != 'unfocused'

        def set_texts(self, texts):
            self.text = texts[0][2]

        def sync(self, path):
            if self.frames == 0 and problem in ('delay', 'worker_exit', 'send_failure'):
                self.callback(69)  # E while feedback is delayed must not start motors.
            if self.frames == 0 and problem == 'space':
                self.callback(32)
            self.frames += 1

        def close(self):
            pass

    monkeypatch.setattr(follow_client, 'Link', Link)
    monkeypatch.setattr(control_viewer, 'ControlViewer', Viewer)
    args = SimpleNamespace(log=tmp_path/'follow.json', source='hardware', execute=True,
                           snapshot=workspace_root()/'data/tabletop/current-scene',
                           host='unused', remote_root='unused', headless=False,
                           duration=None, screenshot=None, auto=False,
                           start_held=problem == 'start_held')
    follow_client.run(args)
    result = json.loads(args.log.read_text())
    assert len(result['frames']) == 3
    assert result['exit_reason'] == 'window_closed'
    assert commands.count('resume') == (1 if problem == 'auto_start' else 0)
    assert closed == [True]
    if problem in ('auto_start', 'start_held', 'space', 'unfocused'):
        assert result['error'] is None
        return
    if problem == 'delay':
        assert commands.count('hold') == 1
        assert 'Feedback recovered; held' in result['frames'][-1]['notice']
        assert result['error'] is None
    else:
        assert result['frames'][-1]['stage'] == 'disconnected'
        assert result['error']


