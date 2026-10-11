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


