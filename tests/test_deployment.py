"""Exercise deployment ordering without contacting a robot or transferring files."""

import json
import os
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def commands(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "commands.jsonl"
    program = """#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
name = Path(sys.argv[0]).name
with open(os.environ['DEPLOY_TEST_LOG'], 'a') as output:
    output.write(json.dumps([name, sys.argv[1:]]) + '\\n')
if os.environ.get('DEPLOY_TEST_FAIL') == name:
    sys.exit(23)
"""
    for name in ("ssh", "rsync"):
        executable = bin_dir / name
        executable.write_text(program)
        executable.chmod(0o755)
    environment = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ["PATH"],
                       DEPLOY_TEST_LOG=str(log), LEROBOT_HOST="test-robot",
                       LEROBOT_REMOTE_ROOT="/home/test/project")
    environment.pop("DEPLOY_TEST_FAIL", None)
    return environment, log


def run_deploy(commands, *arguments):
    environment, log = commands
    result = subprocess.run(["sh", str(ROOT / "scripts/deploy_code.sh"), *arguments],
                            env=environment, capture_output=True, text=True)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return result, calls


def test_dry_run_only_previews_transfers_and_uses_unassigned_device_template(commands):
    result, calls = run_deploy(commands, "--dry-run")
    assert result.returncode == 0, result.stderr
    assert calls
    assert all(name == "rsync" and "--dry-run" in arguments for name, arguments in calls)
    arms = [args for _, args in calls if args[-1].endswith("/configs/arms.toml")]
    assert len(arms) == 1
    assert arms[0][-2] == str(ROOT / "configs/examples/arms.toml")
    assert "--ignore-existing" in arms[0]


def test_success_archives_before_transfers_and_retires_only_afterwards(commands):
    result, calls = run_deploy(commands)
    assert result.returncode == 0, result.stderr
    assert calls[0][0] == calls[-1][0] == "ssh"
    assert "set -eu" in calls[0][1][-1]
    assert "tar --exclude=__pycache__" in calls[0][1][-1]
    for name, args in calls:
        if name == "ssh":
            syntax = subprocess.run(["sh", "-n", "-c", args[-1]], capture_output=True, text=True)
            assert syntax.returncode == 0, syntax.stderr
    assert all(name == "rsync" for name, _ in calls[1:-1])
    assert "retired-interfaces" in calls[-1][1][-1]
    assert "mv " in calls[-1][1][-1]


@pytest.mark.parametrize("failed_command", ["ssh", "rsync"])
def test_failure_aborts_before_retiring_old_runtime(commands, failed_command):
    commands[0]["DEPLOY_TEST_FAIL"] = failed_command
    result, calls = run_deploy(commands)
    assert result.returncode == 23
    assert not any("retired-interfaces" in " ".join(args) for _, args in calls)
    if failed_command == "ssh":
        assert len(calls) == 1


@pytest.mark.parametrize("root", ["/home/test/../../etc", "/home/test/./project",
                                  "/home/test//project", "/tmp/project", "/home/test/a;cmd"])
def test_invalid_remote_root_is_rejected_before_any_command(commands, root):
    commands[0]["LEROBOT_REMOTE_ROOT"] = root
    result, calls = run_deploy(commands)
    assert result.returncode == 2
    assert not calls
