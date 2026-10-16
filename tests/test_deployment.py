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


def test_success_prepares_destination_and_transfers_without_archiving_old_code(commands):
    result, calls = run_deploy(commands)
    assert result.returncode == 0, result.stderr
    assert calls[0] == ["ssh", ["test-robot", "set -eu\n    mkdir -p '/home/test/project'"]]
    assert all(name == "rsync" for name, _ in calls[1:])
    for _, args in calls[1:]:
        assert "--delete" not in args
    configuration = [args for _, args in calls if "--exclude=/arms.toml" in args]
    assert len(configuration) == 2
    assert all("--exclude=/camera.toml" in args and "--exclude=/tabletop.json" in args
               for args in configuration)
    assert all("--exclude=*.local.*" in args for args in configuration)


@pytest.mark.parametrize("failed_command", ["ssh", "rsync"])
def test_failure_aborts_remaining_transfers(commands, failed_command):
    commands[0]["DEPLOY_TEST_FAIL"] = failed_command
    result, calls = run_deploy(commands)
    assert result.returncode == 23
    assert len(calls) == (1 if failed_command == "ssh" else 2)


@pytest.mark.parametrize("root", ["/home/test/../../etc", "/home/test/./project",
                                  "/home/test//project", "/tmp/project", "/home/test/a;cmd"])
def test_invalid_remote_root_is_rejected_before_any_command(commands, root):
    commands[0]["LEROBOT_REMOTE_ROOT"] = root
    result, calls = run_deploy(commands)
    assert result.returncode == 2
    assert not calls
