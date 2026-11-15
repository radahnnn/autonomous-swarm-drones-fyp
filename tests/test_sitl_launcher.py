"""Ownership-based SITL launcher process cleanup tests (UNIT)."""
import inspect
import json
import signal

import pytest

from sitl import sitl_launcher
from sitl.sitl_launcher import SitlLauncher


def test_sitl_launcher_does_not_use_broad_pkill():
    src = inspect.getsource(sitl_launcher)
    assert "pkill" not in src


def test_kill_existing_sitl_only_terminates_owned_processes(tmp_path, monkeypatch):
    launcher = SitlLauncher(1, use_gazebo=False, force_restart=True)
    launcher.repo_root = tmp_path
    runs_dir = tmp_path / "runs" / "console" / "sitl-abc" / "drone-01"
    runs_dir.mkdir(parents=True)
    launcher.pid_file = tmp_path / "runs" / "console" / "sitl_owned_pids.json"
    launcher.pid_file.write_text(json.dumps({"pids": [11111, 22222, 33333]}), encoding="utf-8")

    killed = []

    class FakeProc:
        def __init__(self, pid):
            self.pid = pid
            self.terminated = False
            self.waited = False

        def terminate(self):
            self.terminated = True

        def wait(self, timeout=1.0):
            self.waited = True

        def kill(self):
            pass

    direct_proc = FakeProc(11111)
    launcher.procs = [direct_proc]

    fake_proc_map = {
        22222: ("/opt/ardupilot/arducopter -I0", runs_dir),
        33333: ("/opt/ardupilot/arducopter -I0", tmp_path / "other-checkout" / "runs"),
        44444: ("python3 unrelated_script.py", tmp_path / "runs" / "console"),
    }

    monkeypatch.setattr(launcher, "_proc_cmdline_and_cwd", lambda pid: fake_proc_map.get(pid, (None, None)))
    monkeypatch.setattr(launcher, "_load_owned_pids", lambda: [11111, 22222, 33333, 44444])
    monkeypatch.setattr(launcher, "_terminate_pid", lambda pid, timeout=1.0: killed.append(pid))
    monkeypatch.setattr(sitl_launcher.time, "sleep", lambda _s: None)

    launcher.kill_existing_sitl()

    assert direct_proc.terminated and direct_proc.waited
    assert launcher.procs == []
    assert killed == [22222]
    assert not launcher.pid_file.exists()


def test_is_owned_arducopter_pid_rejects_foreign_or_non_arducopter(tmp_path, monkeypatch):
    launcher = SitlLauncher(1)
    launcher.repo_root = tmp_path
    owned_cwd = tmp_path / "runs" / "independent" / "r1" / "sitl-instances" / "drone-01"
    owned_cwd.mkdir(parents=True)
    foreign_cwd = tmp_path / "other" / "runs"
    foreign_cwd.mkdir(parents=True)

    mapping = {
        5001: ("/deps/arducopter -I0 --model quad", owned_cwd),
        5002: ("/deps/arducopter -I0 --model quad", foreign_cwd),
        5003: ("python3 -m sitl.swarm_console", owned_cwd),
    }
    monkeypatch.setattr(launcher, "_proc_cmdline_and_cwd", lambda pid: mapping.get(pid, (None, None)))

    assert launcher._is_owned_arducopter_pid(5001) is True
    assert launcher._is_owned_arducopter_pid(5002) is False
    assert launcher._is_owned_arducopter_pid(5003) is False


def test_launch_sitl_refuses_to_kill_foreign_arducopter_on_restart_or_mismatch(tmp_path, monkeypatch):
    for force_restart, use_gazebo, model_line in (
        (True, False, "99999 /deps/arducopter --model quad"),
        (False, True, "99999 /deps/arducopter --model quad"),
        (False, False, "99999 /deps/arducopter --model JSON"),
    ):
        launcher = SitlLauncher(1, use_gazebo=use_gazebo, force_restart=force_restart)
        launcher.repo_root = tmp_path
        launcher.pid_file = tmp_path / "runs" / "console" / "sitl_owned_pids.json"
        monkeypatch.setattr(launcher, "_running_sitl_models", lambda line=model_line: line)
        monkeypatch.setattr(launcher, "_load_owned_pids", lambda: [])
        monkeypatch.setattr(sitl_launcher.time, "sleep", lambda _s: None)

        with pytest.raises(RuntimeError, match="Refusing to kill foreign ArduCopter"):
            launcher.launch_sitl()


def test_terminate_pid_sends_sigterm_then_sigkill_only_if_still_alive(monkeypatch):
    signals = []
    alive = {"flag": True}

    def fake_kill(pid, sig):
        signals.append((pid, sig))
        if sig == 0 and not alive["flag"]:
            raise ProcessLookupError()

    monkeypatch.setattr(sitl_launcher.os, "kill", fake_kill)
    monkeypatch.setattr(sitl_launcher.time, "sleep", lambda _s: alive.__setitem__("flag", False))

    SitlLauncher._terminate_pid(12345, timeout=0.5)
    assert signals == [(12345, signal.SIGTERM), (12345, 0), (12345, 0)]

    signals.clear()
    ticks = iter([0.0, 0.0, 1.0])
    monkeypatch.setattr(sitl_launcher.os, "kill", lambda pid, sig: signals.append((pid, sig)))
    monkeypatch.setattr(sitl_launcher.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(sitl_launcher.time, "sleep", lambda _s: None)
    SitlLauncher._terminate_pid(12345, timeout=0.5)
    assert signals == [(12345, signal.SIGTERM), (12345, 0), (12345, signal.SIGKILL)]
