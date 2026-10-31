"""Run a scripted SITL scenario against the standalone swarm console, with health checks and log collection.

    python3 -m sitl.fleet_ctl scenarios/election_10.txt --drones 10 --out runs/independent/my-run

Starts ``sitl.swarm_console --standalone`` in its own process group, feeds it the scenario's commands, records
the console log, a driver log (what was sent and whether each wait was met) and a resource log (CPU, memory,
load every 5 s), then stops the whole group by PID (never by name pattern) and writes ``SHA256SUMS``.

Scenario file, one step per line (``#`` starts a comment):

    wait <regex> <timeout_s>      wait for the console output to match; the run is marked FAILED on timeout
    send <console command>        type a command into the console
    expect <regex> <timeout_s>    send nothing; same as wait but the step is named "expect" in the log
    sleep <seconds>
    status                        shorthand for: send status
    kill-agent <drone number>     SIGTERM that drone's agent process (1-based, as the console shows them)
    health                        check that every agent process of this run is alive; FAIL if one died
                                  unexpectedly (agents killed with kill-agent are exempt)

This is a driver for SITL runs. It does not make a run evidence by itself: read the console's run report.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List


@dataclass(frozen=True)
class Step:
    kind: str
    arg: str = ""
    timeout_s: float = 0.0
    line_no: int = 0


def parse_scenario(text: str) -> List[Step]:
    steps: List[Step] = []
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip() if not raw.lstrip().startswith("send") else raw.strip()
        if not line or line.startswith("#"):
            continue
        word, _, rest = line.partition(" ")
        rest = rest.strip()
        if word in ("wait", "expect"):
            pattern, _, timeout = rest.rpartition(" ")
            try:
                timeout_s = float(timeout)
                re.compile(pattern)
            except (ValueError, re.error) as exc:
                raise ValueError(f"line {number}: '{word}' needs '<regex> <timeout_s>' ({exc})") from exc
            if not pattern or timeout_s <= 0:
                raise ValueError(f"line {number}: '{word}' needs a pattern and a positive timeout")
            steps.append(Step(word, pattern, timeout_s, number))
        elif word == "send":
            if not rest:
                raise ValueError(f"line {number}: 'send' needs a command")
            steps.append(Step("send", rest, line_no=number))
        elif word == "sleep":
            try:
                seconds = float(rest)
            except ValueError as exc:
                raise ValueError(f"line {number}: 'sleep' needs seconds") from exc
            if seconds < 0:
                raise ValueError(f"line {number}: sleep must be non-negative")
            steps.append(Step("sleep", timeout_s=seconds, line_no=number))
        elif word in ("status", "health"):
            steps.append(Step(word, line_no=number))
        elif word == "kill-agent":
            if not rest.isdigit() or int(rest) < 1:
                raise ValueError(f"line {number}: 'kill-agent' needs a drone number starting at 1")
            steps.append(Step("kill-agent", rest, line_no=number))
        else:
            raise ValueError(f"line {number}: unknown step '{word}'")
    return steps


def write_checksums(folder: Path) -> Path:
    lines = []
    for path in sorted(folder.rglob("*")):
        if path.is_file() and path.name != "SHA256SUMS":
            lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(folder)}")
    target = folder / "SHA256SUMS"
    target.write_text("\n".join(lines) + "\n")
    return target


class Runner:
    def __init__(self, steps: List[Step], console_args: List[str], out: Path):
        self.steps, self.out = steps, out
        out.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen(
            [sys.executable, "-u", "-m", "sitl.swarm_console", *console_args, "--evidence-dir", str(out / "evidence")],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
            env=dict(os.environ, PYTHONPATH="."), start_new_session=True)
        self.buffer: List[str] = []
        self.killed: set[int] = set()
        self.failed = False
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._monitor, daemon=True).start()

    def _read(self):
        with open(self.out / "console.log", "w") as handle:
            for line in self.proc.stdout:
                handle.write(line)
                handle.flush()
                self.buffer.append(line)

    def log(self, message: str):
        with open(self.out / "driver.log", "a") as handle:
            handle.write(f"{time.strftime('%H:%M:%S')} {message}\n")

    def _group_pids(self) -> List[int]:
        try:
            output = subprocess.run(["ps", "-eo", "pid=,pgid=,pcpu=,rss=,args="], capture_output=True, text=True).stdout
        except OSError:
            return []
        pids = []
        for line in output.splitlines():
            parts = line.split(None, 4)
            if len(parts) == 5 and parts[1] == str(self.proc.pid):
                pids.append(int(parts[0]))
        return pids

    def _monitor(self):
        while self.proc.poll() is None:
            cpu = rss = 0.0
            output = subprocess.run(["ps", "-eo", "pgid=,pcpu=,rss="], capture_output=True, text=True).stdout
            for line in output.splitlines():
                parts = line.split()
                if len(parts) == 3 and parts[0] == str(self.proc.pid):
                    cpu += float(parts[1])
                    rss += float(parts[2]) / 1024.0
            load = open("/proc/loadavg").read().split()[0]
            with open(self.out / "resources.log", "a") as handle:
                handle.write(f"{time.strftime('%H:%M:%S')} cpu%={cpu:.0f} rss_MB={rss:.0f} load1={load}\n")
            time.sleep(5)

    def agents(self) -> dict[int, int]:
        """Drone number (1-based) -> pid for this run's agent processes."""
        found = {}
        output = subprocess.run(["ps", "-eo", "pid=,pgid=,args="], capture_output=True, text=True).stdout
        for line in output.splitlines():
            parts = line.split(None, 2)
            if len(parts) == 3 and parts[1] == str(self.proc.pid) and "independent_agent_node" in parts[2]:
                match = re.search(r"--id[ =](\d+)", parts[2])
                if match:
                    found[int(match.group(1)) + 1] = int(parts[0])
        return found

    def wait_for(self, pattern: str, timeout_s: float, since: int) -> bool:
        end = time.time() + timeout_s
        compiled = re.compile(pattern)
        while time.time() < end:
            if compiled.search("".join(self.buffer[since:])):
                return True
            if self.proc.poll() is not None:
                return False
            time.sleep(0.25)
        return False

    def send(self, line: str):
        self.log(f"SEND {line!r}")
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()

    def execute(self) -> bool:
        try:
            for step in self.steps:
                if self.proc.poll() is not None:
                    self.log("console exited early")
                    self.failed = True
                    break
                mark = len(self.buffer)
                if step.kind == "send":
                    self.send(step.arg)
                elif step.kind == "status":
                    self.send("status")
                elif step.kind in ("wait", "expect"):
                    ok = self.wait_for(step.arg, step.timeout_s, 0 if step.kind == "wait" else mark)
                    self.log(f"{step.kind} {step.arg!r} (line {step.line_no}): {'OK' if ok else 'TIMEOUT'}")
                    self.failed |= not ok
                elif step.kind == "sleep":
                    time.sleep(step.timeout_s)
                elif step.kind == "kill-agent":
                    pid = self.agents().get(int(step.arg))
                    if pid is None:
                        self.log(f"kill-agent D{step.arg}: no such agent process")
                        self.failed = True
                    else:
                        self.killed.add(int(step.arg))
                        self.log(f"kill-agent D{step.arg}: SIGTERM pid {pid}")
                        os.kill(pid, signal.SIGTERM)
                elif step.kind == "health":
                    alive = set(self.agents())
                    dead = [i for i in range(1, self._expected_agents() + 1) if i not in alive and i not in self.killed]
                    self.log(f"health: alive={sorted(alive)} unexpectedly_dead={dead}")
                    self.failed |= bool(dead)
        finally:
            self.stop()
        return not self.failed

    def _expected_agents(self) -> int:
        return int(self.count)

    def stop(self):
        try:
            self.send("q")
        except Exception:
            pass
        for _ in range(20):
            if self.proc.poll() is not None:
                break
            time.sleep(0.5)
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if self.proc.poll() is None or self._group_pids():
                try:
                    os.killpg(self.proc.pid, sig)
                except ProcessLookupError:
                    pass
                time.sleep(3)
        write_checksums(self.out)
        self.log(f"driver done; failed={self.failed}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--drones", type=int, default=10)
    parser.add_argument("--control-mode", default="hybrid")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--extra", default="", help="extra swarm_console arguments, quoted")
    args = parser.parse_args(argv)
    steps = parse_scenario(args.scenario.read_text())
    console_args = ["--drones", str(args.drones), "--standalone", "--control-mode", args.control_mode,
                    "--agent-runtime", "independent", "--adaptive-swarm", "--transition-protocol",
                    *args.extra.split()]
    runner = Runner(steps, console_args, args.out)
    runner.count = args.drones
    ok = runner.execute()
    print("scenario", "PASSED" if ok else "FAILED", "- see", args.out / "driver.log")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
