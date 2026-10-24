"""Kill the independent operator process and observe node behavior in the same runtime."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import socket
import subprocess
import sys
import time

from sitl.fleet import build_fleet


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def leader_reached_north_goal(text: str, goal_m: float, tolerance_m: float = 0.35) -> bool:
    """Accept the leader once a printed status is within the declared target tolerance."""
    for match in re.finditer(
            r"D1: airborne(?:\s+armed=(?:yes|no))?\s+N=(-?\d+(?:\.\d+)?)", text):
        try:
            if abs(float(match.group(1)) - float(goal_m)) <= float(tolerance_m):
                return True
        except ValueError:
            continue
    return False


def status_confirms_landed(status: dict) -> bool:
    position = status.get("position")
    if (status.get("command") != "landed" or status.get("armed") is not False or
            status.get("airborne") is not False or not isinstance(position, list) or
            len(position) != 3):
        return False
    try:
        return float(position[2]) > -0.5
    except (TypeError, ValueError):
        return False


def currently_confirmed_landed(latest: dict, received_at: dict, now: float,
                               airborne_seen: set[int], count: int,
                               freshness_s: float = 2.5) -> set[int]:
    """Return agents with a fresh, current landed/disarmed status after flight was observed."""
    if freshness_s <= 0:
        raise ValueError("status freshness limit must be positive")
    return {
        drone_id for drone_id in range(count)
        if drone_id in airborne_seen
        and drone_id in latest
        and drone_id in received_at
        and 0.0 <= now - received_at[drone_id] <= freshness_s
        and status_confirms_landed(latest[drone_id])
    }


def terminate_pids(pids: list[int]) -> None:
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except ProcessLookupError:
                pass
        if not alive:
            return
        time.sleep(0.1)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def start_standalone_sitl(root: Path, count: int) -> list[subprocess.Popen]:
    configured = os.environ.get("ARDUCOPTER_BIN")
    candidates = [
        Path(configured) if configured else None,
        root.parent.parent / "dependencies" / "ardupilot" / "build" / "sitl" / "bin" / "arducopter",
        Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter",
    ]
    binary = next((path for path in candidates if path is not None and path.is_file()), None)
    if binary is None:
        raise FileNotFoundError("Could not locate ArduCopter SITL binary")
    params = root / "sitl" / "swarm_params.parm"
    processes = []
    for spec in build_fleet(count):
        command = [str(binary), f"-I{spec.inst}", "--sysid", str(spec.sysid),
                   "--model", "quad", "--home", spec.home_standalone,
                   "--defaults", str(params), "--speedup", "1"]
        processes.append(subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                           stderr=subprocess.DEVNULL, start_new_session=True))
    time.sleep(2.5)
    return processes


class ConsolePty:
    def __init__(self, process: subprocess.Popen, master: int, transcript: Path):
        self.process = process
        self.master = master
        self.transcript = transcript.open("a", encoding="utf-8")
        self.text = ""

    def read(self, timeout: float = 0.25) -> str:
        ready, _, _ = select.select([self.master], [], [], timeout)
        if not ready:
            return ""
        try:
            raw = os.read(self.master, 65536)
        except OSError:
            return ""
        chunk = raw.decode("utf-8", errors="replace")
        self.text += chunk
        self.transcript.write(chunk)
        self.transcript.flush()
        return chunk

    def wait_for(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate(self.text):
                return
            if self.process.poll() is not None:
                raise RuntimeError(f"operator process exited before {description}; see console.log")
            self.read(0.2)
        raise TimeoutError(f"timed out waiting for {description}; see console.log")

    def send(self, command: str) -> None:
        os.write(self.master, (command + "\n").encode())

    def close(self) -> None:
        self.transcript.close()
        try:
            os.close(self.master)
        except OSError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--peer-loss", type=float, default=0.1)
    parser.add_argument("--peer-latency-ms", type=float, default=50.0)
    parser.add_argument("--peer-seed", type=int, default=46)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--operator-loss-timeout", type=float, default=90.0)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    # Launch SITL as a sibling of the operator, so killing the operator cannot
    # also stop its simulated flight controllers during this coordinator-loss test.
    sitl_processes = start_standalone_sitl(root, args.count)
    command = [sys.executable, "-u", "-m", "sitl.swarm_console",
               "--drones", str(args.count), "--standalone",
               "--control-mode", "decentralized", "--agent-runtime", "independent",
               "--peer-loss", str(args.peer_loss),
               "--peer-latency-ms", str(args.peer_latency_ms),
               "--peer-seed", str(args.peer_seed)]
    master, slave = pty.openpty()
    process = subprocess.Popen(command, cwd=root, stdin=slave, stdout=slave, stderr=slave,
                               start_new_session=True, env={**os.environ, "PYTHONPATH": str(root)})
    os.close(slave)

    # Capture the run directory from the console's first line.
    transcript = root / "runs" / "independent" / f"operator-loss-launch-{process.pid}.log"
    terminal = ConsolePty(process, master, transcript)
    operator_killed = False
    try:
        terminal.wait_for(lambda value: ">> Independent-node evidence:" in value, 20.0,
                          "run evidence directory")
        match = re.search(r">> Independent-node evidence: ([^\r\n]+)", terminal.text)
        if not match:
            raise RuntimeError("console did not print its evidence directory")
        run_dir = Path(match.group(1).strip())
        manifest_path = run_dir / "processes.json"
        terminal.transcript.close()
        transcript = run_dir / "console.log"
        terminal.transcript = transcript.open("a", encoding="utf-8")

        terminal.wait_for(lambda value: "Independent Swarm > " in value, 420.0,
                          "all five independent agents becoming ready")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["sitl_pids"] = [sim.pid for sim in sitl_processes]
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        prompt_count = terminal.text.count("Independent Swarm > ")
        terminal.send("launch")
        terminal.wait_for(lambda value: value.count("Takeoff confirmed") >= args.count, 180.0,
                          "all agents confirming takeoff")
        terminal.wait_for(lambda value: value.count("Independent Swarm > ") > prompt_count, 30.0,
                          "operator prompt after launch")
        prompt_count = terminal.text.count("Independent Swarm > ")
        terminal.send("w 2")
        terminal.wait_for(lambda value: value.count("Independent Swarm > ") > prompt_count, 15.0,
                          "north mission accepted")
        movement_deadline = time.monotonic() + 45.0
        while not leader_reached_north_goal(terminal.text, goal_m=2.0):
            if time.monotonic() >= movement_deadline:
                raise TimeoutError("swarm did not move toward the accepted goal; see console.log")
            prompt_count = terminal.text.count("Independent Swarm > ")
            terminal.send("status")
            terminal.wait_for(lambda value: value.count("Independent Swarm > ") > prompt_count,
                              min(12.0, movement_deadline - time.monotonic()),
                              "fresh position status after north mission")

        with (run_dir / "operator_loss_status.jsonl").open("a", encoding="utf-8") as out:
            out.write(json.dumps({"utc": utc_now(), "kind": "operator_process_kill",
                                  "pid": process.pid, "mission_goal_ne_m": 2.0}) + "\n")
        process.kill()
        process.wait(timeout=5.0)
        operator_killed = True
        # The harness stays alive in the same network namespace while the operator dies.
        sockets = []
        for drone_id in range(args.count):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 22000 + drone_id))
            sock.setblocking(False)
            sockets.append(sock)

        status_path = run_dir / "operator_loss_status.jsonl"
        latest = {}
        latest_received_at = {}
        airborne_seen = set()
        landed_seen = set()
        operator_loss_started = time.monotonic()
        fallback_sent = False
        with status_path.open("a", encoding="utf-8") as out:
            while time.monotonic() - operator_loss_started < args.operator_loss_timeout:
                for drone_id, sock in enumerate(sockets):
                    while True:
                        try:
                            raw, _ = sock.recvfrom(8192)
                        except BlockingIOError:
                            break
                        try:
                            status = json.loads(raw.decode("utf-8"))
                        except (UnicodeDecodeError, json.JSONDecodeError):
                            continue
                        if status.get("type") != "status" or status.get("drone_id") != drone_id:
                            continue
                        latest[drone_id] = status
                        latest_received_at[drone_id] = time.monotonic()
                        if status.get("airborne"):
                            airborne_seen.add(drone_id)
                        out.write(json.dumps({"utc": utc_now(), "kind": "node_status_after_operator_loss",
                                              **status}, allow_nan=False, separators=(",", ":")) + "\n")
                        out.flush()
                landed_seen = currently_confirmed_landed(
                    latest, latest_received_at, time.monotonic(), airborne_seen, args.count,
                )
                if len(landed_seen) == args.count:
                    break
                if (not fallback_sent and time.monotonic() - operator_loss_started >
                        args.operator_loss_timeout - 40.0):
                    # Preserve a landing route if lease-expiry LAND did not appear.
                    payload = json.dumps({"version": 1, "type": "command", "sequence": 999999,
                                          "action": "land", "target_alt_m": 5.0,
                                          "issued_at": time.time(), "expires_at": time.time() + 5.0},
                                         separators=(",", ":")).encode()
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                        for drone_id in range(args.count):
                            sender.sendto(payload, ("127.0.0.1", 21000 + drone_id))
                    fallback_sent = True
                    out.write(json.dumps({"utc": utc_now(), "kind": "emergency_land_fallback"}) + "\n")
                    out.flush()
                time.sleep(0.05)

        for sock in sockets:
            sock.close()
        all_landed = len(landed_seen) == args.count
        summary = {
            "utc": utc_now(), "kind": "operator_loss_result", "all_landed": all_landed,
            "all_disarmed": all(
                latest.get(i, {}).get("armed") is False for i in range(args.count)
            ),
            "operator_process_killed": process.poll() is not None,
            "agents_seen": sorted(latest), "airborne_seen": sorted(airborne_seen),
            "landed_seen": sorted(landed_seen), "emergency_land_fallback": fallback_sent,
            "seconds_after_loss": round(time.monotonic() - operator_loss_started, 2),
            "latest": {str(i + 1): latest[i] for i in sorted(latest)},
        }
        with status_path.open("a", encoding="utf-8") as out:
            out.write(json.dumps(summary, allow_nan=False, separators=(",", ":")) + "\n")
        (run_dir / "operator_loss_summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                                            encoding="utf-8")

        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if all_landed:
            terminate_pids(manifest.get("agent_pids", []))
            terminate_pids(manifest.get("sitl_pids", []))
        else:
            raise RuntimeError(f"not all vehicles landed; leaving processes alive. Evidence: {run_dir}")
        print(json.dumps({"all_landed": all_landed, "run_dir": str(run_dir),
                          "operator_loss_summary": str(run_dir / "operator_loss_summary.json"),
                          "fallback_land_used": fallback_sent}))
        if not all_landed:
            raise SystemExit(1)
    finally:
        if process.poll() is None:
            try:
                os.write(master, b"q\n")
                process.wait(timeout=45.0)
            except (OSError, subprocess.TimeoutExpired):
                process.kill()
                process.wait(timeout=5.0)
        terminal.close()
        if not operator_killed:
            # Before the intentional manager-loss point, the console got a chance
            # to land the fleet through its normal q/shutdown path.
            terminate_pids([sim.pid for sim in sitl_processes])


if __name__ == "__main__":
    main()
