"""Persistent browser dashboard that safely restarts the grounded swarm controller."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from urllib.error import URLError
from urllib.request import Request, urlopen

from pymavlink import mavutil

from sitl.fleet import build_fleet
from sitl.operator_dashboard import start_dashboard


class ModeSupervisor:
    def __init__(self, args):
        self.args = args
        self.count = args.drones
        self.control_mode = args.control_mode
        self.lock = threading.RLock()
        self.child = None
        self.child_log = None
        self.child_port = args.child_dashboard_port
        self.switching = False
        self.switch_target = None
        self.transition_error = None
        self.last_state = None
        self.sitl_processes = []
        self.owns_sitl = False
        self.root = Path(__file__).resolve().parent.parent
        self.params = Path(args.params_file).resolve()
        self.sim_vehicle = Path(args.sim_vehicle).resolve()
        self.dashboard_server = None

    def _empty_state(self):
        return {
            "control_mode": self.control_mode, "count": self.count, "healthy": 0,
            "airborne": 0, "armed": 0, "leader": None, "formation": "v",
            "goal": [0.0, 0.0], "altitude": 5.0, "formations": [],
            "formation_labels": {}, "formation_change_enabled": False,
            "formation_ready": False, "peer_fault_enabled": False,
            "recovery_label": "Recovery", "recovery_help": "Starting controller…",
            "agents": [], "obstacles": [], "coordinator_stream": False,
        }

    def _fetch_child_state(self, timeout=1.0):
        with urlopen(f"http://127.0.0.1:{self.child_port}/state", timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def dashboard_state(self):
        with self.lock:
            switching = self.switching
            target = self.switch_target
            error = self.transition_error
            current_mode = self.control_mode
        state = None
        try:
            state = self._fetch_child_state(timeout=0.5)
        except (OSError, URLError, ValueError):
            state = self.last_state.copy() if self.last_state else self._empty_state()
        if not switching:
            self.last_state = state.copy()
        state.update({
            "control_mode": current_mode,
            "transitioning": switching,
            "switch_target": target,
            "transition_error": error,
        })
        return state

    def request_mode(self, mode: str) -> dict:
        if mode not in {"centralized", "decentralized", "hybrid"}:
            return {"status": 400, "message": "Unknown control architecture."}
        state = self.dashboard_state()
        if state["control_mode"] == mode:
            return {"status": 200, "message": f"{mode.title()} mode is already active."}
        if state.get("airborne", 0) or state.get("armed", 0):
            return {"status": 409, "message": "Land and disarm every drone before switching modes."}
        if state.get("healthy", 0) != self.count or not state.get("formation_ready", False):
            return {"status": 409, "message": "Switch blocked: every drone needs fresh, healthy, grounded and disarmed status."}
        with self.lock:
            if self.switching:
                return {"status": 409, "message": "A mode switch is already running."}
            self.switching = True
            self.switch_target = mode
            self.transition_error = None
        threading.Thread(target=self._switch_mode, args=(mode,), daemon=True).start()
        return {"status": 202, "message": f"Switching to {mode.title()} safely. The dashboard and SITL will stay running."}

    def _child_command(self, mode: str) -> list[str]:
        command = [sys.executable, "-u", "-m", "sitl.swarm_console",
                   "--drones", str(self.count), "--standalone",
                   "--control-mode", mode, "--dashboard-port", str(self.child_port)]
        if mode in {"decentralized", "hybrid"}:
            command += ["--agent-runtime", "independent", "--adaptive-swarm"]
        return command

    def _start_child(self, mode: str) -> None:
        logdir = self.root / "runs" / "dashboard-manager"
        logdir.mkdir(parents=True, exist_ok=True)
        log = (logdir / f"{mode}-{int(time.time())}.log").open("a", encoding="utf-8")
        child = subprocess.Popen(
            self._child_command(mode), cwd=self.root, stdin=subprocess.PIPE,
            stdout=log, stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        with self.lock:
            self.child = child
            self.child_log = log
        deadline = time.monotonic() + 420.0
        last_report = 0.0
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError(f"{mode} controller exited during startup; see {log.name}")
            try:
                state = self._fetch_child_state(timeout=1.0)
                if state.get("healthy") == self.count and state.get("formation_ready"):
                    self.last_state = state.copy()
                    return
            except (OSError, URLError, ValueError):
                pass
            if time.monotonic() - last_report > 10:
                print(f"Waiting for {mode} controller and {self.count} healthy grounded vehicles…", flush=True)
                last_report = time.monotonic()
            time.sleep(0.25)
        raise TimeoutError(f"{mode} controller did not become ready within 420 seconds")

    def _stop_child(self) -> None:
        with self.lock:
            child = self.child
        if child is None or child.poll() is not None:
            return
        if child.stdin is None:
            raise RuntimeError("controller input pipe is unavailable; refusing forced mode switch")
        child.stdin.write("q\n")
        child.stdin.flush()
        try:
            code = child.wait(timeout=100.0)
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("controller did not confirm grounded shutdown; left it running for safety") from exc
        if code != 0:
            raise RuntimeError(f"controller stopped with exit code {code}; SITL was left running")
        child.stdin.close()
        with self.lock:
            self.child = None
            if self.child_log:
                self.child_log.close()
                self.child_log = None

    def _switch_mode(self, mode: str) -> None:
        try:
            self._stop_child()
            self._start_child(mode)
            with self.lock:
                self.control_mode = mode
                self.switching = False
                self.switch_target = None
                self.transition_error = None
            print(f"Dashboard mode switch complete: {mode}", flush=True)
        except Exception as exc:
            with self.lock:
                self.switching = False
                self.switch_target = None
                self.transition_error = f"Mode switch stopped safely: {exc}"
            print(self.transition_error, flush=True)

    def proxy_command(self, command: str) -> None:
        parts = command.split()
        if parts and parts[0] in {"shape", "formation"} and len(parts) == 2:
            path, payload = "/formation", {"formation": parts[1]}
        else:
            path, payload = "/command", {"command": command}
        request = Request(
            f"http://127.0.0.1:{self.child_port}{path}",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urlopen(request, timeout=3.0) as response:
            response.read()

    def _wait_heartbeat(self, port: int, timeout: float = 2.0) -> bool:
        connection = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", autoreconnect=False)
        try:
            return connection.wait_heartbeat(timeout=timeout) is not None
        finally:
            connection.close()

    def _ensure_sitl(self) -> None:
        fleet = build_fleet(self.count)
        # Probe concurrently: sequential sub-second waits can miss staggered
        # heartbeat intervals and misclassify a healthy running fleet as partial.
        with ThreadPoolExecutor(max_workers=len(fleet)) as pool:
            probes = [pool.submit(self._wait_heartbeat, spec.port, 2.5) for spec in fleet]
            available = []
            for probe in probes:
                try:
                    available.append(probe.result())
                except OSError:
                    available.append(False)
        if all(available):
            print("Using existing standalone SITL and GCS telemetry streams.", flush=True)
            return
        if any(available):
            missing = [str(spec.index + 1) for spec, found in zip(fleet, available) if not found]
            raise RuntimeError(
                "Found a partial standalone SITL fleet; refusing to launch duplicates. "
                "Missing UDP telemetry for drone(s): " + ", ".join(missing)
            )
        if not self.sim_vehicle.exists():
            raise FileNotFoundError(f"sim_vehicle.py not found: {self.sim_vehicle}")
        self.owns_sitl = True
        for spec in fleet:
            command = [sys.executable, str(self.sim_vehicle), "-v", "ArduCopter",
                       "-I", str(spec.index), "-N", "--auto-sysid",
                       f"--custom-location={spec.home_standalone}",
                       "--out=udp:127.0.0.1:14550",
                       f"--out=udp:127.0.0.1:{spec.port}",
                       f"--add-param-file={self.params}"]
            process = subprocess.Popen(
                command, cwd="/home/drone/ardupilot/ArduCopter",
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.sitl_processes.append(process)
            print(f"Started standalone SITL D{spec.index + 1} with QGC UDP 14550.", flush=True)
            time.sleep(1.5)
        deadline = time.monotonic() + 180.0
        for spec in fleet:
            while time.monotonic() < deadline:
                try:
                    if self._wait_heartbeat(spec.port, 2.0):
                        break
                except OSError:
                    pass
                time.sleep(0.25)
            else:
                raise TimeoutError(f"No MAVLink heartbeat from D{spec.index + 1}")

    def _stop_sitl(self) -> None:
        if not self.owns_sitl:
            return
        for process in self.sitl_processes:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 8.0
        for process in self.sitl_processes:
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.sitl_processes.clear()

    def run(self) -> None:
        self.dashboard_server = start_dashboard(
            self, self.args.dashboard_port, state_provider=self.dashboard_state,
            mode_switch=self.request_mode, command_proxy=self.proxy_command,
        )
        print(f"Persistent swarm dashboard: http://127.0.0.1:{self.args.dashboard_port}", flush=True)
        try:
            self._ensure_sitl()
            self._start_child(self.control_mode)
            print(f"Swarm mode manager ready ({self.count} vehicles, {self.control_mode}).", flush=True)
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nStopping dashboard manager…", flush=True)
        finally:
            try:
                self._stop_child()
            except Exception as exc:
                print(f"Controller remains running for safety: {exc}", flush=True)
            if self.dashboard_server:
                self.dashboard_server.shutdown()
                self.dashboard_server.server_close()
            self._stop_sitl()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drones", type=int, default=10)
    parser.add_argument("--control-mode", choices=("centralized", "decentralized", "hybrid"), default="decentralized")
    parser.add_argument("--dashboard-port", type=int, default=8765)
    parser.add_argument("--child-dashboard-port", type=int, default=8766)
    parser.add_argument("--params-file", default="sitl/swarm_params.parm")
    parser.add_argument("--sim-vehicle", default="/home/drone/ardupilot/Tools/autotest/sim_vehicle.py")
    args = parser.parse_args()
    if not 1 <= args.drones <= 10:
        parser.error("--drones must be between 1 and 10")
    if not 1024 <= args.dashboard_port <= 65535 or not 1024 <= args.child_dashboard_port <= 65535:
        parser.error("dashboard ports must be between 1024 and 65535")
    if args.dashboard_port == args.child_dashboard_port:
        parser.error("parent and child dashboard ports must differ")
    if not Path(args.params_file).is_absolute():
        args.params_file = str(Path(__file__).resolve().parent.parent / args.params_file)
    ModeSupervisor(args).run()


if __name__ == "__main__":
    main()
