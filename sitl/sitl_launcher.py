"""Start and stop the ArduPilot SITL instances (standalone quad or Gazebo JSON) for the independent runtime.

Extracted unchanged from the retired `SwarmConsole` (archive/legacy_runtimes/swarm_console_legacy.py) on 2026-11-04:
the independent console only ever used its SITL start-up and clean-up.
"""
from __future__ import annotations

import atexit
import os
import socket
import subprocess
import time
import uuid
from pathlib import Path
from typing import List, Optional

from pymavlink import mavutil

from sitl.common_frame import CommonCoordinateFrame
from sitl.fleet import DATUM_ALT, DATUM_LAT, DATUM_LON, build_fleet


class SitlLauncher:
    def __init__(self, n: int, use_gazebo: Optional[bool] = None, force_restart: bool = False):
        self.n = n
        self.fleet = build_fleet(n)
        self.repo_root = Path(__file__).resolve().parent.parent
        self.frame = CommonCoordinateFrame(DATUM_LAT, DATUM_LON, DATUM_ALT)
        self.use_gazebo = use_gazebo
        self.force_restart = force_restart
        self.sitl_params_file = None
        self.sitl_run_dir = None
        self.procs: List[subprocess.Popen] = []
        self.use_tcp = False  # True when this launcher started the SITL binaries (no MAVProxy UDP forwarding)
        self.evidence = None

    @staticmethod
    def detect_gazebo() -> bool:
        try:
            return subprocess.run(["pgrep", "-f", "gz sim"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode == 0
        except Exception:
            return False

    @staticmethod
    def tcp_endpoint_available(host: str, port: int, timeout: float = 1.0) -> bool:
        """Check a TCP endpoint with a bounded connect time before pymavlink opens it."""
        try:
            with socket.create_connection((host, int(port)), timeout=max(0.05, float(timeout))):
                return True
        except OSError:
            return False

    @staticmethod
    def kill_existing_sitl():
        try:
            subprocess.run(["pkill", "-9", "-f", "arducopter"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
        except Exception:
            pass

    @staticmethod
    def _running_sitl_models() -> str:
        try:
            res = subprocess.run(["pgrep", "-a", "-f", "arducopter"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return res.stdout if res.returncode == 0 else ""
        except Exception:
            return ""

    def launch_sitl(self):
        is_gazebo = self.use_gazebo if self.use_gazebo is not None else self.detect_gazebo()
        running = self._running_sitl_models()

        if self.force_restart:
            print(">> [--restart] Cleaning up prior SITL processes...")
            self.kill_existing_sitl()
        elif running and is_gazebo and "--model quad" in running:
            print(">> Standalone SITL instances (--model quad) are running but Gazebo is active; restarting in JSON mode...")
            self.kill_existing_sitl()
        elif running and not is_gazebo and "--model JSON" in running:
            print(">> Gazebo-linked SITL instances are running but standalone was requested; restarting...")
            self.kill_existing_sitl()

        try:
            probe = mavutil.mavlink_connection(f"udpin:127.0.0.1:{self.fleet[0].port}")
            alive = probe.wait_heartbeat(timeout=1.5)
            probe.close()
            if alive:
                print(f">> [OK] Detected active SITL instances on UDP {self.fleet[0].port}...; connecting to them.")
                return
        except Exception:
            pass
        if self.tcp_endpoint_available("127.0.0.1", self.fleet[0].tcp_port, timeout=1.0):
            try:
                probe = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.fleet[0].tcp_port}")
                alive = probe.wait_heartbeat(timeout=1.5)
                probe.close()
                if alive:
                    print(f">> [OK] Detected active SITL instances on TCP {self.fleet[0].tcp_port}...; connecting to them.")
                    self.use_tcp = True
                    return
            except Exception:
                pass

        consolidated_sitl_bin = self.repo_root.parent.parent / "dependencies" / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
        default_sitl_bin = consolidated_sitl_bin if consolidated_sitl_bin.exists() else Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
        sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(default_sitl_bin)))
        if not sitl_bin.exists():
            raise FileNotFoundError(f"ArduCopter binary not found at: {sitl_bin} (set ARDUCOPTER_BIN)")
        params_file = str(self.sitl_params_file or (self.repo_root / "sitl" / "swarm_params.parm"))

        if is_gazebo:
            print(f">> [3D GAZEBO] Launching {self.n} SITL JSON instances (FDM ports {[d.fdm_port for d in self.fleet]})...")
        else:
            print(f">> [2D STANDALONE] Launching {self.n} SITL quad instances...")
        # ArduPilot writes eeprom.bin and logs/ relative to its current working
        # directory. Give every instance private files so concurrent vehicles
        # cannot overwrite one another's parameters or DataFlash evidence.
        runtime_root = (Path(self.sitl_run_dir) if self.sitl_run_dir is not None
                        else self.repo_root / "runs" / "console" / f"sitl-{uuid.uuid4().hex}")
        for spec in self.fleet:
            instance_dir = runtime_root / f"drone-{spec.sysid:02d}"
            instance_dir.mkdir(parents=True, exist_ok=True)
            cmd = [str(sitl_bin), f"-I{spec.inst}", "--sysid", str(spec.inst + 1)]
            if is_gazebo:
                cmd += ["--model", "JSON", "--home", spec.home_gazebo, "--sim-address", "127.0.0.1"]
            else:
                cmd += ["--model", "quad", "--home", spec.home_standalone]
            # Keep the controller's SERIAL0/TCP connection private and mirror
            # each vehicle's MAVLink2 stream to QGroundControl on UDP 14550.
            cmd += ["--serial1", "udpclient:127.0.0.1:14550"]
            cmd += ["--defaults", params_file, "--speedup", "1"]
            # Keep the simulated vehicles alive if the operator/manager process is lost.
            # Normal shutdown still terminates these direct child PIDs in cleanup().
            self.procs.append(subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                               cwd=instance_dir, start_new_session=True))
        self.use_tcp = True  # our own instances only expose ArduPilot's TCP server
        atexit.register(self.cleanup)
        time.sleep(2.5)

    def cleanup(self):
        if self.procs:
            print("\n>> Shutting down SITL processes...")
            for p in self.procs:
                p.terminate()
                try:
                    p.wait(timeout=1.0)
                except Exception:
                    p.kill()
            self.procs.clear()
