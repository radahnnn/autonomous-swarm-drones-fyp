#!/usr/bin/env python3
"""
Swarm Flight Console: one pilot console for 1-10 ArduPilot SITL drones, standalone or in Gazebo.

    python3 sitl/swarm_console.py --drones 5 --gazebo        # 3D Gazebo world (cinewhoop_5drones.sdf)
    python3 sitl/swarm_console.py --drones 3 --standalone    # headless 2D SITL, fast arming
    python3 sitl/swarm_console.py --drones 5 --auto-launch   # take off immediately
    python3 -m sitl.swarm_console --drones 5 --standalone --control-mode decentralized

Replaces the former 3-drone, 5-drone and self-healing consoles. Features:
- Any swarm size; formation slots come from sitl/formation_library.py, ports/homes from sitl/fleet.py.
- Drones start on the ground; `launch` arms (EKF-gated), takes off and forms up everything together.
- Centralized mode: live formation morphing with Hungarian slot assignment.
- Decentralized mode: per-agent guidance using only configured one-hop peer state; see
  docs/DECENTRALIZED_SITL.md for the current experimental limits.
- Hybrid mode: centralized waypoint supervision with local peer guidance as a fallback.
- Fault handling: heartbeat watchdog, `fail <id>` injection, reserved failed slots during landing,
  leader bookkeeping, and `revive <id>` / `lift <id>` staged rejoin.
- Square patrol demo (`p`), synchronized landing (`q`).
"""

import argparse
import atexit
import math
import os
import socket
import subprocess
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import List, Optional

import numpy as np
from pymavlink import mavutil

from sitl.common_frame import CommonCoordinateFrame
from sitl.decentralized_guidance import DecentralizedGuidance
from sitl.fleet import DATUM_ALT, DATUM_LAT, DATUM_LON, MAX_DRONES, build_fleet
from sitl.flight_prep import arm_with_retry, takeoff_and_verify, wait_ekf_ready
from sitl.formation_library import STYLES, assign_slots, slots_for
from sitl.run_evidence import RunEvidence, SettledMotionDetector

# Position + yaw setpoint (velocity, acceleration and yaw-rate ignored)
_TYPE_MASK_POS_YAW = 0b0000101111111000


def _armed_state(conn):
    """Return armed/disarmed from a fresh heartbeat, or None if telemetry is missing."""
    heartbeat = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
    if heartbeat is None:
        return None
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return bool(heartbeat.base_mode & armed_flag)


class SwarmConsole:
    # Fixed swarm heading: 0 rad = North. Without a commanded yaw ArduCopter turns each drone
    # toward its own direction of travel, so the drones would end up facing different ways.
    FORMATION_YAW_RAD = 0.0
    CRUISE_ALT = 5.0
    MORPH_STEPS = 45  # 4.5 s at the 10 Hz guidance rate

    def __init__(self, n: int = 5, use_gazebo: Optional[bool] = None, force_restart: bool = False,
                 ekf_timeout: Optional[float] = None, control_mode: str = "centralized",
                 peer_loss: float = 0.0, peer_latency_ms: float = 30.0,
                 peer_seed: Optional[int] = None, peer_block: Optional[str] = None,
                 peer_burst_loss: bool = False, auto_recover: bool = False, agent_runtime: str = "local"):
        if control_mode not in ("centralized", "decentralized", "hybrid"):
            raise ValueError("control_mode must be 'centralized', 'decentralized', or 'hybrid'")
        self.n = n
        self.control_mode = control_mode
        self.decentralized_guidance = (
            DecentralizedGuidance(n, peer_loss=peer_loss, peer_latency_ms=peer_latency_ms,
                                 peer_seed=peer_seed, control_mode=control_mode,
                                 peer_burst_loss=peer_burst_loss, agent_runtime=agent_runtime)
            if control_mode in ("decentralized", "hybrid") else None
        )
        self.coordinator_connected = True
        self.auto_recover = bool(auto_recover)
        self.failure_causes = [None] * n
        self.recovery_stable_since = [None] * n
        self.recovery_attempted = [False] * n
        self.last_armed_state = [None] * n
        if self.decentralized_guidance is not None and peer_block:
            try:
                sender, recipient = (int(value) - 1 for value in peer_block.split(":"))
                self.decentralized_guidance.peer_bus.block_link(sender, recipient)
            except (ValueError, TypeError):
                raise ValueError("peer_block must be a directed link like '1:2' (1-indexed drone IDs)")
        self.fleet = build_fleet(n)
        self.repo_root = Path(__file__).resolve().parent.parent
        self.frame = CommonCoordinateFrame(DATUM_LAT, DATUM_LON, DATUM_ALT)
        self.use_gazebo = use_gazebo
        self.force_restart = force_restart
        self.sitl_params_file = None

        # Swarm state. busy[i]: a worker thread owns drone i's MAVLink connection and the
        # guidance thread must not read it (pymavlink connections are not thread-safe).
        self.drone_health = [True] * n          # False = failed / disconnected
        self.simulated_faults = [False] * n     # True if the pilot injected a failure
        self.is_airborne = [False] * n
        self.last_heartbeat = [time.time()] * n
        self.last_real_heartbeat = [0.0] * n
        self.busy = [False] * n
        self.last_pos = [np.zeros(3) for _ in range(n)]
        self.last_pos_at = [0.0] * n
        self.current_leader_idx = 0

        # Heartbeats are paced by *sim* time: in Gazebo (slow real-time factor) the 2D timeout would false-trip.
        gazebo_mode = use_gazebo if use_gazebo is not None else self.detect_gazebo()
        self.hb_timeout = 25.0 if gazebo_mode else 3.5
        # Wall-clock budget for the EKF to converge (~45 s of sim time): Gazebo can run at 5-30% real time.
        self.ekf_timeout = ekf_timeout if ekf_timeout is not None else (900.0 if gazebo_mode else 300.0)

        self.current_formation_style = "v"
        self.current_offsets = np.zeros((n, 3), dtype=float)
        self.target_offsets = np.zeros((n, 3), dtype=float)
        self.morph_steps_remaining = 0

        self.connections: List[mavutil.mavlink_connection] = []
        self.procs: List[subprocess.Popen] = []
        self.use_tcp = False  # True when this console started the SITL binaries (no MAVProxy UDP forwarding)
        self.frame_origins: List[np.ndarray] = []
        self.centroid_pos = np.array([0.0, 0.0, -self.CRUISE_ALT])  # [North, East, Down]
        self.running = True
        self.lock = threading.Lock()
        self.guidance_thread = None
        self.evidence = None
        self.landed = False
        self.landing_requested = False
        self.recovery_workers = []
        self.last_target_change = time.time()
        self.settled_motion = SettledMotionDetector()

        self._calculate_active_topology()
        self.current_offsets = self.target_offsets.copy()

    # ------------------------------------------------------------------ helpers
    def _active(self, i: int) -> bool:
        return self.drone_health[i] and not self.simulated_faults[i] and self.is_airborne[i]

    def _active_indices(self) -> List[int]:
        return [i for i in range(self.n) if self._active(i)]

    def _valid_id(self, drone_num: int) -> bool:
        if 1 <= drone_num <= self.n:
            return True
        print(f">> Error: Drone ID must be 1 to {self.n}. Received: {drone_num}")
        return False

    # ------------------------------------------------------------------ SITL process management
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
        for spec in self.fleet:
            cmd = [str(sitl_bin), f"-I{spec.inst}", "--sysid", str(spec.inst + 1)]
            if is_gazebo:
                cmd += ["--model", "JSON", "--home", spec.home_gazebo, "--sim-address", "127.0.0.1"]
            else:
                cmd += ["--model", "quad", "--home", spec.home_standalone]
            cmd += ["--defaults", params_file, "--speedup", "1"]
            # Keep the simulated vehicles alive if the operator/manager process is lost.
            # Normal shutdown still terminates these direct child PIDs in cleanup().
            self.procs.append(subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                               start_new_session=True))
        self.use_tcp = True  # our own instances only expose ArduPilot's TCP server
        atexit.register(self.cleanup)
        time.sleep(2.5)

    def cleanup(self):
        if self.decentralized_guidance is not None:
            self.decentralized_guidance.close()
        if self.procs:
            print("\n>> Shutting down SITL processes...")
            for p in self.procs:
                p.terminate()
                try:
                    p.wait(timeout=1.0)
                except Exception:
                    p.kill()
            self.procs.clear()

    # ------------------------------------------------------------------ connection / frames
    def connect_all(self):
        print(f">> Establishing MAVLink links to all {self.n} drones...")
        self.connections.clear()
        for spec in self.fleet:
            endpoint = f"tcp:127.0.0.1:{spec.tcp_port}" if self.use_tcp else f"udpin:127.0.0.1:{spec.port}"
            conn = None
            for attempt in range(10):
                candidate = None
                try:
                    if self.use_tcp and not self.tcp_endpoint_available(
                            "127.0.0.1", spec.tcp_port, timeout=1.0):
                        if attempt == 9:
                            print(f"  [RETRY] {spec.label} TCP endpoint {spec.tcp_port} is unreachable.")
                        time.sleep(0.5)
                        continue
                    candidate = mavutil.mavlink_connection(endpoint)
                    if candidate.wait_heartbeat(timeout=3.0):
                        conn = candidate
                        print(f"  [ONLINE] {spec.label} connected on {endpoint} (SysID={conn.target_system})")
                        break
                except Exception as exc:
                    if attempt == 9:
                        print(f"  [RETRY] {spec.label} link attempt 10/10 failed: {exc}")
                finally:
                    # Timed-out TCP clients must be closed: otherwise repeated retries
                    # fill ArduPilot's small listen backlog and block later connections.
                    if candidate is not None and candidate is not conn:
                        try:
                            candidate.close()
                        except Exception:
                            pass
                if conn is None:
                    time.sleep(0.5)
            if not conn:
                raise RuntimeError(
                    f"Failed to receive a MAVLink heartbeat from {spec.label} on {endpoint} after 10 attempts. "
                    f"Start the SITL drones first (see sitl/launch_console.sh) or let this console launch them."
                )
            self.connections.append(conn)
            self.last_real_heartbeat[spec.inst] = time.time()
            if self.evidence:
                self.evidence.connected += 1
                self.evidence.event("connected", drone=spec.inst + 1, endpoint=endpoint, sysid=conn.target_system)
            conn.mav.param_set_send(conn.target_system, conn.target_component, b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.param_set_send(conn.target_system, conn.target_component, b"GUID_TIMEOUT", 3.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.request_data_stream_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

        print(">> Waiting for EKF convergence (position + GPS 3D fix) on all vehicles...")
        with ThreadPoolExecutor(max_workers=self.n) as executor:
            futures = [executor.submit(wait_ekf_ready, c, self.fleet[i].label, self.ekf_timeout) for i, c in enumerate(self.connections)]
            if not all(f.result() for f in futures):
                raise RuntimeError("EKF did not converge on one or more drones. Check Gazebo RTF / SITL logs.")
        if self.evidence:
            self.evidence.ekf_ready = True
            self.evidence.event("ekf_ready", drones=self.n)

    def sync_frames(self):
        """Each drone's NED offset to the shared datum (works on the ground)."""
        print(">> Synchronizing shared WGS84 flat-earth coordinate frames...")
        self.frame_origins.clear()
        for i, conn in enumerate(self.connections):
            loc_msg, glob_msg = None, None
            t_s = time.time()
            while time.time() - t_s < 15.0:
                m = conn.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.5)
                if m:
                    if m.get_type() == "LOCAL_POSITION_NED":
                        loc_msg = m
                    else:
                        glob_msg = m
                if loc_msg and glob_msg:
                    break
            if not (loc_msg and glob_msg):
                raise RuntimeError(
                    f"No LOCAL/GLOBAL position telemetry from {self.fleet[i].label}; refusing to use a zero frame origin."
                )
            g_ned = self.frame.gps_to_global_ned(glob_msg.lat / 1e7, glob_msg.lon / 1e7, glob_msg.relative_alt / 1000.0)
            l_ned = np.array([loc_msg.x, loc_msg.y, loc_msg.z])
            self.frame_origins.append(g_ned - l_ned)
            self.last_pos[i] = g_ned
            self.last_pos_at[i] = time.time()

    # ------------------------------------------------------------------ launch / liftoff
    def launch_swarm(self, target_alt: Optional[float] = None, wait: bool = False):
        """'launch': arm, take off and form up every grounded, healthy drone together."""
        if self._mission_change_blocked():
            return
        target_alt = self.CRUISE_ALT if target_alt is None else target_alt
        with self.lock:
            idxs = [i for i in range(self.n) if not self.is_airborne[i] and not self.simulated_faults[i] and not self.busy[i]]
            for i in idxs:
                self.busy[i] = True  # claim the connections for the launch worker
        if not idxs:
            print(">> Nothing to launch (all drones airborne, failed, or already launching). Use 'r <id>' for failed drones.")
            return
        if self.evidence:
            self.evidence.event("launch_requested", drones=[i + 1 for i in idxs], target_alt_m=target_alt)
        worker = threading.Thread(target=self._launch_worker, args=(idxs, target_alt), daemon=True)
        worker.start()
        if wait:
            worker.join()

    def _launch_worker(self, idxs, target_alt):
        try:
            print(f"\n>> [LAUNCH] Arming and taking off Drones {[i + 1 for i in idxs]} to {target_alt:.1f}m...")

            def prep(i):
                conn, label = self.connections[i], self.fleet[i].label
                return (
                    wait_ekf_ready(conn, label, self.ekf_timeout)
                    and arm_with_retry(conn, label, timeout=120.0, force_after=60.0)
                    and takeoff_and_verify(conn, label, target_alt)
                )

            with ThreadPoolExecutor(max_workers=len(idxs)) as executor:
                results = list(executor.map(prep, idxs))
            ready = [i for i, ok in zip(idxs, results) if ok]
            for i, ok in zip(idxs, results):
                if not ok:
                    print(f"!! [ERROR] Drone {i + 1} could not launch; leaving it on the ground (try 'lift {i + 1}').")

            print(">> Ascending to cruising hover...")
            positions, reached = {}, set()
            t0 = time.time()
            while time.time() - t0 < 180.0 and len(reached) < len(ready):
                for i in ready:
                    if i in reached:
                        continue
                    latest = None
                    while True:
                        m = self.connections[i].recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                        if not m:
                            break
                        latest = m
                    if latest is not None:
                        positions[i] = self.frame.gps_to_global_ned(latest.lat / 1e7, latest.lon / 1e7, latest.relative_alt / 1000.0)
                        if latest.relative_alt / 1000.0 >= target_alt * 0.75:
                            reached.add(i)
                            print(f"  [{self.fleet[i].label}] Cruising altitude reached!")
                time.sleep(0.1)

            with self.lock:
                for i in reached:
                    self._mark_airborne(i, positions[i])
                self._calculate_active_topology()
                self.morph_steps_remaining = 40
            print(f"\n>> Swarm ready! {len(reached)} drone(s) airborne and forming up.\n")
            if self.evidence:
                self.evidence.event("launch_result", requested=[i + 1 for i in idxs], airborne=[i + 1 for i in reached],
                                    target_alt_m=target_alt)
        finally:
            for i in idxs:
                self.busy[i] = False

    def _mark_airborne(self, idx: int, cur_pos: Optional[np.ndarray]):
        """Caller holds self.lock. Start the drone's morph from where it really is, not from the centroid."""
        self.simulated_faults[idx] = False
        self.failure_causes[idx] = None
        self.recovery_stable_since[idx] = None
        self.recovery_attempted[idx] = False
        self.drone_health[idx] = True
        self.is_airborne[idx] = True
        self.last_heartbeat[idx] = time.time()
        self.last_real_heartbeat[idx] = 0.0  # wait for a heartbeat observed by the guidance reader
        if cur_pos is not None:
            self.last_pos[idx] = cur_pos
            self.last_pos_at[idx] = time.time()
            self.current_offsets[idx] = cur_pos - self.centroid_pos
        self.last_target_change = time.time()

    def _rejoin_corridor_clear(self, idx, now):
        """Conservative horizontal straight-line screen, not a trajectory planner."""
        if any(not 0 <= now - stamp <= 1.0 for stamp in self.last_pos_at):
            return False
        start = self.last_pos[idx][:2]
        end = (self.centroid_pos + self.target_offsets[idx])[:2]
        segment = end - start
        length2 = float(segment @ segment)
        for j in range(self.n):
            if j == idx:
                continue
            point = self.last_pos[j][:2]
            fraction = float(np.clip((point-start) @ segment / length2, 0, 1)) if length2 > 1e-10 else 0.
            if np.linalg.norm(point - (start + fraction * segment)) < 2.5:
                return False
        return True

    def liftoff_drone(self, drone_num: int, target_alt: Optional[float] = None, quiet_if_busy: bool = False):
        """Asynchronously arms and climbs one drone to altitude so it can (re)join the swarm."""
        if not self._valid_id(drone_num):
            return
        if self.landing_requested:
            return
        idx = drone_num - 1
        target_alt = self.CRUISE_ALT if target_alt is None else target_alt

        with self.lock:  # single-flight guard: never start a second worker or re-launch a flying drone
            if self.is_airborne[idx] and not self.simulated_faults[idx]:
                if not quiet_if_busy:
                    print(f">> Drone {drone_num} is already airborne.")
                return
            if self.busy[idx]:
                if not quiet_if_busy:
                    print(f">> Drone {drone_num} already has a liftoff in progress.")
                return
            if any(self.busy) or not self._rejoin_corridor_clear(idx, time.time()):
                print(">> Rejoin deferred: need fresh fleet positions and a clear 2.5 m horizontal corridor; one recovery at a time.")
                return
            if self.decentralized_guidance and idx in self.decentralized_guidance.agent_errors:
                print(">> Agent process failed; restart the session before rejoining this vehicle.")
                return
            self.busy[idx] = True
        if self.evidence:
            self.evidence.event("liftoff_requested", drone=drone_num, target_alt_m=target_alt)

        def _climb_worker():
            conn, label = self.connections[idx], self.fleet[idx].label
            try:
                print(f"\n>> [LIFTOFF] Arming and initiating climb for Drone {drone_num} to {target_alt:.1f}m...")
                if not wait_ekf_ready(conn, label, wall_timeout=self.ekf_timeout,
                                      cancelled=lambda: self.landing_requested):
                    print(f"!! [ERROR] Drone {drone_num}: EKF never converged. Liftoff aborted.")
                    return
                if self.landing_requested:
                    return
                if not arm_with_retry(conn, label, timeout=120.0, force_after=float("inf"),
                                      cancelled=lambda: self.landing_requested):
                    print(f"!! [ERROR] Drone {drone_num} failed to arm for liftoff. Check EKF status.")
                    return

                if self.landing_requested:
                    conn.set_mode(9)
                    return
                # NAV_TAKEOFF is only accepted while landed. If the drone is still in the air (revived
                # mid-LAND) skip it and let the formation setpoints fly it up to its slot instead.
                gp = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
                cur_alt = gp.relative_alt / 1000.0 if gp is not None else 0.0
                if cur_alt >= 1.0:
                    print(f">> Drone {drone_num} still airborne at {cur_alt:.1f}m; rejoining formation directly.")
                    self._rejoin(idx, gp)
                    print(f"\n* [REJOIN COMPLETE] Drone {drone_num} locked back into swarm!\n")
                    return

                conn.mav.command_long_send(
                    conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, float(target_alt),
                )
                print(f">> [ASCENDING] Drone {drone_num} airborne! Climbing to cruising altitude...")

                t_start, reached = time.time(), False
                while not self.landing_requested and time.time() - t_start < 180.0:  # generous wall budget for slow Gazebo RTF
                    m = conn.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.5)
                    if m:
                        alt = m.relative_alt / 1000.0 if m.get_type() == "GLOBAL_POSITION_INT" else -m.z
                        if alt >= target_alt * 0.85:
                            reached = True
                            break
                if not reached:
                    print(f"!! [ERROR] Drone {drone_num} did not reach altitude. Not joining formation.")
                    return
                self._rejoin(idx, conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0))
                print(f"\n* [REJOIN COMPLETE] Drone {drone_num} reached altitude and locked back into swarm!\n")
            finally:
                self.busy[idx] = False  # release the connection back to the guidance thread

        worker = threading.Thread(target=_climb_worker, daemon=True)
        self.recovery_workers.append(worker)
        worker.start()

    def _rejoin(self, idx: int, gp):
        if self.landing_requested:
            return
        cur_pos = None
        if gp is not None:
            cur_pos = self.frame.gps_to_global_ned(gp.lat / 1e7, gp.lon / 1e7, gp.relative_alt / 1000.0)
        with self.lock:
            self._mark_airborne(idx, cur_pos)
            # Failure handling reserves the absent aircraft's slot. Keep that
            # allocation through rejoin so other vehicles do not cross into it.
            self.morph_steps_remaining = self.MORPH_STEPS

    # ------------------------------------------------------------------ formation geometry & fault tolerance
    def _calculate_active_topology(self):
        """Assign the airborne drones to optimal slots of the current formation (Hungarian matching)."""
        active = self._active_indices()
        offsets = np.zeros((self.n, 3), dtype=float)
        if not active:
            self.target_offsets = offsets
            return

        slots = slots_for(self.current_formation_style, len(active))
        if self.control_mode == "decentralized":
            assignment = range(len(active))
        else:
            assignment = assign_slots(np.array([self.current_offsets[i] for i in active]), slots)
        for drone_idx, slot_idx in zip(active, assignment):
            offsets[drone_idx] = slots[slot_idx]

        # Leader bookkeeping is only used by the centralized console display.
        if self.control_mode == "decentralized":
            self.current_leader_idx = active[0]
        elif self.current_formation_style in ("v", "diamond"):
            for drone_idx, slot_idx in zip(active, assignment):
                if slot_idx == 0:
                    self.current_leader_idx = drone_idx
                    break
        elif self.current_leader_idx not in active:
            self.current_leader_idx = active[0]
        self.target_offsets = offsets

    def inject_failure(self, drone_num: int, cause: str = "operator"):
        """Simulates immediate failure / communication loss of Drone N (1-indexed)."""
        if not self._valid_id(drone_num):
            return
        idx = drone_num - 1
        with self.lock:
            if self.simulated_faults[idx]:
                print(f">> Drone {drone_num} is already disconnected.")
                return
            self.simulated_faults[idx] = True
            self.failure_causes[idx] = cause
            self.recovery_stable_since[idx] = None
            self.recovery_attempted[idx] = False
            self.drone_health[idx] = False
            self.is_airborne[idx] = False
            try:
                self.connections[idx].set_mode(9)  # LAND
            except Exception:
                pass

            old_leader = self.current_leader_idx
            active = self._active_indices()
            print("\n" + "!" * 70)
            print(f" [FAULT INJECTION] Drone {drone_num} signal lost / disconnected! Commanded LAND.")
            if self.control_mode == "decentralized":
                print(" [LOCAL PEER GRAPH] Decentralized guidance does not require a swarm leader.")
            elif idx == old_leader and self.control_mode == "centralized":
                if active:
                    self.current_leader_idx = active[0]
                    print(f" [ELECTION ALGORITHM] Leader lost! Dynamically promoting Drone {active[0] + 1} to APEX LEADER!")
                else:
                    print(" [ALERT] All drones in swarm have been lost.")
            elif self.control_mode == "centralized":
                print(f" [SWARM FAULT TOLERANCE] Follower Drone {drone_num} lost. Maintaining active formation.")
            else:
                print(f" [LOCAL RESPONSE] Drone {drone_num} is offline; active agents retain their assigned slots.")
            # Reserve every occupied/failed slot while the LAND sequence runs.
            # Reassigning into the failed vehicle's space before touchdown can
            # cause the landing aircraft and a healthy follower to converge.
            self.target_offsets = self.current_offsets.copy()
            self.morph_steps_remaining = 0
            self.last_target_change = time.time()
            if self.evidence:
                if cause == "heartbeat_timeout":
                    self.evidence.watchdog_timeouts += 1
                self.evidence.event("drone_failed", drone=drone_num, cause=cause,
                                    new_leader=(None if self.control_mode == "decentralized"
                                                else self.current_leader_idx + 1),
                                    remaining=[i + 1 for i in active])
            print(f" [SAFE RESPONSE] {len(active)} remaining agents hold their slots; Drone {drone_num}'s slot stays reserved.")
            print("!" * 70 + "\n")

    def revive_drone(self, drone_num: int, auto_liftoff: bool = True):
        """Restores a failed drone and triggers automatic re-arm + liftoff."""
        if not self._valid_id(drone_num):
            return
        print(f"\n>> [REVIVING] Telemetry link restored for Drone {drone_num}!")
        if self.evidence:
            self.evidence.event("revive_requested", drone=drone_num, auto_liftoff=auto_liftoff)
        if auto_liftoff:
            self.liftoff_drone(drone_num)
        else:
            with self.lock:
                self.simulated_faults[drone_num - 1] = False
                self.drone_health[drone_num - 1] = True
            print(f">> Drone {drone_num} marked online (Grounded). Type 'lift {drone_num}' to take off.")

    def set_formation_style(self, style: str):
        if style not in STYLES:
            print(f">> Unknown style '{style}'. Valid: {list(STYLES)}")
            return
        if self._mission_change_blocked():
            return
        if self.control_mode in ("decentralized", "hybrid") and self._active_indices():
            print(">> Mid-flight formation morphs are disabled in decentralized mode until collision-safe paths are validated.")
            return
        with self.lock:
            self.current_formation_style = style
            self._calculate_active_topology()
            self.morph_steps_remaining = self.MORPH_STEPS
            self.last_target_change = time.time()
        print(f">> [MORPH] Swarm morphing into {style.upper()} topology over {self.MORPH_STEPS / 10:.1f}s...")
        if self.evidence:
            self.evidence.event("formation_command", formation=style)

    # ------------------------------------------------------------------ guidance
    def send_formation_setpoints(self):
        with self.lock:
            centroid = self.centroid_pos.copy()
            offsets = self.current_offsets.copy()
            local_slots = self.target_offsets.copy()
            active = [self._active(i) for i in range(self.n)]
            positions = [p.copy() for p in self.last_pos]
            sample_times = self.last_pos_at.copy()
            fresh = [time.time() - t <= 1.0 for t in sample_times]
        targets = [(centroid + offsets[i]).tolist() if active[i] else None for i in range(self.n)]
        commanded = targets
        if self.decentralized_guidance is not None:
            commanded = self.decentralized_guidance.step(
                positions, sample_times, centroid, local_slots, active, fresh,
                coordinator_connected=(self.control_mode != "hybrid" or self.coordinator_connected),
            )
        if self.decentralized_guidance is not None:
            for idx, error in self.decentralized_guidance.agent_errors.items():
                if self._active(idx):
                    print(f">> Agent {idx+1} failed: {error}; requesting LAND.")
                    self.inject_failure(idx + 1, cause="agent_error")
                    if self.evidence:
                        self.evidence.event("agent_error", drone=idx+1, error=error)
                    commanded[idx] = None
        for i, conn in enumerate(self.connections):
            if not active[i] or commanded[i] is None:
                continue
            tgt_local = np.asarray(commanded[i]) - self.frame_origins[i]
            conn.mav.set_position_target_local_ned_send(
                0, conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                _TYPE_MASK_POS_YAW,
                float(tgt_local[0]), float(tgt_local[1]), float(tgt_local[2]),
                0, 0, 0, 0, 0, 0, self.FORMATION_YAW_RAD, 0,
            )
        return targets

    def guidance_loop(self):
        """
        10 Hz background thread: telemetry intake, heartbeat watchdog, morphing and setpoints.
        It is the only reader of a drone's connection unless a worker has claimed it via self.busy[i].
        """
        with self.lock:
            self.last_heartbeat = [time.time()] * self.n

        while self.running:
            for i, conn in enumerate(self.connections):
                if self.busy[i]:
                    self.last_heartbeat[i] = time.time()  # a worker owns this link and is talking to the drone
                    continue
                while True:
                    msg = conn.recv_match(blocking=False)
                    if not msg:
                        break
                    mtype = msg.get_type()
                    if mtype == "HEARTBEAT":
                        self.last_heartbeat[i] = time.time()
                        self.last_real_heartbeat[i] = self.last_heartbeat[i]
                        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        self.last_armed_state[i] = bool(msg.base_mode & armed_flag)
                    elif mtype == "GLOBAL_POSITION_INT":
                        self.last_pos[i] = self.frame.gps_to_global_ned(msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
                        self.last_pos_at[i] = time.time()

            now = time.time()
            self._consider_auto_recovery(now)
            self._check_watchdogs(now)

            with self.lock:
                if self.morph_steps_remaining > 0:
                    self.current_offsets += (self.target_offsets - self.current_offsets) / self.morph_steps_remaining
                    self.morph_steps_remaining -= 1
            targets = self.send_formation_setpoints()
            if self.evidence:
                with self.lock:
                    positions = [p.tolist() for p in self.last_pos]
                    active = [self._active(i) for i in range(self.n)]
                    fresh = [now - t <= 1.0 for t in self.last_pos_at]
                    ages = [round(now - t, 3) if t and active[i] and not self.busy[i] else None
                            for i, t in enumerate(self.last_real_heartbeat)]
                    formation = self.current_formation_style
                    leader = self.current_leader_idx + 1
                    settled = (self.morph_steps_remaining == 0 and self.settled_motion.update(
                        now, self.last_target_change, positions, self.last_pos_at, active, fresh
                    ))
                self.evidence.sample(positions, targets, active, fresh, ages, formation, leader, settled)
            time.sleep(0.1)

    def _check_watchdogs(self, now):
        for i in range(self.n):
            with self.lock:
                watched = self._active(i) and not self.busy[i]
            if not watched:
                continue
            if now - self.last_heartbeat[i] > self.hb_timeout:
                self.inject_failure(i + 1, cause="heartbeat_timeout")
            elif not 0 <= now - self.last_pos_at[i] <= 2.0:
                print(f"\n[STALE POSITION] Drone {i+1}: requesting LAND; automatic rejoin disabled.")
                self.inject_failure(i + 1, cause="telemetry_timeout")

    def _consider_auto_recovery(self, now: float):
        """Auto-rejoin only heartbeat-loss failures after stable link and confirmed disarm."""
        if not self.auto_recover or self.landing_requested:
            return
        for i in range(self.n):
            eligible = (
                self.simulated_faults[i]
                and self.failure_causes[i] == "heartbeat_timeout"
                and not self.recovery_attempted[i]
                and not self.busy[i]
                and self.last_armed_state[i] is False
                and 0.0 <= now - self.last_real_heartbeat[i] <= 1.0
            )
            if not eligible:
                self.recovery_stable_since[i] = None
                continue
            if self.recovery_stable_since[i] is None:
                self.recovery_stable_since[i] = now
                continue
            if now - self.recovery_stable_since[i] < 3.0:
                continue
            self.recovery_attempted[i] = True
            if self.evidence:
                self.evidence.event("auto_recovery_eligible", drone=i + 1,
                                    stable_link_s=round(now - self.recovery_stable_since[i], 2),
                                    disarmed=True)
            print(f"\n[AUTO RECOVERY] Drone {i + 1} link is stable and disarmed; starting EKF-gated staged rejoin.")
            self.revive_drone(i + 1, auto_liftoff=True)

    def stop_guidance(self):
        """Stop setpoint production and wait until it can no longer race LAND/cleanup."""
        self.running = False
        thread = self.guidance_thread
        if thread is not None and thread is not threading.current_thread():
            # The loop uses non-blocking telemetry reads and a 100 ms sleep, so it
            # should promptly finish its current iteration before shutdown proceeds.
            thread.join()

    def _mission_change_blocked(self):
        if self.landing_requested or any(self.simulated_faults) or any(self.busy):
            print(">> Mission held: finish recovery before moving or changing formation.")
            return True
        if self.control_mode == "hybrid" and not self.coordinator_connected:
            print(">> Coordinator disconnected: holding the last received mission.")
            return True
        return False

    def move(self, dn: float = 0.0, de: float = 0.0, d_alt: float = 0.0):
        if self._mission_change_blocked():
            return False
        with self.lock:
            self.centroid_pos[0] += dn
            self.centroid_pos[1] += de
            self.centroid_pos[2] -= d_alt
            self.last_target_change = time.time()
        print(f">> Vector Setpoint: N={self.centroid_pos[0]:.1f}m, E={self.centroid_pos[1]:.1f}m, Alt={-self.centroid_pos[2]:.1f}m")
        if self.evidence:
            self.evidence.event("centroid_command", north_m=float(self.centroid_pos[0]),
                                east_m=float(self.centroid_pos[1]), altitude_m=float(-self.centroid_pos[2]))

    def run_square_patrol(self):
        print("\n>>> EXECUTING AUTONOMOUS SQUARE PATROL <<<")
        if self.evidence:
            self.evidence.event("patrol_start")
        legs = [
            (8.0, 0.0, "Leg 1: North +8m"),
            (0.0, 6.0, "Leg 2: East +6m"),
            (-8.0, 0.0, "Leg 3: South -8m"),
            (0.0, -6.0, "Leg 4: West -6m (Return)"),
        ]
        for dn, de, name in legs:
            print(f" -> Commanding: {name}...")
            if self.move(dn, de) is False:
                print(">> Patrol stopped: mission change was blocked.")
                return
            for _ in range(6):
                time.sleep(1.0)
                self.print_status_line()
        print(f">>> Square Patrol Completed! Swarm held {self.current_formation_style.upper()} formation! <<<\n")
        if self.evidence:
            self.evidence.event("patrol_commands_complete")

    # ------------------------------------------------------------------ status / landing
    def print_status_line(self):
        with self.lock:
            pos = [p.copy() for p in self.last_pos]
            active = self._active_indices()
        spread = max((np.linalg.norm(pos[i][:2] - pos[j][:2]) for i in active for j in active if i < j), default=0.0)
        print(f"  [SWARM {len(active)}/{self.n} | {self.current_formation_style.upper()}] "
              f"Centroid: ({self.centroid_pos[0]:4.1f}N, {self.centroid_pos[1]:4.1f}E, {-self.centroid_pos[2]:4.1f}Alt) | Max spread={spread:4.2f}m")

    def print_status(self):
        with self.lock:
            positions = [p.copy() for p in self.last_pos]
            active_count = len(self._active_indices())
        print("\n" + "=" * 72)
        leadership = ("Distributed agents" if self.control_mode == "decentralized"
                      else "Hybrid supervisor" if self.control_mode == "hybrid"
                      else f"Apex Leader: Drone {self.current_leader_idx + 1}")
        print(f" SWARM: {active_count}/{self.n} Drones Airborne | {leadership}")
        print(f" Formation: {self.current_formation_style.upper()} | Origin: ({self.centroid_pos[0]:.1f}N, {self.centroid_pos[1]:.1f}E, {-self.centroid_pos[2]:.1f}m)")
        print("-" * 72)
        for i in range(self.n):
            is_active = self._active(i)
            status = "AIRBORNE" if is_active else ("GROUNDED" if not self.simulated_faults[i] else "LOST/FAILED")
            if self.control_mode == "hybrid" and self.decentralized_guidance is not None:
                agent = self.decentralized_guidance.agents[i]
                hybrid = agent.hybrid_controller
                role = (f"{hybrid.get_drone_mode(i).value} a={hybrid.get_alpha(i):.2f}"
                        if is_active else "INACTIVE")
            elif self.control_mode == "decentralized":
                role = "LOCAL AGENT" if is_active else "INACTIVE"
            else:
                role = "APEX LEADER" if i == self.current_leader_idx and is_active else ("WINGMAN" if is_active else "INACTIVE")
            p = positions[i]
            print(f"  Drone {i + 1}: [{status:10s}] Role: {role:11s} | N={p[0]:5.1f}m, E={p[1]:5.1f}m, Alt={-p[2]:4.1f}m")
        now = time.time()
        fresh_ids = [i for i in range(self.n) if 0 <= now - self.last_pos_at[i] <= 1.0]
        separation = min((np.linalg.norm(positions[i][:2] - positions[j][:2])
                          for i in fresh_ids for j in fresh_ids if i < j), default=None)
        print(f" Fresh telemetry: {len(fresh_ids)}/{self.n}; all-visible horizontal separation: {separation}")
        for i in range(self.n):
            if self.decentralized_guidance and self.decentralized_guidance.agent_runtime == "process":
                peers = len(self.decentralized_guidance.agents[i].last_peer_ids)
            else:
                peers = (len(self.decentralized_guidance.peer_bus.neighbors(i, now))
                         if self.decentralized_guidance else 0)
            recovery = "climbing" if self.busy[i] else ("slot reserved" if self.simulated_faults[i] else "none")
            print(f" D{i+1}: peers={peers}, telemetry age={now-self.last_pos_at[i]:.2f}s, recovery={recovery}")
        print("=" * 72 + "\n")

    def print_menu_bar(self):
        print("-" * 76)
        leadership = ("Distributed" if self.control_mode == "decentralized"
                      else "Hybrid" if self.control_mode == "hybrid"
                      else f"Apex Leader: D{self.current_leader_idx + 1}")
        link = f" | Coordinator: {'UP' if self.coordinator_connected else 'LOST'}" if self.control_mode == "hybrid" else ""
        print(f" [SWARM: {len(self._active_indices())}/{self.n} Online | {leadership}{link} | Shape: {self.current_formation_style.upper()}]")
        if self.control_mode == "decentralized":
            print(" [w/s/a/d] Move  [u/j] Alt  p Patrol  f <id> Fail  r <id> Rejoin  status  q Land")
        elif self.control_mode == "hybrid":
            print(" [w/s/a/d] Move  coord off/on  p Patrol  f <id> Fail  r <id> Rejoin  status  q Land")
        else:
            print(" [w/s/a/d] Move  [u/j] Alt  [v/l/k/x/o/e/tri] Shapes  p Patrol  f <id> Fail  r <id> Revive  lift <id>  q Land")
        print("-" * 76)

    def land_all(self, timeout: float = 180.0) -> bool:
        print("\n>> Synchronous LAND commanded for entire swarm...")
        if self.evidence:
            self.evidence.event("land_command", drones=self.n)
        self.landing_requested = True
        self.stop_guidance()
        for worker in self.recovery_workers:
            worker.join()
        for conn in self.connections:
            conn.set_mode(9)  # LAND
            conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + timeout
        last_report = 0.0
        while time.monotonic() < deadline:
            states = [_armed_state(conn) for conn in self.connections]
            if states and all(state is False for state in states):
                print(">> Landing confirmed: all vehicles disarmed after touchdown.")
                self.landed = True
                if self.evidence:
                    self.evidence.landing_confirmed = True
                    self.evidence.event("landing_confirmed")
                return True
            armed = [str(i + 1) for i, state in enumerate(states) if state is True]
            unknown = [str(i + 1) for i, state in enumerate(states) if state is None]
            now = time.monotonic()
            if now - last_report >= 5.0:
                print(f">> Landing in progress; armed={','.join(armed) or 'none'}, telemetry-missing={','.join(unknown) or 'none'}.")
                last_report = now
            time.sleep(0.5)
        print(">> LAND timeout: not all vehicles confirmed disarmed; stopping SITL after timeout.")
        if self.evidence:
            self.evidence.event("landing_timeout")
        return False


_SHAPE_ALIASES = {
    "v": "v", "l": "line", "line": "line", "k": "column", "col": "column", "column": "column",
    "x": "diamond", "diamond": "diamond", "o": "ring", "ring": "ring",
    "e": "echelon", "echelon": "echelon", "tri": "triangle", "triangle": "triangle",
}


def handle_command(console: SwarmConsole, cmd: str) -> bool:
    """Execute one pilot command. Returns False when the console should exit."""
    tokens = cmd.strip().lower().split()
    if not tokens:
        return True
    if console.evidence:
        console.evidence.event("pilot_command", command=cmd.strip())
    action = tokens[0]
    num = tokens[1] if len(tokens) > 1 else None
    val = float(num) if num is not None and num.replace(".", "", 1).replace("-", "", 1).isdigit() else None
    drone_id = int(num) if num is not None and num.isdigit() else None

    if action in ("w", "north"):
        console.move(dn=val if val is not None else 6.0)
    elif action in ("s", "south"):
        console.move(dn=-(val if val is not None else 6.0))
    elif action in ("a", "west"):
        console.move(de=-(val if val is not None else 5.0))
    elif action in ("d", "east"):
        console.move(de=val if val is not None else 5.0)
    elif action in ("u", "up"):
        console.move(d_alt=val if val is not None else 2.0)
    elif action in ("j", "down"):
        console.move(d_alt=-(val if val is not None else 2.0))
    elif action in ("c", "center"):
        if console._mission_change_blocked():
            return True
        with console.lock:
            console.centroid_pos = np.array([0.0, 0.0, -console.CRUISE_ALT])
            console.last_target_change = time.time()
        print(f">> Centroid returned to origin (0, 0, {console.CRUISE_ALT:.0f}m).")
    elif action in _SHAPE_ALIASES:
        console.set_formation_style(_SHAPE_ALIASES[action])
    elif action in ("p", "patrol"):
        console.run_square_patrol()
    elif action in ("fail", "f", "kill", "drop"):
        console.inject_failure(drone_id) if drone_id else print("Usage: f <drone_id>")
    elif action in ("revive", "r", "heal"):
        console.revive_drone(drone_id) if drone_id else print("Usage: r <drone_id>")
    elif action in ("takeoff", "launch", "fly", "go", "ready"):
        console.launch_swarm()
    elif action in ("lift", "liftoff", "t", "climb"):
        if drone_id:
            console.liftoff_drone(drone_id)
        elif num in ("all", "swarm"):
            console.launch_swarm()
        else:
            print("Usage: lift <drone_id> or 'lift all'")
    elif action in ("status", "stat", "info"):
        console.print_status()
    elif action in ("coord", "coordinator"):
        if console.control_mode != "hybrid":
            print(">> Coordinator link switching is available in hybrid mode.")
        elif num in ("off", "lost", "down"):
            console.coordinator_connected = False
            print(">> Coordinator link cut; agents will use local peer guidance and hold their last slots.")
            if console.evidence:
                console.evidence.event("coordinator_link", connected=False)
        elif num in ("on", "up", "restore"):
            console.coordinator_connected = True
            print(">> Coordinator link restored; hybrid recovery uses heartbeat hysteresis and a gradual blend.")
            if console.evidence:
                console.evidence.event("coordinator_link", connected=True)
        else:
            print("Usage: coord off | coord on")
    elif action in ("q", "exit", "land"):
        console.land_all()
        return False
    else:
        print("Commands: w a s d u j c | v l k x o e tri | p | f <id> r <id> lift <id> launch | status | q")
    return True


def _print_help(n: int, control_mode: str = "centralized", agent_runtime: str = "local"):
    print("\n" + "=" * 76)
    print(f"      SWARM FLIGHT CONSOLE ({n} DRONES)")
    print("=" * 76)
    print(" Movement (optional step size, e.g. 'w 10'):  w North  s South  a West  d East  u Climb  j Descend  c Center")
    print(" Formations:  v V  l Line  k Column  x Diamond  o Ring  e Echelon  tri Triangle")
    if control_mode == "decentralized":
        print(" Decentralized stage: choose the shape before takeoff; rejoin is staged into the reserved slot.")
        if agent_runtime == "process":
            print(" Each agent process exchanges one-hop peer state directly over impaired UDP loopback.")
            print(" The host still owns MAVLink telemetry, mission input, and initial slots/topology.")
        else:
            print(" Each local controller uses its own state and one-hop peer states (SITL host emulation).")
    elif control_mode == "hybrid":
        print(" Hybrid stage: central waypoints with per-agent peer-guidance fallback on coordinator loss.")
        print(" Use 'coord off' / 'coord on' to demonstrate fallback and hysteretic recovery.")
    print(" Mission:  launch (arm+takeoff+form up)   p Square patrol   status   q Land")
    if control_mode in ("decentralized", "hybrid"):
        print(" Fault test: f <id> land one drone; r <id> performs a staged recover and rejoin.")
    else:
        print(" Fault tolerance:  f <id> fail drone   r <id> revive + rejoin   lift <id> launch one drone")
    print("=" * 76)


def main():
    parser = argparse.ArgumentParser(description="Swarm Flight Console (ArduPilot SITL, standalone or Gazebo)")
    parser.add_argument("-n", "--drones", type=int, default=5, help=f"swarm size, 1-{MAX_DRONES} (default 5)")
    parser.add_argument("--gazebo", action="store_true", help="force Gazebo 3D mode (--model JSON)")
    parser.add_argument("--standalone", action="store_true", help="force standalone SITL (--model quad)")
    parser.add_argument("--restart", action="store_true", help="kill existing SITL processes and start fresh")
    parser.add_argument("--ekf-timeout", type=float, default=None, help="seconds to wait for the EKF (default 300, 900 in Gazebo)")
    parser.add_argument("--auto-launch", action="store_true", help="take off and form up immediately")
    parser.add_argument("--control-mode", choices=("centralized", "decentralized", "hybrid"), default="centralized",
                        help="decentralized and hybrid are experimental per-agent SITL modes")
    parser.add_argument("--peer-loss", type=float, default=0.0,
                        help="decentralized SITL only: independent one-hop packet loss probability (0-1)")
    parser.add_argument("--peer-latency-ms", type=float, default=30.0,
                        help="decentralized SITL only: mean one-hop peer-message latency in milliseconds")
    parser.add_argument("--peer-jitter-ms", type=float, default=None,
                        help="independent runtime only: standard deviation of one-hop peer delay in milliseconds (default 5 when mean delay is nonzero)")
    parser.add_argument("--peer-seed", type=int, default=None,
                        help="decentralized SITL only: reproducible peer-channel random seed")
    parser.add_argument("--peer-burst-loss", action="store_true",
                        help="decentralized/hybrid SITL only: add bursty Gilbert-Elliott packet loss")
    parser.add_argument("--peer-duplicate-rate", type=float, default=0.0,
                        help="independent runtime only: duplicate probability per peer datagram (0-1)")
    parser.add_argument("--peer-reorder-rate", type=float, default=0.0,
                        help="independent runtime only: probability of extra delay that can reorder peer datagrams (0-1)")
    parser.add_argument("--peer-reorder-delay-ms", type=float, default=150.0,
                        help="independent runtime only: variable extra delay scale for reordered datagrams")
    parser.add_argument("--peer-roster", type=Path, default=None,
                        help="independent runtime only: shared JSON file with per-agent peer bind/advertised addresses")
    parser.add_argument("--peer-auth-key-file", type=Path, default=None,
                        help="independent runtime only: shared HMAC key file for direct peer datagrams")
    parser.add_argument("--external-agents", action="store_true",
                        help="independent runtime only: run operator console without launching SITL or agent processes")
    parser.add_argument("--operator-bind-host", default="127.0.0.1",
                        help="independent runtime only: local interface for agent status UDP sockets (use 0.0.0.0 for remote agents)")
    parser.add_argument("--agent-runtime", choices=("local", "process", "independent"), default="local",
                        help="run experimental controllers in processes, or give every drone its own MAVLink connection")
    parser.add_argument("--formation", choices=tuple(STYLES), default="v",
                        help="starting formation for independent agent runtime")
    parser.add_argument("--auto-recover", action="store_true",
                        help="after heartbeat loss, rejoin only when the link is stable 3s and disarm is confirmed")
    parser.add_argument("--peer-block", type=str, default=None,
                        help="decentralized SITL only: directed link(s) to drop, e.g. 2:4 or 1:2,2:1")
    parser.add_argument("--sitl-gcs-failsafe", action="store_true",
                        help="independent runtime only: enable ArduPilot SITL LAND after 5s without that agent's GCS heartbeat")
    parser.add_argument("--sitl-sensor-profile", choices=("baseline", "stress"), default="baseline",
                        help="independent runtime only: use run-local SITL sensor/environment profile; stress values are assumptions, not hardware-calibrated")
    parser.add_argument("--evidence-dir", type=Path, default=None, help="parent folder for per-run JSONL and summary (default runs/console)")
    args = parser.parse_args()
    if not 1 <= args.drones <= MAX_DRONES:
        parser.error(f"--drones must be between 1 and {MAX_DRONES}")
    if (not math.isfinite(args.peer_loss) or not 0.0 <= args.peer_loss <= 1.0 or
            not math.isfinite(args.peer_latency_ms) or args.peer_latency_ms < 0):
        parser.error("--peer-loss must be 0..1 and --peer-latency-ms finite and non-negative")
    if (args.peer_jitter_ms is not None and
            (not math.isfinite(args.peer_jitter_ms) or args.peer_jitter_ms < 0)):
        parser.error("--peer-jitter-ms must be finite and non-negative")
    if args.peer_jitter_ms is not None and args.agent_runtime != "independent":
        parser.error("--peer-jitter-ms requires --agent-runtime independent")
    if (not math.isfinite(args.peer_duplicate_rate) or
            not 0.0 <= args.peer_duplicate_rate <= 1.0 or
            not math.isfinite(args.peer_reorder_rate) or
            not 0.0 <= args.peer_reorder_rate <= 1.0 or
            not math.isfinite(args.peer_reorder_delay_ms) or
            args.peer_reorder_delay_ms < 0):
        parser.error("peer duplicate/reorder rates must be finite 0..1 and extra delay finite/non-negative")
    if (args.peer_duplicate_rate or args.peer_reorder_rate) and args.agent_runtime != "independent":
        parser.error("peer duplicate/reorder injection currently requires --agent-runtime independent")
    if (args.peer_roster is not None or args.peer_auth_key_file is not None) and args.agent_runtime != "independent":
        parser.error("peer roster and authentication options require --agent-runtime independent")
    if args.agent_runtime in ("process", "independent") and args.control_mode == "centralized":
        parser.error(f"--agent-runtime {args.agent_runtime} requires decentralized or hybrid control")
    if args.agent_runtime == "independent":
        if args.control_mode != "decentralized":
            parser.error("--agent-runtime independent currently supports --control-mode decentralized only")
        if args.auto_recover:
            parser.error("--auto-recover is not implemented for independent UDP nodes yet")
        if args.external_agents and (args.peer_roster is None or args.peer_auth_key_file is None):
            parser.error("--external-agents requires --peer-roster and --peer-auth-key-file")
        if args.external_agents and (args.peer_loss or args.peer_latency_ms != 30.0 or
                                     args.peer_burst_loss or args.peer_seed is not None or
                                     args.peer_block or args.peer_duplicate_rate or
                                     args.peer_reorder_rate or args.peer_jitter_ms is not None or
                                     args.sitl_gcs_failsafe or
                                     args.sitl_sensor_profile != "baseline"):
            parser.error("network fault injection is configured on external agents, not the operator console")
        if args.peer_block:
            try:
                links = [tuple(int(value) for value in link.split(":"))
                         for link in args.peer_block.split(",")]
            except (TypeError, ValueError):
                parser.error("--peer-block must contain directed links like 2:4,3:5 (1-indexed IDs)")
            if any(len(link) != 2 or not (1 <= link[0] <= args.drones and
                                           1 <= link[1] <= args.drones) or link[0] == link[1]
                   for link in links):
                parser.error("--peer-block IDs must be different valid drone IDs")
        from sitl.independent_swarm_console import run_independent_console
        run_independent_console(args)
        return
    if args.sitl_gcs_failsafe or args.sitl_sensor_profile != "baseline":
        parser.error("--sitl-gcs-failsafe and --sitl-sensor-profile require --agent-runtime independent")
    if args.sitl_gcs_failsafe:
        parser.error("--sitl-gcs-failsafe is currently supported only with --agent-runtime independent")
    use_gazebo = True if args.gazebo else (False if args.standalone else None)

    console = SwarmConsole(n=args.drones, use_gazebo=use_gazebo, force_restart=args.restart,
                           ekf_timeout=args.ekf_timeout, control_mode=args.control_mode,
                           peer_loss=args.peer_loss, peer_latency_ms=args.peer_latency_ms,
                           peer_seed=args.peer_seed, peer_block=args.peer_block,
                           peer_burst_loss=args.peer_burst_loss, auto_recover=args.auto_recover, agent_runtime=args.agent_runtime)
    mode = "gazebo" if use_gazebo else "standalone" if use_gazebo is False else "auto"
    console.evidence = RunEvidence(console.repo_root, args.drones, mode, console.hb_timeout, args.evidence_dir)
    console.evidence.event("control_mode", mode=args.control_mode,
                           architecture=(
                               f"{args.control_mode} per-agent processes with direct impaired UDP peer state; host-owned MAVLink"
                               if args.agent_runtime == "process" else
                               "hybrid supervisory waypoints with local peer fallback"
                               if args.control_mode == "hybrid" else
                               "host-emulated per-agent 1-hop guidance over impaired peer bus"
                           ),
                           peer_loss=args.peer_loss, peer_latency_ms=args.peer_latency_ms,
                           peer_seed=args.peer_seed, peer_block=args.peer_block,
                           peer_burst_loss=args.peer_burst_loss,
                           peer_duplicate_rate=args.peer_duplicate_rate,
                           peer_reorder_rate=args.peer_reorder_rate,
                           peer_reorder_delay_ms=args.peer_reorder_delay_ms,
                           auto_recover=args.auto_recover, agent_runtime=args.agent_runtime)
    print(f">> Run evidence: {console.evidence.directory}")
    try:
        console.launch_sitl()
        console.connect_all()
        console.sync_frames()
        console.guidance_thread = threading.Thread(target=console.guidance_loop, daemon=True)
        console.guidance_thread.start()

        _print_help(args.drones, args.control_mode, args.agent_runtime)
        if args.auto_launch:
            console.launch_swarm(wait=True)
        else:
            print("\n>> All drones are on the ground and ready. Type 'launch' to take off.\n")

        while True:
            console.print_menu_bar()
            try:
                if not handle_command(console, input("Swarm Command > ")):
                    break
            except EOFError:
                console.land_all()
                break
            except Exception as exc:
                console.evidence.event("command_error", error=str(exc))
                traceback.print_exc()
                print(f"!! Command failed: {exc} (console is still running)")
    except KeyboardInterrupt:
        print("\n>> Interrupted by pilot. Shutting down...")
        console.land_all()
    except Exception as exc:
        console.evidence.event("run_error", error=str(exc))
        raise
    finally:
        console.stop_guidance()
        if console.decentralized_guidance is not None:
            channel = console.decentralized_guidance.peer_bus.channel
            console.evidence.event("peer_bus_summary", transmitted=channel.total_transmitted,
                                   delivered=channel.total_delivered,
                                   dropped_loss=channel.total_dropped_loss,
                                   dropped_range=channel.total_dropped_range,
                                   dropped_stale_source=console.decentralized_guidance.peer_bus.rejected_stale_source,
                                   rejected_out_of_order=console.decentralized_guidance.peer_bus.rejected_out_of_order,
                                   blocked_links=sorted(console.decentralized_guidance.peer_bus.blocked_links))
            if console.decentralized_guidance.agent_runtime == "process":
                agents = console.decentralized_guidance.agents
                keys = ("queued", "sent", "dropped_loss", "received", "rejected_stale",
                        "rejected_replay", "rejected_invalid")
                totals = {key: sum(int(agent.last_peer_metrics.get(key, 0)) for agent in agents)
                          for key in keys}
                console.evidence.event(
                    "direct_peer_udp_summary",
                    agents=[{"drone": i + 1,
                             "last_peers": list(agent.last_peer_ids),
                             **agent.last_peer_metrics}
                            for i, agent in enumerate(agents)],
                    totals=totals,
                )
            if console.control_mode == "hybrid":
                console.evidence.event("hybrid_summary", coordinator_connected=console.coordinator_connected,
                                       agents=[{
                                           "drone": i + 1,
                                           "mode": agent.hybrid_controller.get_drone_mode(i).value,
                                           "alpha": agent.hybrid_controller.get_alpha(i),
                                           "mode_switches": agent.hybrid_controller.switch_counts.get(i, 0),
                                       } for i, agent in enumerate(console.decentralized_guidance.agents)])
        try:
            console.cleanup()
        finally:
            console.evidence.close("landed" if console.landed else "incomplete")


if __name__ == "__main__":
    main()
