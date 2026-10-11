#!/usr/bin/env python3
"""
Self-Healing Autonomous Swarm Flight Console (Resilient 5-Drone Hybrid Architecture).

Advanced Features:
- Decentralized Fault-Tolerance: Monitors individual drone heartbeat & telemetry at 10 Hz.
- Dynamic Leader Election: Automatically re-elects the Apex Leader if Drone 1 drops out.
- Topology Reconfiguration: Dynamically morphs formation geometry (5 -> 4 -> 3 -> 2 -> 1).
- Live Failure Injection: 'fail <id>' (alias 'f <id>') simulates immediate drone loss.
- High-Efficiency Liftoff & Re-joining:
    - 'revive <id>' (alias 'r <id>') clears fault and automatically triggers re-arm + liftoff climb!
    - 'liftoff <id>' (alias 't <id>' / 'lift <id>') independently commands any grounded drone to arm & climb back to formation.
- Streamlined Command Parser: Supports rapid shortcodes, distance multipliers (e.g. 'w 12', 'u 4'), and instant response.
"""

import atexit
import itertools
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
from typing import Dict, List, Optional
import numpy as np
from pymavlink import mavutil
try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    linear_sum_assignment = None

from sitl.common_frame import CommonCoordinateFrame
from sitl.flight_prep import arm_with_retry, takeoff_and_verify, wait_ekf_ready


class SelfHealingSwarmConsole:
    FORMATION_YAW_RAD = 0.0

    def __init__(self, use_gazebo: Optional[bool] = None, force_restart: bool = False):
        self.repo_root = Path(__file__).resolve().parent.parent
        self.datum_lat = -35.363261
        self.datum_lon = 149.165230
        self.datum_alt = 584.0
        self.frame = CommonCoordinateFrame(self.datum_lat, self.datum_lon, self.datum_alt)
        self.use_gazebo = use_gazebo
        self.force_restart = force_restart

        self.gazebo_home = "-35.363261,149.165230,584,0"
        self.drone_configs = [
            {"id": 0, "inst": 0, "port": 14552, "udp": True, "home": "-35.363261,149.165230,584,0", "label": "Drone 1 (SysID 1)"},
            {"id": 1, "inst": 1, "port": 14562, "udp": True, "home": "-35.363283,149.165208,584,0", "label": "Drone 2 (SysID 2)"},
            {"id": 2, "inst": 2, "port": 14572, "udp": True, "home": "-35.363283,149.165252,584,0", "label": "Drone 3 (SysID 3)"},
            {"id": 3, "inst": 3, "port": 14582, "udp": True, "home": "-35.363306,149.165186,584,0", "label": "Drone 4 (SysID 4)"},
            {"id": 4, "inst": 4, "port": 14592, "udp": True, "home": "-35.363306,149.165274,584,0", "label": "Drone 5 (SysID 5)"},
        ]

        # Resilient Swarm States
        self.drone_health = [True] * 5          # True = healthy/active, False = failed/disconnected
        self.simulated_faults = [False] * 5     # True if pilot injected failure
        self.is_airborne = [False] * 5          # True if hovering at cruising altitude
        self.last_heartbeat = [time.time()] * 5
        self.current_leader_idx = 0             # Initially Drone 1

        # busy[i]: a worker thread owns drone i's MAVLink connection (guidance thread must not read it)
        self.busy = [False] * 5
        self.last_pos = [np.zeros(3) for _ in range(5)]  # latest global NED per drone (guidance thread)
        # Heartbeats are paced by *sim* time: ~1 per 5.7s wall at 17.6% RTF, so the 2D timeout would false-trip.
        gazebo_mode = use_gazebo if use_gazebo is not None else self.detect_gazebo()
        self.hb_timeout = 25.0 if gazebo_mode else 3.5

        # Formation Topologies
        self.current_formation_style = "v"      # "v", "line", "column", "diamond", "ring"
        self.current_offsets = np.zeros((5, 3), dtype=float)
        self.target_offsets = np.zeros((5, 3), dtype=float)
        self.morph_steps_remaining = 0

        self.connections: List[mavutil.mavlink_connection] = []
        self.procs = []
        self.frame_origins = []
        self.centroid_pos = np.array([0.0, 0.0, -5.0])  # [North, East, Down]
        self.running = True
        self.lock = threading.Lock()

        self._calculate_active_topology()
        self.current_offsets = self.target_offsets.copy()

    @staticmethod
    def detect_gazebo() -> bool:
        try:
            res = subprocess.run(["pgrep", "-f", "gz sim"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return res.returncode == 0
        except Exception:
            return False

    @staticmethod
    def kill_existing_sitl():
        try:
            subprocess.run(["pkill", "-9", "-f", "arducopter"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
        except Exception:
            pass

    def launch_sitl(self):
        is_gazebo = self.use_gazebo if self.use_gazebo is not None else self.detect_gazebo()

        if self.force_restart:
            print(">> [--restart] Cleaning up prior SITL processes...")
            self.kill_existing_sitl()

        already_running = False
        try:
            test_conn = mavutil.mavlink_connection("udpin:127.0.0.1:14552")
            msg = test_conn.wait_heartbeat(timeout=1.5)
            if msg:
                print(">> [OK] Detected active SITL instances on UDP 14552..14592.")
                print(">> Connecting directly to existing 5 drones.")
                already_running = True
            test_conn.close()
        except Exception:
            pass

        if already_running:
            return

        sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter")))
        if not sitl_bin.exists():
            raise FileNotFoundError(f"ArduCopter binary not found at: {sitl_bin}")

        params_file = str(self.repo_root / "sitl" / "swarm_params.parm")
        if is_gazebo:
            print(">> [3D GAZEBO] Launching SITL JSON bridge to Gazebo Ports 9002, 9012, 9022, 9032, 9042...")
        else:
            print(">> [2D STANDALONE] Launching 5 ArduPilot SITL quad physics instances...")

        for cfg in self.drone_configs:
            if is_gazebo:
                cmd = [
                    str(sitl_bin),
                    f"-I{cfg['inst']}",
                    "--model", "JSON",
                    "--home", self.gazebo_home,
                    "--defaults", params_file,
                    "--sim-address", "127.0.0.1",
                    "--speedup", "1",
                ]
            else:
                cmd = [
                    str(sitl_bin),
                    f"-I{cfg['inst']}",
                    "--model", "quad",
                    "--home", cfg["home"],
                    "--defaults", params_file,
                    "--speedup", "1",
                ]
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.procs.append(p)

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

    def connect_all(self):
        print(">> Establishing MAVLink links to all 5 drones...")
        self.connections.clear()
        for cfg in self.drone_configs:
            endpoint = f"udpin:127.0.0.1:{cfg['port']}"
            conn = None
            for attempt in range(60):
                try:
                    conn = mavutil.mavlink_connection(endpoint)
                    msg = conn.wait_heartbeat(timeout=3.0)
                    if msg:
                        print(f"  [ONLINE] {cfg['label']} connected on {endpoint} (SysID={conn.target_system})")
                        break
                    conn = None
                except Exception:
                    conn = None
                    time.sleep(0.5)
            if not conn:
                raise RuntimeError(
                    f"Failed to connect to {cfg['label']} on {endpoint}.\n"
                    f"Make sure SITL drones are running via: bash sitl/start_swarm_5drones_2d.sh"
                )
            self.connections.append(conn)

            conn.mav.param_set_send(
                conn.target_system, conn.target_component,
                b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            conn.mav.param_set_send(
                conn.target_system, conn.target_component,
                b"GUID_TIMEOUT", 3.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
            )

        print(">> Waiting for EKF convergence (position + GPS 3D fix) on all vehicles...")
        from concurrent.futures import ThreadPoolExecutor

        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(wait_ekf_ready, conn, self.drone_configs[i]["label"], 300.0)
                for i, conn in enumerate(self.connections)
            ]
            if not all(f.result() for f in futures):
                raise RuntimeError("EKF did not converge on one or more drones. Check Gazebo RTF / SITL logs.")

    def takeoff_and_assemble(self):
        print("\n>> Synchronous ARM & TAKEOFF to 5.0m altitude across all 5 drones...")
        from concurrent.futures import ThreadPoolExecutor

        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(arm_with_retry, conn, self.drone_configs[i]["label"])
                for i, conn in enumerate(self.connections)
            ]
            if not all(f.result() for f in futures):
                raise RuntimeError("Arming pre-checks failed on one or more drones.")

        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(takeoff_and_verify, conn, self.drone_configs[i]["label"], 5.0)
                for i, conn in enumerate(self.connections)
            ]
            if not all(f.result() for f in futures):
                raise RuntimeError("Takeoff trajectory interrupted.")

        print(">> Ascending to 5.0m cruising hover...")
        t_climb = time.time()
        hover_flags = [False] * 5
        while time.time() - t_climb < 120.0 and not all(hover_flags):
            for i, conn in enumerate(self.connections):
                if not hover_flags[i]:
                    latest_m = None
                    while True:
                        m = conn.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=False)
                        if not m:
                            break
                        latest_m = m
                    if latest_m:
                        alt = (
                            latest_m.relative_alt / 1000.0
                            if latest_m.get_type() == "GLOBAL_POSITION_INT"
                            else -latest_m.z
                        )
                        if alt >= 3.8:
                            hover_flags[i] = True
                            self.is_airborne[i] = True
                            print(f"  [{self.drone_configs[i]['label']}] Cruising altitude reached ({alt:.2f}m)!")
                        elif time.time() - t_climb > 6.0 and alt < 1.0:
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, 5.0
                            )
            time.sleep(0.2)

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
                    elif m.get_type() == "GLOBAL_POSITION_INT":
                        glob_msg = m
                if loc_msg and glob_msg:
                    break

            if not (loc_msg and glob_msg):
                raise RuntimeError(
                    f"No LOCAL/GLOBAL position telemetry from {self.drone_configs[i]['label']}; "
                    f"refusing to use a zero frame origin."
                )
            g_ned = self.frame.gps_to_global_ned(glob_msg.lat / 1e7, glob_msg.lon / 1e7, glob_msg.relative_alt / 1000.0)
            l_ned = np.array([loc_msg.x, loc_msg.y, loc_msg.z])
            self.frame_origins.append(g_ned - l_ned)

        print(">> Locking initial 5-drone Flying-V formation...")
        for _ in range(30):
            self.send_formation_setpoints()
            time.sleep(0.1)
        print(">> Swarm stabilized! Self-Healing Fault Tolerant controller active.\n")

    def sync_frames(self):
        """Compute each drone's NED offset to the shared datum (works on the ground)."""
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
                    elif m.get_type() == "GLOBAL_POSITION_INT":
                        glob_msg = m
                if loc_msg and glob_msg:
                    break
            if not (loc_msg and glob_msg):
                raise RuntimeError(
                    f"No LOCAL/GLOBAL position telemetry from {self.drone_configs[i]['label']}; "
                    f"refusing to use a zero frame origin."
                )
            g_ned = self.frame.gps_to_global_ned(glob_msg.lat / 1e7, glob_msg.lon / 1e7, glob_msg.relative_alt / 1000.0)
            l_ned = np.array([loc_msg.x, loc_msg.y, loc_msg.z])
            self.frame_origins.append(g_ned - l_ned)

    def launch_swarm(self, target_alt: float = 5.0):
        """'launch': arm, take off and form up every grounded, healthy drone together."""
        with self.lock:
            idxs = [
                i for i in range(5)
                if not self.is_airborne[i] and not self.simulated_faults[i] and not self.busy[i]
            ]
            for i in idxs:
                self.busy[i] = True  # claim the connections for the launch worker
        if not idxs:
            print(">> Nothing to launch (all drones airborne, failed, or already launching). Use 'r <id>' for failed drones.")
            return
        threading.Thread(target=self._launch_worker, args=(idxs, target_alt), daemon=True).start()

    def _launch_worker(self, idxs, target_alt):
        from concurrent.futures import ThreadPoolExecutor

        try:
            print(f"\n>> [LAUNCH] Arming and taking off Drones {[i + 1 for i in idxs]} to {target_alt:.1f}m...")

            def prep(i):
                conn, label = self.connections[i], self.drone_configs[i]["label"]
                return (
                    wait_ekf_ready(conn, label, 300.0)
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
                        positions[i] = self.frame.gps_to_global_ned(
                            latest.lat / 1e7, latest.lon / 1e7, latest.relative_alt / 1000.0
                        )
                        if latest.relative_alt / 1000.0 >= target_alt * 0.75:
                            reached.add(i)
                            print(f"  [{self.drone_configs[i]['label']}] Cruising altitude reached!")
                time.sleep(0.1)

            with self.lock:
                for i in reached:
                    self.drone_health[i] = True
                    self.is_airborne[i] = True
                    self.last_heartbeat[i] = time.time()
                    # Start the morph from where the drone really is, not from the centroid.
                    self.current_offsets[i] = positions[i] - self.centroid_pos
                self._calculate_active_topology()
                self.morph_steps_remaining = 40
            print(f"\n>> Swarm ready! {len(reached)} drone(s) airborne and forming up.\n")
        finally:
            for i in idxs:
                self.busy[i] = False

    # =========================================================================
    # Resilient Formation Geometry & Dynamic Election Algorithms
    # =========================================================================
    def _calculate_active_topology(self):
        active_indices = [
            i for i in range(5)
            if self.drone_health[i] and not self.simulated_faults[i] and self.is_airborne[i]
        ]
        n = len(active_indices)
        offsets = np.zeros((5, 3), dtype=float)

        if n == 0:
            self.target_offsets = offsets
            return

        style = self.current_formation_style

        # Template slot coordinates for candidate geometries
        if style == "v":
            if n == 5:
                slots = np.array([[0.0, 0.0, 0.0], [-3.0, -3.0, 0.0], [-3.0, 3.0, 0.0], [-6.0, -6.0, 0.0], [-6.0, 6.0, 0.0]])
            elif n == 4:
                slots = np.array([[0.0, 0.0, 0.0], [-3.0, -3.0, 0.0], [-3.0, 3.0, 0.0], [-5.0, 0.0, 0.0]])
            elif n == 3:
                slots = np.array([[0.0, 0.0, 0.0], [-3.0, -3.0, 0.0], [-3.0, 3.0, 0.0]])
            elif n == 2:
                slots = np.array([[0.0, -2.5, 0.0], [0.0, 2.5, 0.0]])
            else:
                slots = np.array([[0.0, 0.0, 0.0]])
        elif style == "line":
            if n == 5:
                slots = np.array([[0.0, -6.0, 0.0], [0.0, -3.0, 0.0], [0.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 6.0, 0.0]])
            elif n == 4:
                slots = np.array([[0.0, -4.5, 0.0], [0.0, -1.5, 0.0], [0.0, 1.5, 0.0], [0.0, 4.5, 0.0]])
            elif n == 3:
                slots = np.array([[0.0, -3.0, 0.0], [0.0, 0.0, 0.0], [0.0, 3.0, 0.0]])
            elif n == 2:
                slots = np.array([[0.0, -2.5, 0.0], [0.0, 2.5, 0.0]])
            else:
                slots = np.array([[0.0, 0.0, 0.0]])
        elif style == "column":
            if n == 5:
                slots = np.array([[6.0, 0.0, 0.0], [3.0, 0.0, 0.0], [0.0, 0.0, 0.0], [-3.0, 0.0, 0.0], [-6.0, 0.0, 0.0]])
            elif n == 4:
                slots = np.array([[4.5, 0.0, 0.0], [1.5, 0.0, 0.0], [-1.5, 0.0, 0.0], [-4.5, 0.0, 0.0]])
            elif n == 3:
                slots = np.array([[3.0, 0.0, 0.0], [0.0, 0.0, 0.0], [-3.0, 0.0, 0.0]])
            elif n == 2:
                slots = np.array([[2.5, 0.0, 0.0], [-2.5, 0.0, 0.0]])
            else:
                slots = np.array([[0.0, 0.0, 0.0]])
        elif style == "diamond":
            if n == 5:
                slots = np.array([[4.0, 0.0, 0.0], [0.0, -4.0, 0.0], [0.0, 4.0, 0.0], [-4.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
            elif n == 4:
                slots = np.array([[3.5, 0.0, 0.0], [0.0, -3.5, 0.0], [0.0, 3.5, 0.0], [-3.5, 0.0, 0.0]])
            elif n == 3:
                slots = np.array([[2.5, 0.0, 0.0], [-2.5, -3.0, 0.0], [-2.5, 3.0, 0.0]])
            elif n == 2:
                slots = np.array([[0.0, -2.5, 0.0], [0.0, 2.5, 0.0]])
            else:
                slots = np.array([[0.0, 0.0, 0.0]])
        else:  # ring
            r = 4.0
            slots = np.array([
                [r * math.cos(k * 2 * math.pi / n - math.pi / 2), r * math.sin(k * 2 * math.pi / n - math.pi / 2), 0.0]
                for k in range(n)
            ])

        # Solve optimal bipartite matching (Hungarian Algorithm) to minimize
        # total travel distance and strictly prevent path crossovers / collisions.
        curr_pts = np.array([self.current_offsets[i] for i in active_indices])
        cost_matrix = np.zeros((n, n), dtype=float)
        for i in range(n):
            for j in range(n):
                diff = curr_pts[i] - slots[j]
                cost_matrix[i, j] = diff[0]**2 + diff[1]**2 + 2.0 * diff[2]**2

        if linear_sum_assignment is not None:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
        else:
            best_perm = None
            min_c = float('inf')
            for p in itertools.permutations(range(n)):
                c = sum(cost_matrix[i, p[i]] for i in range(n))
                if c < min_c:
                    min_c = c
                    best_perm = p
            row_ind = list(range(n))
            col_ind = list(best_perm)

        for r, c in zip(row_ind, col_ind):
            offsets[active_indices[r]] = slots[c]

        # In apex-centered formations (V, diamond), the drone matched to slot 0 is the apex leader
        if style in ["v", "diamond"]:
            for r, c in zip(row_ind, col_ind):
                if c == 0:
                    self.current_leader_idx = active_indices[r]
                    break
        elif self.current_leader_idx not in active_indices:
            self.current_leader_idx = active_indices[0]

        self.target_offsets = offsets

    def inject_failure(self, drone_num: int):
        """Simulates immediate failure / communication loss of Drone N (1-indexed)."""
        idx = drone_num - 1
        if idx < 0 or idx >= 5:
            print(f">> Error: Drone ID must be 1 to 5. Received: {drone_num}")
            return

        with self.lock:
            if self.simulated_faults[idx]:
                print(f">> Drone {drone_num} is already disconnected.")
                return

            self.simulated_faults[idx] = True
            self.drone_health[idx] = False
            self.is_airborne[idx] = False

            # Command disconnected drone to LAND immediately
            try:
                conn = self.connections[idx]
                conn.set_mode(9)  # LAND mode
            except Exception:
                pass

            old_leader = self.current_leader_idx
            active_indices = [
                i for i in range(5)
                if not self.simulated_faults[i] and self.drone_health[i] and self.is_airborne[i]
            ]

            print("\n" + "!" * 70)
            print(f" [FAULT INJECTION] Drone {drone_num} signal lost / disconnected! Commanded LAND.")
            if idx == old_leader:
                if active_indices:
                    new_leader = active_indices[0]
                    self.current_leader_idx = new_leader
                    print(f" [ELECTION ALGORITHM] Leader lost! Dynamically promoting Drone {new_leader + 1} to APEX LEADER!")
                else:
                    print(" [ALERT] All drones in swarm have been lost.")
            else:
                print(f" [SWARM FAULT TOLERANCE] Follower Drone {drone_num} lost. Maintaining active formation.")

            self._calculate_active_topology()
            self.morph_steps_remaining = 45
            print(f" [SELF-HEALING] Reconfiguring remaining {len(active_indices)} drones into optimal topology...")
            print("!" * 70 + "\n")

    def liftoff_drone(self, drone_num: int, target_alt: float = 5.0, quiet_if_busy: bool = False):
        """Asynchronously arms and climbs Drone N to target altitude to rejoin the swarm."""
        idx = drone_num - 1
        if idx < 0 or idx >= 5:
            print(f">> Error: Drone ID must be 1 to 5. Received: {drone_num}")
            return

        # Single-flight guard: never start a second worker or re-launch a flying drone.
        with self.lock:
            if self.is_airborne[idx] and not self.simulated_faults[idx]:
                if not quiet_if_busy:
                    print(f">> Drone {drone_num} is already airborne.")
                return
            if self.busy[idx]:
                if not quiet_if_busy:
                    print(f">> Drone {drone_num} already has a liftoff in progress.")
                return
            self.busy[idx] = True  # claim the MAVLink connection for this worker

        def _climb_worker():
            conn = self.connections[idx]
            label = self.drone_configs[idx]["label"]
            try:
                print(f"\n>> [LIFTOFF] Arming and initiating climb for Drone {drone_num} to {target_alt:.1f}m...")

                # 1. Wait for a converged EKF (adapts to any Gazebo real-time factor)
                if not wait_ekf_ready(conn, label, wall_timeout=300.0):
                    print(f"!! [ERROR] Drone {drone_num}: EKF never converged. Liftoff aborted.")
                    return

                # 2. GUIDED + arm (EKF is ready, so this should succeed quickly)
                armed = arm_with_retry(conn, label, timeout=120.0, force_after=60.0)
                if not armed:
                    print(f"!! [ERROR] Drone {drone_num} failed to arm for liftoff. Check EKF status.")
                    return

                # 3. Command NAV_TAKEOFF (ArduCopter only accepts it while landed).
                #    If the drone is still in the air (revived mid-LAND), skip it and let the
                #    formation setpoints fly it up to its slot instead.
                cur_alt = 0.0
                gp = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
                if gp is not None:
                    cur_alt = gp.relative_alt / 1000.0
                if cur_alt >= 1.0:
                    print(f">> Drone {drone_num} still airborne at {cur_alt:.1f}m; rejoining formation directly.")
                    cur_pos = None
                    if gp is not None:
                        cur_pos = self.frame.gps_to_global_ned(
                            gp.lat / 1e7, gp.lon / 1e7, gp.relative_alt / 1000.0
                        )
                    with self.lock:
                        self.simulated_faults[idx] = False
                        self.drone_health[idx] = True
                        self.is_airborne[idx] = True
                        self.last_heartbeat[idx] = time.time()
                        if cur_pos is not None:
                            self.last_pos[idx] = cur_pos
                            self.current_offsets[idx] = cur_pos - self.centroid_pos
                        self._calculate_active_topology()
                        self.morph_steps_remaining = 45
                    print(f"\n* [REJOIN COMPLETE] Drone {drone_num} locked back into swarm!\n")
                    return

                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, float(target_alt)
                )
                print(f">> [ASCENDING] Drone {drone_num} airborne! Climbing to cruising altitude...")

                # 4. Monitor altitude (wall budget is generous for slow Gazebo RTF)
                t_start = time.time()
                reached = False
                while time.time() - t_start < 180.0:
                    m = conn.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.5)
                    if m:
                        alt = m.relative_alt / 1000.0 if m.get_type() == "GLOBAL_POSITION_INT" else -m.z
                        if alt >= (target_alt * 0.85):
                            reached = True
                            break
                if not reached:
                    print(f"!! [ERROR] Drone {drone_num} did not reach altitude. Not joining formation.")
                    return

                # Read actual physical position from drone to initialize its offset accurately
                cur_pos = None
                gp = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
                if gp is not None:
                    cur_pos = self.frame.gps_to_global_ned(
                        gp.lat / 1e7, gp.lon / 1e7, gp.relative_alt / 1000.0
                    )

                # Only now is the drone healthy and part of the swarm.
                with self.lock:
                    self.simulated_faults[idx] = False
                    self.drone_health[idx] = True
                    self.is_airborne[idx] = True
                    self.last_heartbeat[idx] = time.time()
                    if cur_pos is not None:
                        self.last_pos[idx] = cur_pos
                        self.current_offsets[idx] = cur_pos - self.centroid_pos
                    self._calculate_active_topology()
                    self.morph_steps_remaining = 45
                print(f"\n* [REJOIN COMPLETE] Drone {drone_num} reached altitude and locked back into swarm!\n")
            finally:
                self.busy[idx] = False  # release the connection back to the guidance thread

        t = threading.Thread(target=_climb_worker, daemon=True)
        t.start()

    def revive_drone(self, drone_num: int, auto_liftoff: bool = True):
        """Restores a failed drone and triggers automatic re-arm + liftoff."""
        idx = drone_num - 1
        if idx < 0 or idx >= 5:
            print(f">> Error: Drone ID must be 1 to 5. Received: {drone_num}")
            return

        print(f"\n>> [REVIVING] Telemetry link restored for Drone {drone_num}!")
        if auto_liftoff:
            self.liftoff_drone(drone_num)
        else:
            with self.lock:
                self.simulated_faults[idx] = False
                self.drone_health[idx] = True
            print(f">> Drone {drone_num} marked online (Grounded). Type 'lift {drone_num}' to take off.")

    def set_formation_style(self, style: str):
        valid = ["v", "line", "column", "diamond", "ring"]
        if style not in valid:
            print(f">> Unknown style '{style}'. Valid: {valid}")
            return
        with self.lock:
            self.current_formation_style = style
            self._calculate_active_topology()
            self.morph_steps_remaining = 45
        print(f">> [MORPH] Swarm morphing into {style.upper()} topology over 4.5s...")

    def send_formation_setpoints(self):
        with self.lock:
            centroid = self.centroid_pos.copy()
            offsets = self.current_offsets.copy()
            healthy = list(self.drone_health)
            faults = list(self.simulated_faults)
            airborne = list(self.is_airborne)

        type_mask = 0b0000101111111000  # Position + Yaw
        for i, conn in enumerate(self.connections):
            if not healthy[i] or faults[i] or not airborne[i]:
                continue

            tgt_global = np.array([
                centroid[0] + offsets[i, 0],
                centroid[1] + offsets[i, 1],
                centroid[2] + offsets[i, 2]
            ])
            tgt_local = tgt_global - self.frame_origins[i]
            conn.mav.set_position_target_local_ned_send(
                0, conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                float(tgt_local[0]), float(tgt_local[1]), float(tgt_local[2]),
                0, 0, 0, 0, 0, 0, self.FORMATION_YAW_RAD, 0
            )

    def guidance_loop(self):
        """
        Continuous 10 Hz background thread with automatic heartbeat timeout detection.

        This thread is the only reader of a drone's MAVLink connection unless a
        worker (liftoff) has claimed it via self.busy[i]; pymavlink connections are
        not thread-safe, so a claimed connection is never touched here.
        """
        now = time.time()
        with self.lock:
            self.last_heartbeat = [now] * 5

        while self.running:
            for i, conn in enumerate(self.connections):
                if self.busy[i]:
                    # A worker owns this link and is actively talking to the drone.
                    self.last_heartbeat[i] = time.time()
                    continue
                while True:
                    msg = conn.recv_match(blocking=False)
                    if not msg:
                        break
                    mtype = msg.get_type()
                    if mtype == "HEARTBEAT":
                        self.last_heartbeat[i] = time.time()
                    elif mtype == "GLOBAL_POSITION_INT":
                        self.last_pos[i] = self.frame.gps_to_global_ned(
                            msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
                        )

            now = time.time()
            for i in range(5):
                with self.lock:
                    watched = not self.simulated_faults[i] and self.drone_health[i] and self.is_airborne[i]
                if watched and not self.busy[i] and now - self.last_heartbeat[i] > self.hb_timeout:
                    print(f"\n[TIMEOUT] Drone {i + 1} silent for >{self.hb_timeout:.0f}s! Triggering dynamic failover...")
                    self.inject_failure(i + 1)

            with self.lock:
                if self.morph_steps_remaining > 0:
                    alpha = 1.0 / self.morph_steps_remaining
                    self.current_offsets += alpha * (self.target_offsets - self.current_offsets)
                    self.morph_steps_remaining -= 1

            self.send_formation_setpoints()
            time.sleep(0.1)

    def move(self, dn: float = 0.0, de: float = 0.0, d_alt: float = 0.0):
        with self.lock:
            self.centroid_pos[0] += dn
            self.centroid_pos[1] += de
            self.centroid_pos[2] -= d_alt
        print(f">> Vector Setpoint: N={self.centroid_pos[0]:.1f}m, E={self.centroid_pos[1]:.1f}m, Alt={-self.centroid_pos[2]:.1f}m")

    def print_status(self):
        with self.lock:
            positions = [p.copy() for p in self.last_pos]

        active_count = sum(
            1 for i in range(5)
            if self.drone_health[i] and not self.simulated_faults[i] and self.is_airborne[i]
        )
        print("\n" + "=" * 72)
        print(f" RESILIENT SWARM: {active_count}/5 Drones Airborne | Apex Leader: Drone {self.current_leader_idx + 1}")
        print(f" Formation: {self.current_formation_style.upper()} | Origin: ({self.centroid_pos[0]:.1f}N, {self.centroid_pos[1]:.1f}E, {-self.centroid_pos[2]:.1f}m)")
        print("-" * 72)
        for i in range(5):
            is_active = self.drone_health[i] and not self.simulated_faults[i] and self.is_airborne[i]
            status = "AIRBORNE" if is_active else ("GROUNDED" if not self.simulated_faults[i] else "LOST/FAILED")
            role = "APEX LEADER" if i == self.current_leader_idx and is_active else ("WINGMAN" if is_active else "INACTIVE")
            pos = positions[i]
            print(f"  Drone {i + 1}: [{status:10s}] Role: {role:11s} | N={pos[0]:5.1f}m, E={pos[1]:5.1f}m, Alt={-pos[2]:4.1f}m")
        print("=" * 72 + "\n")

    def print_menu_bar(self):
        active_count = sum(
            1 for i in range(5)
            if self.drone_health[i] and not self.simulated_faults[i] and self.is_airborne[i]
        )
        print("-" * 76)
        print(f" [SWARM: {active_count}/5 Online | Apex Leader: D{self.current_leader_idx + 1} | Shape: {self.current_formation_style.upper()}]")
        print(" [w/s/a/d] Move   [u/j] Alt   [v/l/k/x/o] Shapes   f <id> Fail   r <id> Revive   lift <id> Launch   q Land")
        print("-" * 76)

    def land_all(self):
        print("\n>> Synchronous LAND commanded for entire swarm...")
        self.running = False
        for conn in self.connections:
            conn.set_mode(9)  # LAND
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
        print(">> Synchronous landing in progress. Motors will disarm on touchdown.")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Self-Healing Autonomous Swarm Flight Console")
    parser.add_argument("--gazebo", action="store_true", help="Force Gazebo 3D simulation mode")
    parser.add_argument("--standalone", action="store_true", help="Force standalone SITL mode")
    parser.add_argument("--restart", action="store_true", help="Restart fresh")
    args = parser.parse_args()

    use_gazebo = True if args.gazebo else (False if args.standalone else None)

    console = SelfHealingSwarmConsole(use_gazebo=use_gazebo, force_restart=args.restart)
    console.launch_sitl()
    console.connect_all()
    console.sync_frames()

    t = threading.Thread(target=console.guidance_loop, daemon=True)
    t.start()

    print("\n" + "=" * 76)
    print("      SELF-HEALING AUTONOMOUS SWARM CONSOLE (HIGH-EFFICIENCY MODE)     ")
    print("=" * 76)
    print(" Directional Control (optional step size, e.g. 'w 10'):")
    print("   [w] North (+6m)        [s] South (-6m)")
    print("   [a] West (-5m)         [d] East (+5m)")
    print("   [u] Climb (+2m)        [j] Descend (-2m)")
    print("   [c] Center Origin (0, 0, 5m)")
    print("")
    print(" Formation Topologies (Instant Switch):")
    print("   [v] Flying-V           [l] Line Abreast       [k] Column (In-Trail)")
    print("   [x] Diamond            [o] Ring / Perimeter")
    print("")
    print(" Fault-Tolerance & Liftoff Commands:")
    print("   fail <1-5>  (alias 'f <id>')   -> Kill drone (e.g. 'f 1' promotes Drone 2)")
    print("   revive <1-5>(alias 'r <id>')   -> Reconnect, Auto-Arm & Liftoff to formation")
    print("   lift <1-5>  (alias 't <id>')   -> Explicitly liftoff & climb one grounded drone")
    print("")
    print(" Status & Landing:")
    print("   [launch]         -> ARM, TAKE OFF and FORM UP all grounded drones (drones start on the ground)")
    print("   [status / Enter] -> Display live swarm role hierarchy & health table")
    print("   [q / land]       -> Synchronous Land and Safe Disarm")
    print("=" * 76)
    print("\n>> All drones are on the ground and ready. Type 'launch' to take off.\n")

    try:
        while True:
            console.print_menu_bar()
            try:
                cmd = input("Swarm Command > ").strip().lower()
                tokens = cmd.split()
                if not tokens:
                    continue

                action = tokens[0]
                val = float(tokens[1]) if len(tokens) > 1 and tokens[1].replace(".", "", 1).replace("-", "", 1).isdigit() else None

                # Fast directional movements with optional distance
                if action in ["w", "north"]:
                    console.move(dn=val if val is not None else 6.0)
                elif action in ["s", "south"]:
                    console.move(dn=-(val if val is not None else 6.0))
                elif action in ["a", "west"]:
                    console.move(de=-(val if val is not None else 5.0))
                elif action in ["d", "east"]:
                    console.move(de=val if val is not None else 5.0)
                elif action in ["u", "up"]:
                    console.move(d_alt=val if val is not None else 2.0)
                elif action in ["j", "down"]:
                    console.move(d_alt=-(val if val is not None else 2.0))
                elif action in ["c", "center"]:
                    with console.lock:
                        console.centroid_pos = np.array([0.0, 0.0, -5.0])
                    print(">> Centroid returned to origin (0, 0, 5m).")

                # Formations
                elif action == "v":
                    console.set_formation_style("v")
                elif action in ["l", "line"]:
                    console.set_formation_style("line")
                elif action in ["k", "col", "column"]:
                    console.set_formation_style("column")
                elif action in ["x", "diamond"]:
                    console.set_formation_style("diamond")
                elif action in ["o", "ring"]:
                    console.set_formation_style("ring")

                # Fault Injection
                elif action in ["fail", "f", "kill", "drop"]:
                    d_id = int(tokens[1]) if len(tokens) > 1 and tokens[1].isdigit() else None
                    if d_id:
                        console.inject_failure(d_id)
                    else:
                        print("Usage: f <drone_id> (e.g. 'f 1' or 'f 2')")

                # Revive + Auto-Liftoff
                elif action in ["revive", "r", "heal"]:
                    d_id = int(tokens[1]) if len(tokens) > 1 and tokens[1].isdigit() else None
                    if d_id:
                        console.revive_drone(d_id, auto_liftoff=True)
                    else:
                        print("Usage: r <drone_id> (e.g. 'r 2' or 'r 1')")

                # Master Swarm Takeoff & Individual Liftoff
                elif action in ["takeoff", "launch", "fly", "go", "ready"]:
                    console.launch_swarm(target_alt=5.0)

                elif action in ["lift", "liftoff", "t", "climb"]:
                    if len(tokens) > 1 and tokens[1].isdigit():
                        console.liftoff_drone(int(tokens[1]))
                    elif len(tokens) > 1 and tokens[1] in ["all", "swarm"]:
                        console.launch_swarm(target_alt=5.0)
                    else:
                        print("Usage: lift <drone_id> (e.g. 'lift 2') or 'lift all'")

                # Telemetry ('s' is South, so status uses its own aliases)
                elif action in ["status", "stat", "info"]:
                    console.print_status()

                # Exit / Land
                elif action in ["q", "exit", "land"]:
                    console.land_all()
                    break
                else:
                    print("Commands: w, a, s, d, u, j, v, l, k, x, o, f <id>, r <id>, lift <id>, status, q")
            except EOFError:
                console.land_all()
                break
            except Exception as exc:
                traceback.print_exc()
                print(f"!! Command failed: {exc} (console is still running)")
    except KeyboardInterrupt:
        print("\n>> Interrupted by pilot. Shutting down...")
        console.land_all()
    finally:
        console.cleanup()


if __name__ == "__main__":
    main()
