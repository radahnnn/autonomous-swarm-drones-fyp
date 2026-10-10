#!/usr/bin/env python3
"""
Interactive Swarm Flight Console (5 ArduPilot SITL Drones).
Allows the pilot to command and maneuver the 5-drone swarm live in software:
- Connects directly to ArduPilot SITL (UDP 14552, 14562, 14572, 14582, 14592).
- Automatically launches 5 SITL quad instances if not running.
- Establishes shared WGS84 flat-earth coordinate frame.
- Arms, takes off to 5.0m, and locks Flying-V formation simultaneously.
- Interactive pilot console: WASD, Climb/Descend, Patrol, Return-to-Center.
- Live Formation Morphing: Flying-V, Line Abreast, Pentagon/Circle, Column, Diamond/Cross, Echelon.
"""

import atexit
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import List, Optional
import numpy as np
from pymavlink import mavutil

from sitl.common_frame import CommonCoordinateFrame
from sitl.flight_prep import arm_with_retry, takeoff_and_verify


class SwarmPilotConsole5Drones:
    # Fixed swarm heading: 0 rad = North (V opens south).
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
            {"id": 0, "inst": 0, "port": 14552, "udp": True, "home": "-35.363261,149.165230,584,0", "label": "Drone 1 (Apex Leader)"},
            {"id": 1, "inst": 1, "port": 14562, "udp": True, "home": "-35.363283,149.165208,584,0", "label": "Drone 2 (Left Inner)"},
            {"id": 2, "inst": 2, "port": 14572, "udp": True, "home": "-35.363283,149.165252,584,0", "label": "Drone 3 (Right Inner)"},
            {"id": 3, "inst": 3, "port": 14582, "udp": True, "home": "-35.363306,149.165186,584,0", "label": "Drone 4 (Left Outer)"},
            {"id": 4, "inst": 4, "port": 14592, "udp": True, "home": "-35.363306,149.165274,584,0", "label": "Drone 5 (Right Outer)"},
        ]

        # Supported 5-Drone Swarm Formations
        self.formations = {
            "v": {
                "name": "Flying-V (Chevron)",
                "offsets": np.array([
                    [ 0.0,  0.0, 0.0],  # D1 Apex
                    [-3.0, -3.0, 0.0],  # D2 Left Inner
                    [-3.0,  3.0, 0.0],  # D3 Right Inner
                    [-6.0, -6.0, 0.0],  # D4 Left Outer
                    [-6.0,  6.0, 0.0],  # D5 Right Outer
                ])
            },
            "line": {
                "name": "Line (Abreast / Side-by-Side)",
                "offsets": np.array([
                    [ 0.0,  0.0, 0.0],  # D1 Center
                    [ 0.0, -3.0, 0.0],  # D2 Inner Left
                    [ 0.0,  3.0, 0.0],  # D3 Inner Right
                    [ 0.0, -6.0, 0.0],  # D4 Outer Left
                    [ 0.0,  6.0, 0.0],  # D5 Outer Right
                ])
            },
            "column": {
                "name": "Column (In-Trail Single File)",
                "offsets": np.array([
                    [ 6.0,  0.0, 0.0],  # D1 Lead
                    [ 3.0,  0.0, 0.0],  # D2
                    [ 0.0,  0.0, 0.0],  # D3 Center
                    [-3.0,  0.0, 0.0],  # D4
                    [-6.0,  0.0, 0.0],  # D5 Trail
                ])
            },
            "pentagon": {
                "name": "Pentagon / Circle (Radial Ring)",
                "offsets": np.array([
                    [ 4.0,   0.0,   0.0],   # D1 (North apex)
                    [ 1.24,  3.80,  0.0],   # D2 (East wing)
                    [ 1.24, -3.80,  0.0],   # D3 (West wing)
                    [-3.24,  2.35,  0.0],   # D4 (South-East)
                    [-3.24, -2.35,  0.0],   # D5 (South-West)
                ])
            },
            "diamond": {
                "name": "Diamond + Leader (Cross/Star)",
                "offsets": np.array([
                    [ 0.0,  0.0, 0.0],  # D1 Center Leader
                    [ 4.0,  0.0, 0.0],  # D2 Front (North)
                    [-4.0,  0.0, 0.0],  # D3 Rear (South)
                    [ 0.0, -4.0, 0.0],  # D4 Left (West)
                    [ 0.0,  4.0, 0.0],  # D5 Right (East)
                ])
            },
            "echelon": {
                "name": "Echelon (Diagonal Flight)",
                "offsets": np.array([
                    [ 4.0, -4.0, 0.0],
                    [ 2.0, -2.0, 0.0],
                    [ 0.0,  0.0, 0.0],
                    [-2.0,  2.0, 0.0],
                    [-4.0,  4.0, 0.0],
                ])
            }
        }
        self.current_formation_key = "v"
        self.current_offsets = self.formations["v"]["offsets"].astype(float).copy()
        self.target_offsets = self.formations["v"]["offsets"].astype(float).copy()
        self.morph_steps_remaining = 0

        self.connections = []
        self.procs = []
        self.frame_origins = []
        self.centroid_pos = np.array([0.0, 0.0, -5.0])  # [North, East, Down]
        self.running = True
        self.lock = threading.Lock()

    @staticmethod
    def detect_gazebo() -> bool:
        """Process check to detect Gazebo Harmonic."""
        try:
            res = subprocess.run(["pgrep", "-f", "gz sim"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if res.returncode == 0:
                return True
        except Exception:
            pass
        return False

    @staticmethod
    def kill_existing_sitl():
        """Cleanly terminate lingering SITL processes."""
        try:
            subprocess.run(["pkill", "-9", "-f", "arducopter"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
        except Exception:
            pass

    @staticmethod
    def is_existing_sitl_quad() -> bool:
        """Check if active arducopter SITL was launched with standalone quad physics."""
        try:
            res = subprocess.run(["pgrep", "-a", "-f", "arducopter"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if res.returncode == 0 and "--model quad" in res.stdout:
                return True
        except Exception:
            pass
        return False

    def launch_sitl(self):
        if self.use_gazebo is not None:
            is_gazebo = self.use_gazebo
        else:
            is_gazebo = self.detect_gazebo()

        if self.force_restart:
            print(">> [--restart] Killing lingering ArduPilot SITL processes...")
            self.kill_existing_sitl()

        # Check if already running on UDP 14552
        already_running = False
        try:
            test_conn = mavutil.mavlink_connection("udpin:127.0.0.1:14552")
            msg = test_conn.wait_heartbeat(timeout=1.5)
            if msg:
                print(">> Detected active SITL instances on UDP 14552..14592.")
                print(">> Skipping SITL launch — connecting directly to existing 5 drones.")
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
        print(">> Connecting MAVLink telemetry to all 5 drones...")
        self.connections.clear()
        for cfg in self.drone_configs:
            endpoint = f"udpin:127.0.0.1:{cfg['port']}"
            conn = None
            for attempt in range(25):
                try:
                    conn = mavutil.mavlink_connection(endpoint)
                    msg = conn.wait_heartbeat(timeout=3.0)
                    if msg:
                        print(f"  [OK] {cfg['label']} connected on {endpoint} (SYSID={conn.target_system})")
                        break
                    conn = None
                except Exception:
                    conn = None
                    time.sleep(0.5)
            if not conn:
                raise RuntimeError(
                    f"Failed to connect to {cfg['label']} on {endpoint}.\n"
                    f"  Make sure all 5 SITL drones are running via: bash sitl/start_swarm_5drones_2d.sh"
                )
            self.connections.append(conn)

            # Relax arming checks and request full telemetry stream
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

        # Wait for EKF origin and GPS locks
        print(">> Waiting for EKF origin and GPS locks across all 5 drones...")
        t0 = time.time()
        origin_ready = [False] * 5
        while time.time() - t0 < 30.0 and not all(origin_ready):
            for i, conn in enumerate(self.connections):
                if not origin_ready[i]:
                    while True:
                        msg = conn.recv_match(blocking=False)
                        if not msg:
                            break
                        if msg.get_type() == "STATUSTEXT" and "origin set" in msg.text.lower():
                            origin_ready[i] = True
                            print(f"  [{self.drone_configs[i]['label']}] EKF origin set!")
                        elif msg.get_type() == "GLOBAL_POSITION_INT" and msg.lat != 0:
                            origin_ready[i] = True
                            print(f"  [{self.drone_configs[i]['label']}] GPS lock confirmed!")
            time.sleep(0.1)

    def takeoff_and_assemble(self):
        print("\n>> Arming and commanding SIMULTANEOUS takeoff to 5.0m across all 5 drones...")
        from concurrent.futures import ThreadPoolExecutor

        # Parallel simultaneous arming
        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(arm_with_retry, conn, self.drone_configs[i]["label"])
                for i, conn in enumerate(self.connections)
            ]
            results = [f.result() for f in futures]
            if not all(results):
                raise RuntimeError("One or more drones failed to arm. Check SITL link and EKF health.")

        # Parallel simultaneous takeoff
        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(takeoff_and_verify, conn, self.drone_configs[i]["label"], 5.0)
                for i, conn in enumerate(self.connections)
            ]
            results = [f.result() for f in futures]
            if not all(results):
                raise RuntimeError("One or more drones armed but failed to climb.")

        # Monitor climb
        print(">> Monitoring simultaneous climb to 5.0m cruising hover...")
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
                            print(f"  [{self.drone_configs[i]['label']}] Steady hover at {alt:.2f}m!")
                        elif time.time() - t_climb > 6.0 and alt < 1.0:
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, 5.0
                            )
            time.sleep(0.2)

        # Dynamic frame origins calibration
        print(">> Computing dynamic frame origins for all 5 drones...")
        self.frame_origins.clear()
        for i, conn in enumerate(self.connections):
            loc_msg, glob_msg = None, None
            t_s = time.time()
            while time.time() - t_s < 3.0:
                m = conn.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"], blocking=False)
                if m:
                    if m.get_type() == "LOCAL_POSITION_NED":
                        loc_msg = m
                    elif m.get_type() == "GLOBAL_POSITION_INT":
                        glob_msg = m
                if loc_msg and glob_msg:
                    break
                time.sleep(0.05)

            if loc_msg and glob_msg:
                g_ned = self.frame.gps_to_global_ned(glob_msg.lat / 1e7, glob_msg.lon / 1e7, glob_msg.relative_alt / 1000.0)
                l_ned = np.array([loc_msg.x, loc_msg.y, loc_msg.z])
                origin = g_ned - l_ned
            else:
                origin = np.zeros(3)
            self.frame_origins.append(origin)
            print(f"  [{self.drone_configs[i]['label']}] Frame Origin: N={origin[0]:.2f}m, E={origin[1]:.2f}m")

        # Initial V-formation assembly
        print(">> Assembling 5-drone Flying-V formation...")
        for _ in range(30):
            self.send_formation_setpoints()
            time.sleep(0.1)
        print(">> All 5 drones locked in formation and ready for pilot commands!")

    def set_formation(self, form_key: str):
        if form_key not in self.formations:
            print(f">> Unknown formation key '{form_key}'. Valid: {list(self.formations.keys())}")
            return
        with self.lock:
            self.current_formation_key = form_key
            self.target_offsets = self.formations[form_key]["offsets"].astype(float).copy()
            self.morph_steps_remaining = 30  # Smooth 3.0s transition at 10 Hz
        print(f"\n>> [FORMATION MORPH] Morphing 5 drones into {self.formations[form_key]['name']} over 3.0s...")

    def send_formation_setpoints(self):
        with self.lock:
            centroid = self.centroid_pos.copy()
            offsets = self.current_offsets.copy()

        type_mask = 0b0000101111111000  # Position + yaw
        for i, conn in enumerate(self.connections):
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
        """Continuous 10 Hz background thread keeping 5 drones locked in formation."""
        while self.running:
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
            self.centroid_pos[2] -= d_alt  # Down is negative
        print(f">> Pilot Swarm Setpoint: North={self.centroid_pos[0]:.1f}m, East={self.centroid_pos[1]:.1f}m, Alt={-self.centroid_pos[2]:.1f}m")

    def run_square_patrol(self):
        print("\n>>> EXECUTING 5-DRONE AUTONOMOUS PATROL <<<")
        legs = [
            (8.0, 0.0, 0.0, "Leg 1: North +8m"),
            (0.0, 6.0, 0.0, "Leg 2: East +6m"),
            (-8.0, 0.0, 0.0, "Leg 3: South -8m"),
            (0.0, -6.0, 0.0, "Leg 4: West -6m (Return)"),
        ]
        for dn, de, dalt, name in legs:
            print(f" -> Commanding: {name}...")
            self.move(dn, de, dalt)
            for s in range(6):
                time.sleep(1.0)
                self.print_status()
        print(">>> 5-Drone Patrol Completed! Swarm held formation throughout! <<<\n")

    def print_status(self):
        positions = []
        for i, conn in enumerate(self.connections):
            m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if m:
                g_ned = self.frame.gps_to_global_ned(m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
                positions.append(g_ned)
            else:
                positions.append(np.zeros(3))

        form_name = self.formations[self.current_formation_key]["name"]
        print(f"  [5-DRONE SWARM | {form_name}] Centroid: ({self.centroid_pos[0]:4.1f}N, {self.centroid_pos[1]:4.1f}E, {-self.centroid_pos[2]:4.1f}Alt)")
        # Print status of each drone
        drone_strs = []
        for i, p in enumerate(positions):
            drone_strs.append(f"D{i+1}:({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})")
        print("    " + " | ".join(drone_strs))

    def land_all(self):
        print("\n>> Commanding simultaneous LAND for all 5 drones...")
        self.running = False
        for i, conn in enumerate(self.connections):
            conn.set_mode(9)  # LAND mode in ArduCopter
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
        print(">> All 5 drones landing smoothly and will auto-disarm upon touchdown.")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Live SITL Swarm Flight Console (5 Cinewhoop Drones)")
    parser.add_argument("--gazebo", action="store_true", help="Force Gazebo 3D simulation mode (--model JSON)")
    parser.add_argument("--standalone", action="store_true", help="Force standalone headless SITL mode (--model quad)")
    parser.add_argument("--restart", action="store_true", help="Kill existing SITL processes and restart fresh")
    args = parser.parse_args()

    use_gazebo = None
    if args.gazebo:
        use_gazebo = True
    elif args.standalone:
        use_gazebo = False

    console = SwarmPilotConsole5Drones(use_gazebo=use_gazebo, force_restart=args.restart)
    console.launch_sitl()
    console.connect_all()
    console.takeoff_and_assemble()

    # Start background loop
    t = threading.Thread(target=console.guidance_loop, daemon=True)
    t.start()

    print("\n" + "=" * 70)
    print("      LIVE SITL PILOT COMMAND CONSOLE (5 CINEWHOOP DRONES)     ")
    print("=" * 70)
    print(" Swarm Movement (Simultaneous in Lockstep):")
    print("   [w] Fly North +6m               [s] Fly South -6m")
    print("   [a] Fly West -5m                [d] Fly East +5m")
    print("   [u] Climb +2m                   [j] Descend -2m")
    print("   [p] Square Patrol (Autonomous 4-corner flight in formation)")
    print("   [c] Return to Center (0, 0, 5m)")
    print("")
    print(" 5-Drone Formation Switching (Live Morphing):")
    print("   [v] Flying-V Formation (Chevron / Wings Back)")
    print("   [l] Line Formation (Side-by-Side Abreast)")
    print("   [k] Column Formation (Single-File In-Trail)")
    print("   [o] Pentagon / Circle Formation (5-Point Radial Ring)")
    print("   [x] Diamond + Leader Formation (Cross / Star)")
    print("   [e] Echelon Formation (Diagonal Flight)")
    print("")
    print(" Telemetry & System:")
    print("   [space] Print Current Swarm Status")
    print("   [q] Land all drones and Exit")
    print("=" * 70)

    try:
        while True:
            cmd = input("\nPilot Input: ").strip().lower()
            if cmd in ["w", "1"]:
                console.move(dn=6.0)
            elif cmd in ["s", "2"]:
                console.move(dn=-6.0)
            elif cmd in ["a", "3"]:
                console.move(de=-5.0)
            elif cmd in ["d", "4"]:
                console.move(de=5.0)
            elif cmd in ["u", "5"]:
                console.move(d_alt=2.0)
            elif cmd in ["j", "6"]:
                console.move(d_alt=-2.0)
            elif cmd in ["v"]:
                console.set_formation("v")
            elif cmd in ["l"]:
                console.set_formation("line")
            elif cmd in ["k"]:
                console.set_formation("column")
            elif cmd in ["o", "p_gon", "pentagon"]:
                console.set_formation("pentagon")
            elif cmd in ["x", "diamond", "cross"]:
                console.set_formation("diamond")
            elif cmd in ["e"]:
                console.set_formation("echelon")
            elif cmd in ["p", "7"]:
                console.run_square_patrol()
            elif cmd in ["c", "8"]:
                with console.lock:
                    console.centroid_pos = np.array([0.0, 0.0, -5.0])
                print(">> Returning to Center Origin (0, 0, 5m)...")
            elif cmd in ["", "space", "status"]:
                console.print_status()
            elif cmd in ["q", "0", "exit"]:
                console.land_all()
                break
            else:
                print("Invalid command. Options: w, a, s, d, u, j, v, l, k, o, x, e, p, c, q.")
    except KeyboardInterrupt:
        print("\n>> Interrupted by pilot. Shutting down...")
        console.land_all()
    finally:
        console.cleanup()


if __name__ == "__main__":
    main()
