#!/usr/bin/env python3
"""
Interactive Swarm Flight Console (3 ArduPilot SITL Drones).
Allows the pilot to command and maneuver the 3-drone swarm live in software:
- Connects directly to ArduPilot SITL (TCP 5760, 5770, 5780).
- Automatically launches 3 SITL quad instances if not running.
- Establishes shared WGS84 flat-earth coordinate frame.
- Arms, takes off to 5.0m, and locks Flying-V formation.
- Interactive pilot console: WASD, Climb/Descend, Patrol, Return-to-Center.
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


class SwarmPilotConsole:
    def __init__(self, use_gazebo: Optional[bool] = None, force_restart: bool = False):
        self.repo_root = Path(__file__).resolve().parent.parent
        self.datum_lat = -35.363261
        self.datum_lon = 149.165230
        self.datum_alt = 584.0
        self.frame = CommonCoordinateFrame(self.datum_lat, self.datum_lon, self.datum_alt)
        self.use_gazebo = use_gazebo
        self.force_restart = force_restart

        # Gazebo (JSON) mode: the plugin feeds absolute world positions that SITL adds to --home, so all
        # instances must share ONE home (the world origin). Staggered homes (used only by the standalone
        # --model quad mode, to keep drones apart) would shift each drone away from its Gazebo spawn point.
        self.gazebo_home = "-35.363261,149.165230,584,0"
        self.drone_configs = [
            {"id": 0, "inst": 0, "port": 5760, "home": "-35.363261,149.165230,584,0", "label": "Drone 0 (Apex)"},
            {"id": 1, "inst": 1, "port": 5770, "home": "-35.363261,149.165285,584,0", "label": "Drone 1 (Left Wing)"},
            {"id": 2, "inst": 2, "port": 5780, "home": "-35.363261,149.165340,584,0", "label": "Drone 2 (Right Wing)"},
        ]

        # V-Formation offsets: Apex at (0,0), Left at (-3,-3), Right at (-3,+3)
        self.v_offsets = np.array([
            [ 0.0,  0.0, 0.0],
            [-3.0, -3.0, 0.0],
            [-3.0,  3.0, 0.0],
        ])

        self.connections = []
        self.procs = []
        self.frame_origins = []
        self.centroid_pos = np.array([0.0, 0.0, -5.0])  # [North, East, Down]
        self.running = True
        self.lock = threading.Lock()

    @staticmethod
    def detect_gazebo() -> bool:
        """Fast instant process and socket check to detect Gazebo Harmonic."""
        try:
            res = subprocess.run(["pgrep", "-f", "gz sim"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if res.returncode == 0:
                return True
        except Exception:
            pass
        try:
            res = subprocess.run(["pgrep", "-f", "gz server"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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
        # Determine whether Gazebo mode is active
        if self.use_gazebo is not None:
            is_gazebo = self.use_gazebo
        else:
            is_gazebo = self.detect_gazebo()

        # Handle process cleanup / model mismatch
        if self.force_restart:
            print(">> [--restart] Killing lingering ArduPilot SITL processes...")
            self.kill_existing_sitl()
        elif is_gazebo and self.is_existing_sitl_quad():
            print(">> Lingering standalone SITL instances (--model quad) detected while Gazebo is active.")
            print(">> Restarting SITL in Gazebo JSON mode (--model JSON)...")
            self.kill_existing_sitl()
        elif not is_gazebo and not self.is_existing_sitl_quad():
            # Standalone requested but running with JSON
            try:
                res = subprocess.run(["pgrep", "-a", "-f", "arducopter"], stdout=subprocess.PIPE, text=True)
                if res.returncode == 0 and "--model JSON" in res.stdout:
                    print(">> Lingering Gazebo SITL instances detected while standalone requested. Restarting...")
                    self.kill_existing_sitl()
            except Exception:
                pass

        # Check if already running with matching configuration
        try:
            test_conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
            msg = test_conn.wait_heartbeat(timeout=1.5)
            if msg:
                print(">> Connected to active ArduPilot SITL instances.")
                return
        except Exception:
            pass

        if is_gazebo:
            print(">> [3D GAZEBO DETECTED] Linking ArduPilot SITL to Gazebo Cinewhoops (FDM Ports 9002, 9012, 9022)...")
        else:
            print(">> Gazebo not detected. Launching standalone ArduPilot SITL quad physics...")

        sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter")))
        if not sitl_bin.exists():
            raise FileNotFoundError(f"ArduCopter binary not found at: {sitl_bin}")

        params_file = str(self.repo_root / "sitl" / "swarm_params.parm")
        for cfg in self.drone_configs:
            if is_gazebo:
                cmd = [
                    str(sitl_bin),
                    f"-I{cfg['inst']}",
                    "--model", "JSON",
                    "--home", self.gazebo_home,  # one shared home in Gazebo mode (see drone_configs note)
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
        print(">> Connecting MAVLink telemetry to all 3 drones...")
        self.connections.clear()
        for cfg in self.drone_configs:
            conn = None
            for _ in range(15):
                try:
                    conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{cfg['port']}")
                    msg = conn.wait_heartbeat(timeout=2.0)
                    if msg:
                        break
                except Exception:
                    time.sleep(0.5)
            if not conn:
                raise RuntimeError(f"Failed to connect to {cfg['label']}")
            self.connections.append(conn)

            # Set parameters and request stream
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

        # Wait for EKF origin lock
        print(">> Waiting for EKF origin and GPS locks...")
        t0 = time.time()
        origin_ready = [False, False, False]
        while time.time() - t0 < 20.0 and not all(origin_ready):
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
        print("\n>> Arming and commanding takeoff to 5.0m...")
        # Arm every drone first (waits for the EKF position estimate; never takes off unarmed)
        for i, conn in enumerate(self.connections):
            if not arm_with_retry(conn, self.drone_configs[i]["label"]):
                raise RuntimeError(
                    f"{self.drone_configs[i]['label']} did not arm. Check the SITL/Gazebo link and EKF health."
                )
        for i, conn in enumerate(self.connections):
            if not takeoff_and_verify(conn, self.drone_configs[i]["label"], 5.0):
                raise RuntimeError(f"{self.drone_configs[i]['label']} armed but did not climb. Check the Gazebo motor link.")

        # Monitor climb
        print(">> Monitoring climb to 5.0m cruising hover...")
        t_climb = time.time()
        hover_flags = [False, False, False]
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
                            # Resend takeoff if not airborne yet
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, 5.0
                            )
            time.sleep(0.2)

        # Compute dynamic frame origins
        print(">> Computing dynamic frame origins for all 3 drones...")
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
        print(">> Assembling V-formation...")
        for _ in range(30):
            self.send_formation_setpoints()
            time.sleep(0.1)
        print(">> Swarm in formation and ready for pilot commands!")

    def send_formation_setpoints(self):
        with self.lock:
            centroid = self.centroid_pos.copy()

        type_mask = 0b0000111111111000  # Position only
        for i, conn in enumerate(self.connections):
            tgt_global = np.array([
                centroid[0] + self.v_offsets[i, 0],
                centroid[1] + self.v_offsets[i, 1],
                centroid[2]
            ])
            tgt_local = tgt_global - self.frame_origins[i]
            conn.mav.set_position_target_local_ned_send(
                0, conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                float(tgt_local[0]), float(tgt_local[1]), float(tgt_local[2]),
                0, 0, 0, 0, 0, 0, 0, 0
            )

    def guidance_loop(self):
        """Continuous 10 Hz background thread keeping swarm locked to pilot centroid."""
        while self.running:
            self.send_formation_setpoints()
            time.sleep(0.1)

    def move(self, dn: float = 0.0, de: float = 0.0, d_alt: float = 0.0):
        with self.lock:
            self.centroid_pos[0] += dn
            self.centroid_pos[1] += de
            self.centroid_pos[2] -= d_alt  # Down is negative
        print(f">> Pilot Setpoint: North={self.centroid_pos[0]:.1f}m, East={self.centroid_pos[1]:.1f}m, Alt={-self.centroid_pos[2]:.1f}m")

    def run_square_patrol(self):
        print("\n>>> EXECUTING AUTONOMOUS SQUARE PATROL <<<")
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
        print(">>> Square Patrol Completed! Swarm held V-formation! <<<\n")

    def print_status(self):
        positions = []
        for i, conn in enumerate(self.connections):
            m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if m:
                g_ned = self.frame.gps_to_global_ned(m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
                positions.append(g_ned)
            else:
                positions.append(np.zeros(3))

        p0, p1, p2 = positions[0], positions[1], positions[2]
        d01 = np.linalg.norm(p0[:2] - p1[:2])
        d02 = np.linalg.norm(p0[:2] - p2[:2])
        span = np.linalg.norm(p1[:2] - p2[:2])
        print(f"  [SWARM STATUS] Apex: ({p0[0]:4.1f}N, {p0[1]:4.1f}E, {p0[2]:4.1f}Alt) | D0-D1={d01:4.2f}m | D0-D2={d02:4.2f}m | Span={span:4.2f}m")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Live SITL Swarm Flight Console (3 Cinewhoop Drones)")
    parser.add_argument("--gazebo", action="store_true", help="Force Gazebo 3D simulation mode (--model JSON)")
    parser.add_argument("--standalone", action="store_true", help="Force standalone headless SITL mode (--model quad)")
    parser.add_argument("--restart", action="store_true", help="Kill existing SITL processes and restart fresh")
    args = parser.parse_args()

    use_gazebo = None
    if args.gazebo:
        use_gazebo = True
    elif args.standalone:
        use_gazebo = False

    console = SwarmPilotConsole(use_gazebo=use_gazebo, force_restart=args.restart)
    console.launch_sitl()
    console.connect_all()
    console.takeoff_and_assemble()

    # Start background loop
    t = threading.Thread(target=console.guidance_loop, daemon=True)
    t.start()

    print("\n" + "=" * 65)
    print("      LIVE SITL PILOT COMMAND CONSOLE (3 CINEWHOOP DRONES)     ")
    print("=" * 65)
    print(" Pilot Commands:")
    print("   [w / 1] Fly North +6m          [s / 2] Fly South -6m")
    print("   [a / 3] Fly West -5m           [d / 4] Fly East +5m")
    print("   [u / 5] Climb +2m              [j / 6] Descend -2m")
    print("   [p / 7] Square Patrol Route (Autonomous 4-corner flight)")
    print("   [c / 8] Return to Center (0, 0, 5m)")
    print("   [space] Print Current Telemetry Status")
    print("   [q / 0] Land and Exit")
    print("=" * 65)

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
            elif cmd in ["p", "7"]:
                console.run_square_patrol()
            elif cmd in ["c", "8"]:
                with console.lock:
                    console.centroid_pos = np.array([0.0, 0.0, -5.0])
                print(">> Returning to Center Origin (0, 0, 5m)...")
            elif cmd in ["", "space", "status"]:
                console.print_status()
            elif cmd in ["q", "0", "exit"]:
                print(">> Exiting pilot console...")
                console.running = False
                break
            else:
                print("Invalid command. Use: w, a, s, d, u, j, p, c, or q.")
    except KeyboardInterrupt:
        print("\n>> Interrupted by pilot. Shutting down...")
        console.running = False
    finally:
        console.cleanup()


if __name__ == "__main__":
    main()
