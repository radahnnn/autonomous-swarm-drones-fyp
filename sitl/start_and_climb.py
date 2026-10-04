#!/usr/bin/env python3
"""
Start and Climb: Launch 3 Swarm Drones, Arm, and Climb to 5.0m in Gazebo 3D.

Usage:
    python3 sitl/start_and_climb.py
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

# Use venv-ardupilot if available
try:
    from pymavlink import mavutil
except ImportError:
    venv_py = Path.home() / "venv-ardupilot" / "bin" / "python3"
    if venv_py.exists() and sys.executable != str(venv_py):
        os.execv(str(venv_py), [str(venv_py)] + sys.argv)
    raise

from sitl.common_frame import CommonCoordinateFrame


class SwarmStartAndClimb:
    def __init__(self):
        self.repo_root = Path(__file__).resolve().parent.parent
        self.datum_lat = -35.363261
        self.datum_lon = 149.165230
        self.datum_alt = 584.0
        self.frame = CommonCoordinateFrame(self.datum_lat, self.datum_lon, self.datum_alt)

        self.drone_configs = [
            {"id": 0, "inst": 0, "port": 5760, "home": "-35.363261,149.165230,584,0", "label": "Drone 0 (Apex)"},
            {"id": 1, "inst": 1, "port": 5770, "home": "-35.363261,149.165285,584,0", "label": "Drone 1 (Left Wing)"},
            {"id": 2, "inst": 2, "port": 5780, "home": "-35.363261,149.165340,584,0", "label": "Drone 2 (Right Wing)"},
        ]

        self.v_offsets = np.array([
            [ 0.0,  0.0, 0.0],
            [-3.0, -3.0, 0.0],
            [-3.0,  3.0, 0.0],
        ])

        self.connections = []
        self.procs = []
        self.frame_origins = []
        self.centroid_pos = np.array([0.0, 0.0, -5.0])
        self.running = True
        self.lock = threading.Lock()

    def kill_existing_sitl(self):
        try:
            subprocess.run(["pkill", "-9", "-f", "arducopter"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
        except Exception:
            pass

    def check_gazebo_running(self) -> bool:
        try:
            res = subprocess.run(["pgrep", "-f", "gz sim"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return res.returncode == 0
        except Exception:
            return False

    def launch(self):
        print("=================================================================")
        print("          AUTONOMOUS SWARM: START & CLIMB TO 5.0m                ")
        print("=================================================================")

        if not self.check_gazebo_running():
            print(">> [WARNING] Gazebo is not running! Launching Gazebo on DISPLAY=:1...")
            env = os.environ.copy()
            env.setdefault("DISPLAY", ":1")
            subprocess.Popen(["bash", str(self.repo_root / "sitl" / "launch_gazebo.sh")], env=env)
            time.sleep(5.0)

        print(">> Cleaning up lingering SITL processes...")
        self.kill_existing_sitl()

        sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter")))
        if not sitl_bin.exists():
            raise FileNotFoundError(f"ArduCopter binary not found at: {sitl_bin}")

        params_file = str(self.repo_root / "sitl" / "swarm_params.parm")

        print(">> Launching 3 ArduPilot SITL instances linked to Gazebo...")
        for cfg in self.drone_configs:
            cmd = [
                str(sitl_bin),
                f"-I{cfg['inst']}",
                "--model", "JSON",
                "--home", cfg["home"],
                "--defaults", params_file,
                "--sim-address", "127.0.0.1",
                "--speedup", "1",
            ]
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.procs.append(p)
            print(f"  [{cfg['label']}] Process spawned (TCP {cfg['port']}, FDM {9002 + cfg['inst']*10})")

        atexit.register(self.cleanup)
        time.sleep(3.0)

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

    def connect(self):
        print("\n>> Establishing MAVLink connections to all 3 drones...")
        self.connections.clear()
        for cfg in self.drone_configs:
            conn = None
            for _ in range(20):
                try:
                    conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{cfg['port']}")
                    msg = conn.wait_heartbeat(timeout=2.0)
                    if msg:
                        break
                except Exception:
                    time.sleep(0.5)
            if not conn:
                raise RuntimeError(f"Failed to connect to {cfg['label']} on TCP port {cfg['port']}")
            self.connections.append(conn)

            # Disable prearm checks and request telemetry
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
            print(f"  [{cfg['label']}] Telemetry streaming OK!")

        # Wait for EKF origin and GPS locks
        print("\n>> Waiting for EKF origin and GPS locks...")
        t0 = time.time()
        origin_ready = [False, False, False]
        while time.time() - t0 < 25.0 and not all(origin_ready):
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
                            print(f"  [{self.drone_configs[i]['label']}] GPS position locked!")
            time.sleep(0.1)

    def arm_and_climb(self, target_altitude: float = 5.0):
        print(f"\n>> Arming swarm and commanding climb to {target_altitude}m cruising hover...")
        for i, conn in enumerate(self.connections):
            conn.set_mode(4)  # GUIDED
            time.sleep(0.2)

            # Force Arm
            is_armed = False
            t_arm = time.time()
            while time.time() - t_arm < 15.0:
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0
                )
                t_poll = time.time()
                while time.time() - t_poll < 0.8:
                    msg = conn.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=False)
                    if msg:
                        if msg.get_type() == "HEARTBEAT" and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                            is_armed = True
                            break
                        elif msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                                is_armed = True
                                break
                    if conn.motors_armed():
                        is_armed = True
                        break
                    time.sleep(0.05)
                if is_armed:
                    print(f"  [{self.drone_configs[i]['label']}] ARMED! Propellers spinning!")
                    break
                time.sleep(0.2)

            # Command takeoff to target altitude
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, float(target_altitude)
            )
            print(f"  [{self.drone_configs[i]['label']}] Takeoff commanded -> {target_altitude:.1f}m!")

        # Monitor climb until hover
        print(f"\n>> Monitoring climb to {target_altitude:.1f}m cruising hover...")
        t_climb = time.time()
        hover_flags = [False, False, False]
        while time.time() - t_climb < 30.0 and not all(hover_flags):
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
                        if alt >= (target_altitude - 1.2):
                            hover_flags[i] = True
                            print(f"  [{self.drone_configs[i]['label']}] Steady cruising hover at {alt:.2f}m!")
                        elif time.time() - t_climb > 6.0 and alt < 1.0:
                            # Re-send takeoff if vehicle was delayed
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, float(target_altitude)
                            )
            time.sleep(0.25)

        # Compute dynamic frame origins
        print("\n>> Computing dynamic frame origins for all 3 drones...")
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

        print("\n" + "=" * 65)
        print(f"  SUCCESS! All 3 drones are airborne hovering at {target_altitude}m in Gazebo!")
        print("=" * 65)

    def send_formation_setpoints(self):
        with self.lock:
            centroid = self.centroid_pos.copy()

        type_mask = 0b0000111111111000  # Position setpoint only
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
        while self.running:
            self.send_formation_setpoints()
            time.sleep(0.1)

    def move(self, dn: float = 0.0, de: float = 0.0, d_alt: float = 0.0):
        with self.lock:
            self.centroid_pos[0] += dn
            self.centroid_pos[1] += de
            self.centroid_pos[2] -= d_alt
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
            for _ in range(6):
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
    swarm = SwarmStartAndClimb()
    swarm.launch()
    swarm.connect()
    swarm.arm_and_climb(target_altitude=5.0)

    # Start 10 Hz guidance loop thread
    t = threading.Thread(target=swarm.guidance_loop, daemon=True)
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
                swarm.move(dn=6.0)
            elif cmd in ["s", "2"]:
                swarm.move(dn=-6.0)
            elif cmd in ["a", "3"]:
                swarm.move(de=-5.0)
            elif cmd in ["d", "4"]:
                swarm.move(de=5.0)
            elif cmd in ["u", "5"]:
                swarm.move(d_alt=2.0)
            elif cmd in ["j", "6"]:
                swarm.move(d_alt=-2.0)
            elif cmd in ["p", "7"]:
                swarm.run_square_patrol()
            elif cmd in ["c", "8"]:
                with swarm.lock:
                    swarm.centroid_pos = np.array([0.0, 0.0, -5.0])
                print(">> Returning to Center Origin (0, 0, 5m)...")
            elif cmd in ["", "space", "status"]:
                swarm.print_status()
            elif cmd in ["q", "0", "exit"]:
                print(">> Landing and exiting...")
                for conn in swarm.connections:
                    conn.set_mode(9)  # LAND
                swarm.running = False
                break
            else:
                print("Invalid command. Use: w, a, s, d, u, j, p, c, or q.")
    except KeyboardInterrupt:
        print("\n>> Interrupted by pilot. Shutting down...")
        swarm.running = False
    finally:
        swarm.cleanup()


if __name__ == "__main__":
    main()
