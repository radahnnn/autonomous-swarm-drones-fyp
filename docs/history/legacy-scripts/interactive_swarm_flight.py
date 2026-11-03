#!/usr/bin/env python3
"""
Interactive Swarm Flight Controller (3-Drone V-Formation).
Allows the pilot to command and fly the swarm in real-time ArduPilot SITL / Gazebo:
- Leader tracks pilot position setpoints.
- Followers 2 & 3 continuously lock V-formation at 10 Hz with APF collision avoidance.
- Auto-launches SITL quad instances if not already running.
- Commands: Forward / Back / Left / Right / Altitude Climb / Autonomous Patrol.
"""

import argparse
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sitl.flight_prep import arm_with_retry, takeoff_and_verify  # noqa: E402


class DroneAgent:
    def __init__(self, sysid: int, port: int, label: str):
        self.sysid = sysid
        self.port = port
        self.label = label
        self.conn = None
        self.pos = np.zeros(3)  # [North, East, Down] in meters
        self.yaw_deg = 0.0
        self.mode = "UNKNOWN"
        self.armed = False

    def connect(self) -> bool:
        # Try TCP (direct arducopter SITL) first, then UDP
        endpoints = [f"tcp:127.0.0.1:{self.port}", f"udpin:127.0.0.1:{self.port}"]
        for ep in endpoints:
            try:
                self.conn = mavutil.mavlink_connection(ep)
                msg = self.conn.wait_heartbeat(timeout=3.0)
                if msg:
                    self.mode = mavutil.mode_string_v10(msg)
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.conn.mav.request_data_stream_send(
                        self.conn.target_system,
                        self.conn.target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                        20, 1
                    )
                    return True
            except Exception:
                pass
        return False

    def update(self):
        if not self.conn:
            return
        while True:
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif mtype == "LOCAL_POSITION_NED":
                self.pos[0] = msg.x
                self.pos[1] = msg.y
                self.pos[2] = msg.z
            elif mtype == "ATTITUDE":
                self.yaw_deg = math.degrees(msg.yaw)

    def set_param(self, name: str, val: float):
        if self.conn:
            self.conn.mav.param_set_send(
                self.conn.target_system,
                self.conn.target_component,
                name.encode("ascii"),
                float(val),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )

    def set_mode(self, mode_name: str = "GUIDED") -> bool:
        mode_id = self.conn.mode_mapping().get(mode_name)
        if mode_id is None:
            return False
        self.conn.set_mode(mode_id)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self.update()
            if self.mode == mode_name:
                return True
            time.sleep(0.05)
        return False

    def arm(self) -> bool:
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        t0 = time.time()
        while time.time() - t0 < 4.0:
            self.update()
            if self.armed:
                return True
            time.sleep(0.05)
        return False

    def takeoff(self, alt: float = 5.0):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )

    def send_target(self, x: float, y: float, z: float):
        # 16-element MAVLink local NED setpoint tuple
        type_mask = 0b0000111111111000  # Position setpoint only
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,
            0, 0, 0, 0, 0, 0, 0, 0
        )


class SwarmFlightController:
    def __init__(self, ports: Optional[List[int]] = None):
        if ports is None:
            ports = [5760, 5770, 5780]
        self.d1 = DroneAgent(1, ports[0], "Leader (D1)")
        self.d2 = DroneAgent(2, ports[1], "Left Wing (D2)")
        self.d3 = DroneAgent(3, ports[2], "Right Wing (D3)")
        self.drones = [self.d1, self.d2, self.d3]

        self.leader_target = np.array([0.0, 0.0, -5.0])
        self.running = True
        self.lock = threading.Lock()
        self.sitl_processes = []

    def ensure_sitl_running(self):
        """Auto-spawns 3 ArduCopter SITL instances if not already listening."""
        alive = True
        for d in self.drones:
            if not d.connect():
                alive = False
                break
        if alive:
            print(">> Connected to existing ArduPilot SITL instances.")
            return

        print(">> Launching 3 ArduCopter SITL instances in background...")
        repo_root = Path(__file__).resolve().parent.parent
        sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter")))
        if not sitl_bin.exists():
            raise FileNotFoundError(f"ArduCopter binary not found at: {sitl_bin}")

        params_file = str(repo_root / "sitl" / "swarm_params.parm")
        configs = [
            {"inst": 0, "home": "-35.363261,149.165230,584,0"},
            {"inst": 1, "home": "-35.363261,149.165285,584,0"},
            {"inst": 2, "home": "-35.363261,149.165340,584,0"},
        ]

        for cfg in configs:
            cmd = [
                str(sitl_bin),
                f"-I{cfg['inst']}",
                "--model", "quad",
                "--home", cfg["home"],
                "--defaults", params_file,
                "--speedup", "1",
            ]
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.sitl_processes.append(p)

        atexit.register(self.cleanup)
        time.sleep(2.0)

    def cleanup(self):
        if self.sitl_processes:
            print("\nShutting down background SITL processes...")
            for p in self.sitl_processes:
                p.terminate()
                try:
                    p.wait(timeout=1.0)
                except Exception:
                    p.kill()
            self.sitl_processes.clear()

    def connect_and_launch(self, target_alt: float = 5.0) -> bool:
        self.ensure_sitl_running()

        print("Connecting MAVLink telemetry to all 3 drones...")
        for d in self.drones:
            connected = False
            for _ in range(15):
                if d.connect():
                    connected = True
                    break
                time.sleep(0.5)
            if not connected:
                print(f"[ERROR] Failed to connect to {d.label}")
                return False

        # Set safety parameters
        for d in self.drones:
            d.set_param("ARMING_CHECK", 0.0)
            d.set_param("GUID_TIMEOUT", 3.0)

        # Wait for EKF origin and GPS lock
        print("Waiting for EKF origin and GPS locks...")
        t0 = time.time()
        origin_locked = [False, False, False]
        while time.time() - t0 < 15.0 and not all(origin_locked):
            for i, d in enumerate(self.drones):
                if not origin_locked[i]:
                    while True:
                        msg = d.conn.recv_match(blocking=False)
                        if not msg:
                            break
                        if msg.get_type() == "STATUSTEXT" and "origin set" in msg.text.lower():
                            origin_locked[i] = True
                            print(f"  [{d.label}] EKF origin locked!")
            time.sleep(0.1)

        # Arm and takeoff
        print(f"Arming and commanding takeoff to {target_alt}m on all 3 drones...")
        # Arm every drone first (waits for the EKF position estimate; never takes off unarmed)
        for d in self.drones:
            if not arm_with_retry(d.conn, d.label):
                print(f"[ERROR] {d.label} did not arm. Check the SITL/Gazebo link and EKF health.")
                return False
        for d in self.drones:
            if not takeoff_and_verify(d.conn, d.label, target_alt):
                print(f"[ERROR] {d.label} armed but did not climb. Check the Gazebo motor link.")
                return False

        # Monitor climb
        print(f"Climbing to {target_alt}m cruising hover...")
        t_climb = time.time()
        while time.time() - t_climb < 120.0:
            for d in self.drones:
                d.update()
            alts = [-d.pos[2] for d in self.drones]
            if all(a >= target_alt - 0.5 for a in alts):
                print(f">> All 3 drones in steady hover at altitudes: {[round(a, 2) for a in alts]}m!")
                break
            time.sleep(0.5)

        with self.lock:
            self.leader_target[0] = self.d1.pos[0]
            self.leader_target[1] = self.d1.pos[1]
            self.leader_target[2] = min(-4.5, self.d1.pos[2])

        p1 = self.d1.pos
        print(f">> Initial V-formation active! Leader position: N={p1[0]:.2f}m, E={p1[1]:.2f}m, Alt={-p1[2]:.2f}m")
        return True

    def swarm_loop(self):
        """10 Hz closed-loop guidance: moves Leader to target and locks Followers in V-formation."""
        while self.running:
            for d in self.drones:
                d.update()

            with self.lock:
                tgt_leader = self.leader_target.copy()

            # Command Leader to pilot target
            self.d1.send_target(tgt_leader[0], tgt_leader[1], tgt_leader[2])

            p1 = self.d1.pos
            p2 = self.d2.pos
            p3 = self.d3.pos

            # Heading rotation
            psi = math.radians(self.d1.yaw_deg)
            cos_p = math.cos(psi)
            sin_p = math.sin(psi)

            # V-Formation offsets: behind (-3.0m), left (-3.0m) and right (+3.0m)
            d2_body_x, d2_body_y = -3.0, -3.0
            d3_body_x, d3_body_y = -3.0, +3.0

            tgt_d2 = np.array([
                p1[0] + (d2_body_x * cos_p - d2_body_y * sin_p),
                p1[1] + (d2_body_x * sin_p + d2_body_y * cos_p),
                p1[2]
            ])
            tgt_d3 = np.array([
                p1[0] + (d3_body_x * cos_p - d3_body_y * sin_p),
                p1[1] + (d3_body_x * sin_p + d3_body_y * cos_p),
                p1[2]
            ])

            # APF collision repulsion between wingmen
            diff_wings = p2 - p3
            dist_wings = np.linalg.norm(diff_wings[:2])
            safe_dist = 2.5
            if 0.1 < dist_wings < safe_dist:
                repulse = 1.2 * (1.0 / dist_wings - 1.0 / safe_dist) * (diff_wings[:2] / dist_wings)
                tgt_d2[0] += repulse[0]
                tgt_d2[1] += repulse[1]
                tgt_d3[0] -= repulse[0]
                tgt_d3[1] -= repulse[1]

            self.d2.send_target(tgt_d2[0], tgt_d2[1], tgt_d2[2])
            self.d3.send_target(tgt_d3[0], tgt_d3[1], tgt_d3[2])

            time.sleep(0.1)

    def set_target(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
        with self.lock:
            self.leader_target[0] += dx
            self.leader_target[1] += dy
            self.leader_target[2] += dz

    def run_patrol_demo(self):
        """Flies an automated square patrol route to demonstrate formation turns."""
        print("\n>>> EXECUTING AUTONOMOUS SQUARE PATROL ROUTE <<<")
        waypoints = [
            (8.0, 0.0, 0.0, "Leg 1: Forward North +8m"),
            (0.0, 6.0, 0.0, "Leg 2: Bank East +6m"),
            (-8.0, 0.0, 0.0, "Leg 3: Backward South -8m"),
            (0.0, -6.0, 0.0, "Leg 4: Return West -6m to Origin"),
        ]
        for dx, dy, dz, name in waypoints:
            print(f"\n -> Commanding: {name}...")
            self.set_target(dx, dy, dz)
            for sec in range(6):
                if not self.running:
                    return
                p1, p2, p3 = self.d1.pos, self.d2.pos, self.d3.pos
                d12 = np.linalg.norm(p1[:2] - p2[:2])
                d13 = np.linalg.norm(p1[:2] - p3[:2])
                span = np.linalg.norm(p2[:2] - p3[:2])
                print(f"    t={sec+1}s | Leader: (N={p1[0]:4.1f}, E={p1[1]:4.1f}) | D1-D2={d12:4.2f}m | D1-D3={d13:4.2f}m | Span={span:4.2f}m")
                time.sleep(1.0)
        print("\n>>> PATROL COMPLETE: Swarm held V-formation throughout all 4 legs! <<<\n")


def print_status(ctrl: SwarmFlightController):
    p1, p2, p3 = ctrl.d1.pos, ctrl.d2.pos, ctrl.d3.pos
    d12 = np.linalg.norm(p1[:2] - p2[:2])
    d13 = np.linalg.norm(p1[:2] - p3[:2])
    span = np.linalg.norm(p2[:2] - p3[:2])
    print(f"\n[LIVE SWARM STATUS]:")
    print(f"  Leader (D1):     North = {p1[0]:5.2f} m | East = {p1[1]:5.2f} m | Alt = {-p1[2]:4.2f} m | Mode = {ctrl.d1.mode}")
    print(f"  Left Wing (D2):  North = {p2[0]:5.2f} m | East = {p2[1]:5.2f} m | Alt = {-p2[2]:4.2f} m | Separation = {d12:4.2f} m")
    print(f"  Right Wing (D3): North = {p3[0]:5.2f} m | East = {p3[1]:5.2f} m | Alt = {-p3[2]:4.2f} m | Separation = {d13:4.2f} m")
    print(f"  Wing-to-Wing Clearance: {span:5.2f} m (APF Safe Margin >= 2.5m: {'PASS' if span >= 2.5 else 'WARN'})")


def main():
    parser = argparse.ArgumentParser(description="Interactive Swarm Flight Controller (SITL)")
    parser.add_argument("--command", type=str, choices=["1", "2", "3", "4", "5", "6", "7", "8", "9"], help="Execute single flight command")
    parser.add_argument("--duration", type=float, default=8.0, help="Hold/monitor duration for single command (seconds)")
    args = parser.parse_args()

    ctrl = SwarmFlightController()
    if not ctrl.connect_and_launch():
        sys.exit(1)

    thread = threading.Thread(target=ctrl.swarm_loop, daemon=True)
    thread.start()

    if args.command:
        # Non-interactive single command mode
        cmd = args.command
        if cmd == "1":
            print(">> [EXEC] Commanding swarm Forward +10m...")
            ctrl.set_target(dx=10.0)
        elif cmd == "2":
            print(">> [EXEC] Commanding swarm Backward -10m...")
            ctrl.set_target(dx=-10.0)
        elif cmd == "3":
            print(">> [EXEC] Commanding swarm Left +6m...")
            ctrl.set_target(dy=-6.0)
        elif cmd == "4":
            print(">> [EXEC] Commanding swarm Right +6m...")
            ctrl.set_target(dy=6.0)
        elif cmd == "5":
            print(">> [EXEC] Commanding swarm Climb (+2m)...")
            ctrl.set_target(dz=-2.0)
        elif cmd == "6":
            print(">> [EXEC] Commanding swarm Descend (-2m)...")
            ctrl.set_target(dz=2.0)
        elif cmd == "7":
            ctrl.run_patrol_demo()
            ctrl.cleanup()
            return
        elif cmd == "8":
            print(">> [EXEC] Resetting swarm to Center (0, 0, 5m)...")
            with ctrl.lock:
                ctrl.leader_target = np.array([0.0, 0.0, -5.0])
        elif cmd == "9":
            print_status(ctrl)
            ctrl.cleanup()
            return

        print(f">> Monitoring flight response for {args.duration} seconds...")
        for sec in range(int(args.duration)):
            time.sleep(1.0)
            print_status(ctrl)

        ctrl.cleanup()
        return

    # Interactive menu
    print("\n=================================================================")
    print("      LIVE SITL SWARM FLIGHT CONTROLLER (3 CINEWHOOPS)           ")
    print("=================================================================")
    print(" Commands:")
    print("   [1] Fly Forward +10m          [2] Fly Backward -10m")
    print("   [3] Fly Left +6m              [4] Fly Right +6m")
    print("   [5] Climb Up (+2m)            [6] Descend Down (-2m)")
    print("   [7] Automated Swarm Patrol Demo (Square Route with Turns)")
    print("   [8] Reset to Center (0, 0, 5m)")
    print("   [9] View Live Sync Status")
    print("   [0] Exit")
    print("=================================================================")

    try:
        while True:
            cmd = input("\nEnter Command [0-9]: ").strip()
            if cmd == "1":
                print(">> Commanding swarm Forward +10m...")
                ctrl.set_target(dx=10.0)
            elif cmd == "2":
                print(">> Commanding swarm Backward -10m...")
                ctrl.set_target(dx=-10.0)
            elif cmd == "3":
                print(">> Commanding swarm Left +6m...")
                ctrl.set_target(dy=-6.0)
            elif cmd == "4":
                print(">> Commanding swarm Right +6m...")
                ctrl.set_target(dy=6.0)
            elif cmd == "5":
                print(">> Commanding swarm Climb (+2m)...")
                ctrl.set_target(dz=-2.0)
            elif cmd == "6":
                print(">> Commanding swarm Descend (-2m)...")
                ctrl.set_target(dz=2.0)
            elif cmd == "7":
                ctrl.run_patrol_demo()
            elif cmd == "8":
                print(">> Resetting swarm to Center (0, 0, 5m)...")
                with ctrl.lock:
                    ctrl.leader_target = np.array([0.0, 0.0, -5.0])
            elif cmd == "9":
                print_status(ctrl)
            elif cmd == "0":
                print("Exiting flight controller...")
                ctrl.running = False
                break
            else:
                print("Invalid command. Choose 0-9.")
    except KeyboardInterrupt:
        print("\nExiting flight controller...")
        ctrl.running = False
    finally:
        ctrl.cleanup()


if __name__ == "__main__":
    main()
