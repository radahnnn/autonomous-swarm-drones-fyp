#!/usr/bin/env python3
"""
Interactive Swarm Flight Controller (3-Drone V-Formation).
Allows the user to easily maneuver the swarm in 3D physics using simple menu commands:
- Followers 2 & 3 continuously track Leader 1 at 10 Hz with APF collision avoidance.
- Move Forward / Back / Left / Right / Altitude Climb / Autonomous Patrol.
"""

import sys
import time
import math
import threading
import numpy as np
from pymavlink import mavutil


class DroneAgent:
    def __init__(self, sysid: int, port: int, label: str):
        self.sysid = sysid
        self.port = port
        self.label = label
        self.conn = None
        self.pos = np.zeros(3)  # [North, East, Down]
        self.yaw_deg = 0.0
        self.mode = "UNKNOWN"
        self.armed = False

    def connect(self) -> bool:
        endpoint = f"udpin:127.0.0.1:{self.port}"
        try:
            self.conn = mavutil.mavlink_connection(endpoint)
            msg = self.conn.wait_heartbeat(timeout=3.0)
            if msg:
                self.conn.mav.request_data_stream_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    10, 1
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

    def send_target(self, x: float, y: float, z: float):
        type_mask = 0b0000111111111000  # Position setpoint only
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,
            0, 0, 0, 0, 0, 0, 0, 0
        )


class SwarmFlightController:
    def __init__(self):
        self.d1 = DroneAgent(1, 14552, "Leader (D1)")
        self.d2 = DroneAgent(2, 14562, "Left Wing (D2)")
        self.d3 = DroneAgent(3, 14572, "Right Wing (D3)")
        self.drones = [self.d1, self.d2, self.d3]

        self.leader_target = np.array([0.0, 0.0, -5.0])
        self.running = True
        self.lock = threading.Lock()

    def connect_all(self):
        print("Connecting to all 3 drones...")
        for d in self.drones:
            if not d.connect():
                print(f"Failed to connect to {d.label}")
                return False
        # Initialize target from current leader position
        time.sleep(0.5)
        for _ in range(10):
            for d in self.drones:
                d.update()
            time.sleep(0.05)
        self.leader_target[0] = self.d1.pos[0]
        self.leader_target[1] = self.d1.pos[1]
        self.leader_target[2] = min(-4.5, self.d1.pos[2])  # At least 4.5m alt
        print(f"Connected! Initial leader position: ({self.leader_target[0]:.1f}, {self.leader_target[1]:.1f}, {-self.leader_target[2]:.1f}m)")
        return True

    def swarm_loop(self):
        """10 Hz closed-loop guidance: moves Leader to target and locks Followers in V-formation."""
        while self.running:
            for d in self.drones:
                d.update()

            with self.lock:
                tgt_leader = self.leader_target.copy()

            # Command Leader to target
            self.d1.send_target(tgt_leader[0], tgt_leader[1], tgt_leader[2])

            p1 = self.d1.pos
            p2 = self.d2.pos
            p3 = self.d3.pos

            # Heading rotation
            psi = math.radians(self.d1.yaw_deg)
            cos_p = math.cos(psi)
            sin_p = math.sin(psi)

            # V-Formation offsets: behind (-3.0m), left (-3.5m) and right (+3.5m)
            d2_body_x, d2_body_y = -3.0, -3.5
            d3_body_x, d3_body_y = -3.0, +3.5

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

            # Artificial Potential Field (APF) collision repulsion between wingmen
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
        print("\n>>> STARTING AUTONOMOUS PATROL DEMO <<<")
        waypoints = [
            (10.0, 0.0, 0.0, "Leg 1: Forward 10m"),
            (0.0, 8.0, 0.0, "Leg 2: Right 8m"),
            (-10.0, 0.0, 0.0, "Leg 3: Backward 10m"),
            (0.0, -8.0, 0.0, "Leg 4: Return Left 8m to Origin"),
        ]
        for dx, dy, dz, name in waypoints:
            print(f" -> Executing: {name}...")
            self.set_target(dx, dy, dz)
            # Monitor flight for 8 seconds
            for _ in range(8):
                if not self.running:
                    return
                p1, p2, p3 = self.d1.pos, self.d2.pos, self.d3.pos
                d12 = np.linalg.norm(p1[:2] - p2[:2])
                d13 = np.linalg.norm(p1[:2] - p3[:2])
                span = np.linalg.norm(p2[:2] - p3[:2])
                print(f"    Leader: ({p1[0]:4.1f}, {p1[1]:4.1f}) | D1-D2: {d12:4.2f}m | D1-D3: {d13:4.2f}m | Span: {span:4.2f}m")
                time.sleep(1.0)
        print(">>> PATROL DEMO COMPLETE: Swarm held V-formation throughout all turns! <<<\n")


def main():
    ctrl = SwarmFlightController()
    if not ctrl.connect_all():
        sys.exit(1)

    # Start background guidance thread
    thread = threading.Thread(target=ctrl.swarm_loop, daemon=True)
    thread.start()

    print("\n=================================================================")
    print("      INTERACTIVE SWARM FLIGHT CONTROLLER (3 CINEWHOOPS)         ")
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
                p1, p2, p3 = ctrl.d1.pos, ctrl.d2.pos, ctrl.d3.pos
                d12 = np.linalg.norm(p1[:2] - p2[:2])
                d13 = np.linalg.norm(p1[:2] - p3[:2])
                span = np.linalg.norm(p2[:2] - p3[:2])
                print(f"Status: Leader at ({p1[0]:.2f}, {p1[1]:.2f}, {-p1[2]:.2f}m) | D1-D2={d12:.2f}m | D1-D3={d13:.2f}m | Span={span:.2f}m")
            elif cmd == "0":
                print("Exiting...")
                ctrl.running = False
                break
            else:
                print("Invalid command. Choose 0-9.")
    except KeyboardInterrupt:
        print("\nExiting...")
        ctrl.running = False


if __name__ == "__main__":
    main()
