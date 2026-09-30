#!/usr/bin/env python3
"""
Real-Time Swarm Synchronization HUD & Formation Health Monitor.
Connects to all 3 Cinewhoop SITL instances and displays:
- Real-time 3D NED coordinates & cruising altitudes
- Inter-drone clearance & wingspan geometry
- Formation symmetry & synchronization health index
- Interactive Leader Tracking test
"""

import sys
import time
import math
import numpy as np
from pymavlink import mavutil

DRONE_CONFIGS = [
    {"sysid": 1, "port": 14552, "role": "Leader (Apex)"},
    {"sysid": 2, "port": 14562, "role": "Wingman Left"},
    {"sysid": 3, "port": 14572, "role": "Wingman Right"},
]


class DroneTelemetry:
    def __init__(self, sysid: int, port: int, role: str):
        self.sysid = sysid
        self.port = port
        self.role = role
        self.conn = None
        self.pos = np.zeros(3)  # [North, East, Down]
        self.vel = np.zeros(3)  # [Vn, Ve, Vd]
        self.yaw = 0.0
        self.mode = "DISCONN"
        self.armed = False
        self.connected = False

    def connect(self) -> bool:
        endpoint = f"udpin:127.0.0.1:{self.port}"
        try:
            self.conn = mavutil.mavlink_connection(endpoint)
            msg = self.conn.wait_heartbeat(timeout=3.0)
            if msg:
                self.connected = True
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
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
                self.vel[0] = msg.vx
                self.vel[1] = msg.vy
                self.vel[2] = msg.vz
            elif mtype == "ATTITUDE":
                self.yaw = math.degrees(msg.yaw)


def print_hud(drones):
    p1, p2, p3 = drones[0].pos, drones[1].pos, drones[2].pos

    d12 = np.linalg.norm(p1[:2] - p2[:2])
    d13 = np.linalg.norm(p1[:2] - p3[:2])
    wingspan = np.linalg.norm(p2[:2] - p3[:2])

    alt1 = -p1[2]
    alt2 = -p2[2]
    alt3 = -p3[2]
    alt_max_delta = max(abs(alt1 - alt2), abs(alt1 - alt3), abs(alt2 - alt3))
    symmetry_delta = abs(d12 - d13)

    # Health score (100% when delta < 0.1m)
    health = max(0.0, min(100.0, 100.0 - (alt_max_delta * 15.0 + symmetry_delta * 20.0)))
    status = "LOCKED / IN SYNC" if health >= 90.0 else ("CONVERGING" if health >= 60.0 else "UNSYNCHRONIZED")

    # Clear terminal
    sys.stdout.write("\033[2J\033[H")
    
    print("================================================================================")
    print("             AUTONOMOUS SWARM SYNCHRONIZATION HUD (3 CINEWHOOPS)                ")
    print("================================================================================")
    print(f" Swarm Status: [{status}]    |    Sync Health Index: {health:5.1f} / 100.0%")
    print("--------------------------------------------------------------------------------")
    print("  DRONE ID   | ROLE            | MODE   | ARMED | ALTITUDE | NORTH    | EAST")
    print("--------------------------------------------------------------------------------")
    for d in drones:
        arm_str = "ARMED" if d.armed else "DISARM"
        print(f"  Drone {d.sysid:<4} | {d.role:<15} | {d.mode:<6} | {arm_str:<6}| {-d.pos[2]:6.2f}m  | {d.pos[0]:6.2f}m  | {d.pos[1]:6.2f}m")
    print("--------------------------------------------------------------------------------")
    print("  FORMATION GEOMETRY (FLYING-V)")
    print(f"  • Leader - Left Wing (D1 -> D2):  {d12:5.2f} m")
    print(f"  • Leader - Right Wing (D1 -> D3): {d13:5.2f} m")
    print(f"  • Wing-to-Wing Clearance (D2-D3): {wingspan:5.2f} m   (Safe threshold: >= 2.50m)")
    print(f"  • Altitude Divergence:            {alt_max_delta:5.2f} m   (Tolerance: < 0.20m)")
    print(f"  • V-Shape Symmetry Error:         {symmetry_delta:5.2f} m   (Tolerance: < 0.15m)")
    print("--------------------------------------------------------------------------------")
    print("  ASCII FORMATION RADAR VIEW (Top-Down):")
    print("")
    print(f"                     [D1 Apex Leader]  ({p1[0]:4.1f}, {p1[1]:4.1f}, {alt1:4.1f}m)")
    print(f"                       /           \\")
    print(f"                      /             \\     Distance: {d13:4.2f}m")
    print(f"  Distance: {d12:4.2f}m  /               \\")
    print(f"                    /                 \\")
    print(f"     [D2 Left Wing] <-----------------> [D3 Right Wing]")
    print(f"  ({p2[0]:4.1f}, {p2[1]:4.1f}, {alt2:4.1f}m)    Wingspan: {wingspan:4.2f}m    ({p3[0]:4.1f}, {p3[1]:4.1f}, {alt3:4.1f}m)")
    print("")
    print("================================================================================")
    print(" HOW TO TEST REAL-TIME SYNC:")
    print(" 1. In Drone 1's MAVProxy tab, type:  forward 10")
    print(" 2. Watch D2 and D3 track D1 forward in lockstep, holding the exact same V-shape!")
    print(" 3. To return back, type:  back 10")
    print("================================================================================")
    print(" Press Ctrl+C to exit HUD.")
    sys.stdout.flush()


def main():
    drones = [DroneTelemetry(c["sysid"], c["port"], c["role"]) for c in DRONE_CONFIGS]
    print("Connecting to all 3 drones...")
    for d in drones:
        if not d.connect():
            print(f"Warning: Could not connect to {d.role} on port {d.port}")

    try:
        while True:
            for d in drones:
                d.update()
            print_hud(drones)
            time.sleep(0.2)  # 5 Hz refresh
    except KeyboardInterrupt:
        print("\nHUD closed.")


if __name__ == "__main__":
    main()
