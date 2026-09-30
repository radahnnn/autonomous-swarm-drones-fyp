"""
3-Drone Autonomous Swarm Formation Controller (ArduPilot SITL).
Coordinates a 3-drone V-Formation (Chevron / Flying-V) in real-time:
- Drone 1 (SYSID 1, TCP 5762): Apex / Flight Leader (piloted via QGroundControl or hovering).
- Drone 2 (SYSID 2, TCP 5772): Right Wingman (holds 4m East, 3m Behind Leader).
- Drone 3 (SYSID 3, TCP 5782): Left Wingman (holds 4m West, 3m Behind Leader).
Includes real-time APF collision avoidance between wingmen.
"""

import math
import sys
import time
from typing import Dict, List, Optional
import numpy as np
from pymavlink import mavutil


class DroneAgent:
    """Manages connection, telemetry, and flight commands for a single SITL copter."""

    def __init__(self, tcp_port: int, sysid: int, label: str):
        self.tcp_port = tcp_port
        self.sysid = sysid
        self.label = label
        self.conn = None
        self.local_pos = np.zeros(3)  # [North, East, Down]
        self.yaw_deg = 0.0
        self.is_armed = False
        self.mode = "UNKNOWN"

    def connect(self) -> bool:
        print(f"Connecting to {self.label} on tcp:127.0.0.1:{self.tcp_port}...")
        self.conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.tcp_port}")
        msg = self.conn.wait_heartbeat(timeout=6.0)
        if not msg:
            print(f" [FAILED] No heartbeat from {self.label}")
            return False

        self.mode = mavutil.mode_string_v10(msg)
        self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f" [CONNECTED] {self.label}: SYSID={self.conn.target_system}, Mode={self.mode}, Armed={self.is_armed}")

        # Request 10 Hz position and attitude telemetry
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1,
        )
        return True

    def poll_telemetry(self) -> None:
        """Drain MAVLink queue and update spatial state."""
        while True:
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg)
                self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif mtype == "LOCAL_POSITION_NED":
                self.local_pos[0] = msg.x
                self.local_pos[1] = msg.y
                self.local_pos[2] = msg.z
            elif mtype == "ATTITUDE":
                self.yaw_deg = math.degrees(msg.yaw)

    def set_mode(self, mode_name: str = "GUIDED") -> bool:
        mode_id = self.conn.mode_mapping().get(mode_name)
        if mode_id is None:
            return False
        self.conn.set_mode(mode_id)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self.poll_telemetry()
            if self.mode == mode_name:
                return True
            time.sleep(0.1)
        return False

    def arm(self) -> bool:
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0,
        )
        t0 = time.time()
        while time.time() - t0 < 4.0:
            self.poll_telemetry()
            if self.is_armed:
                return True
            time.sleep(0.1)
        return False

    def takeoff(self, alt: float = 5.0) -> None:
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            alt,
        )

    def send_target_position(self, x: float, y: float, z: float) -> None:
        type_mask = 0b0000111111111000  # Position setpoint
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )


def main():
    print("=================================================================")
    print("  Swarm Drones FYP: 3-Drone Autonomous V-Formation Controller   ")
    print("=================================================================")

    d1 = DroneAgent(tcp_port=5762, sysid=1, label="Drone 1 (Apex Leader)")
    d2 = DroneAgent(tcp_port=5772, sysid=2, label="Drone 2 (Right Wing)")
    d3 = DroneAgent(tcp_port=5782, sysid=3, label="Drone 3 (Left Wing)")

    drones = [d1, d2, d3]

    for d in drones:
        if not d.connect():
            print(f"\n[ERROR] Could not connect to {d.label}.")
            print("Make sure Drones 1, 2, and 3 are running in their respective terminals.")
            sys.exit(1)

    # Initial telemetry flush
    for _ in range(10):
        for d in drones:
            d.poll_telemetry()
        time.sleep(0.1)

    # Launch Wingmen if needed
    for follower in [d2, d3]:
        if not follower.is_armed:
            print(f"\n-> Launching {follower.label} to 5.0 meters...")
            follower.set_mode("GUIDED")
            follower.arm()
            time.sleep(1.0)
            follower.takeoff(5.0)

            # Wait for ascent
            for _ in range(15):
                follower.poll_telemetry()
                alt = -follower.local_pos[2]
                print(f"    - {follower.label} Alt: {alt:.1f} m")
                if alt >= 4.5:
                    print(f"   {follower.label} reached cruising altitude!")
                    break
                time.sleep(1.0)
        else:
            print(f"{follower.label} is already airborne!")

    # V-Formation Relative Offsets: [North, East, Down]
    # Leader at [0, 0, 0]
    # Right Wingman: 3m behind (-X), 4m right (+Y)
    # Left Wingman: 3m behind (-X), 4m left (-Y)
    offset_right = np.array([-3.0, +4.0, 0.0])
    offset_left = np.array([-3.0, -4.0, 0.0])

    print("\n=================================================================")
    print("  3-DRONE V-FORMATION ENGAGED!")
    print("  - Drone 1 (Apex): Leader (Fly with QGroundControl)")
    print("  - Drone 2 (Right Wing): +4m East, -3m North")
    print("  - Drone 3 (Left Wing):  -4m East, -3m North")
    print("  Press Ctrl+C to stop.")
    print("=================================================================\n")

    t_start = time.time()
    try:
        while True:
            for d in drones:
                d.poll_telemetry()

            p1 = d1.local_pos
            p2 = d2.local_pos
            p3 = d3.local_pos

            # Calculate nominal targets relative to Leader
            target_d2 = p1 + offset_right
            target_d3 = p1 + offset_left

            # Maintain safe hovering altitude matching Leader
            cruise_z = min(-4.5, p1[2])
            target_d2[2] = cruise_z
            target_d3[2] = cruise_z

            # APF Collision avoidance between the two wingmen (D2 and D3)
            diff_wing = p2 - p3
            dist_wing = np.linalg.norm(diff_wing[:2])
            safe_wing_dist = 2.5  # meters
            if 0.1 < dist_wing < safe_wing_dist:
                repulse = 1.5 * (1.0 / dist_wing - 1.0 / safe_wing_dist) * (diff_wing[:2] / dist_wing)
                target_d2[0] += repulse[0]
                target_d2[1] += repulse[1]
                target_d3[0] -= repulse[0]
                target_d3[1] -= repulse[1]

            # Stream position setpoints at 10 Hz
            d2.send_target_position(target_d2[0], target_d2[1], target_d2[2])
            d3.send_target_position(target_d3[0], target_d3[1], target_d3[2])

            # Performance metrics
            dist_12 = np.linalg.norm(p1 - p2)
            dist_13 = np.linalg.norm(p1 - p3)

            t_elapsed = time.time() - t_start
            sys.stdout.write(
                f"\r[{t_elapsed:5.1f}s] Apex D1: ({p1[0]:4.1f}, {p1[1]:4.1f}) | "
                f"D1-D2: {dist_12:4.2f}m | D1-D3: {dist_13:4.2f}m | Wing Span: {dist_wing:4.2f}m   "
            )
            sys.stdout.flush()
            time.sleep(0.1)  # 10 Hz

    except KeyboardInterrupt:
        print("\n\n3-Drone Swarm Controller stopped. All drones hovering safely.")


if __name__ == "__main__":
    main()
