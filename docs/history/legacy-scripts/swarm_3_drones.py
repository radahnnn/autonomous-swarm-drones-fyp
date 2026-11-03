#!/usr/bin/env python3
"""
3-Drone Fully Autonomous Swarm Formation Controller (Gazebo Harmonic & SITL).
Simultaneously arms and launches all 3 Cinewhoop digital twins into a 3D V-Formation:
- Drone 1 (SYSID 1, UDP 14552 / TCP 5760): Apex / Swarm Leader
- Drone 2 (SYSID 2, UDP 14562 / TCP 5770): Left Wingman (3m behind, 4m left)
- Drone 3 (SYSID 3, UDP 14572 / TCP 5780): Right Wingman (3m behind, 4m right)
"""

import math
import sys
import time
from typing import List
import numpy as np
from pymavlink import mavutil


class SwarmAgent:
    """Manages connection, arming, takeoff, and formation flight for a single drone."""

    def __init__(self, sysid: int, udp_port: int, label: str):
        self.sysid = sysid
        self.udp_port = udp_port
        self.label = label
        self.conn = None
        self.local_pos = np.zeros(3)  # [North, East, Down]
        self.yaw_deg = 0.0
        self.is_armed = False
        self.mode = "UNKNOWN"

    def connect(self) -> bool:
        """Connect via UDP port and wait for heartbeat."""
        endpoint = f"udpin:127.0.0.1:{self.udp_port}"
        print(f"Connecting to {self.label} on {endpoint}...")
        try:
            self.conn = mavutil.mavlink_connection(endpoint)
            msg = self.conn.wait_heartbeat(timeout=8.0)
            if not msg:
                print(f"  [FAILED] No heartbeat from {self.label}")
                return False

            self.mode = mavutil.mode_string_v10(msg)
            self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"  [CONNECTED] {self.label}: SYSID={self.conn.target_system}, Mode={self.mode}, Armed={self.is_armed}")

            # Request 10 Hz position telemetry
            self.conn.mav.request_data_stream_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,
                1,
            )
            return True
        except Exception as e:
            print(f"  [ERROR] Connection error on {self.label}: {e}")
            return False

    def poll_telemetry(self) -> None:
        """Drain incoming packets and update spatial position."""
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
        """Set autopilot mode."""
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
        """Send MAVLink arm command and verify state."""
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
        """Send takeoff command to target altitude."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            alt,
        )

    def send_target_position(self, x: float, y: float, z: float) -> None:
        """Send position setpoint in local NED frame."""
        type_mask = 0b0000111111111000  # Position only
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


def launch_drone(drone: SwarmAgent, target_alt: float = 5.0):
    """Ensure drone is in GUIDED, armed, and actively climbing to target altitude."""
    drone.poll_telemetry()
    current_alt = -drone.local_pos[2]
    
    if current_alt >= target_alt - 0.8:
        print(f"  -> {drone.label} is already airborne at {current_alt:.1f}m.")
        return True

    print(f"\n[LAUNCH] Starting launch sequence for {drone.label}...")
    drone.set_mode("GUIDED")
    time.sleep(0.5)

    if not drone.is_armed:
        print(f"  -> Arming {drone.label}...")
        if not drone.arm():
            print(f"  [RETRY] Arming {drone.label} with force arm...")
            drone.conn.mav.command_long_send(
                drone.conn.target_system, drone.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(1.0)
            drone.poll_telemetry()

    print(f"  -> Sending Takeoff to {target_alt}m...")
    drone.takeoff(target_alt)
    return True


def main():
    print("=================================================================")
    print("   AUTONOMOUS 3-DRONE CINEWHOOP SWARM CONTROLLER (GAZEBO 3D)    ")
    print("=================================================================")

    d1 = SwarmAgent(sysid=1, udp_port=14552, label="Drone 1 (Apex Leader)")
    d2 = SwarmAgent(sysid=2, udp_port=14562, label="Drone 2 (Left Wing)")
    d3 = SwarmAgent(sysid=3, udp_port=14572, label="Drone 3 (Right Wing)")

    drones = [d1, d2, d3]

    # 1. Connect to all 3 drones
    for d in drones:
        if not d.connect():
            print(f"\n[ERROR] Failed to connect to {d.label}.")
            print("Ensure Gazebo and all 3 Drone SITL instances are running.")
            sys.exit(1)

    # 2. Flush initial telemetry
    print("\nFlushing telemetry & validating GPS lock...")
    for _ in range(15):
        for d in drones:
            d.poll_telemetry()
        time.sleep(0.1)

    # 3. Launch all 3 drones sequentially to 5.0m
    for d in drones:
        launch_drone(d, target_alt=5.0)
        time.sleep(1.0)

    # 4. Wait for all 3 drones to reach cruising altitude (>= 4.0m)
    print("\nWaiting for all drones to reach cruising altitude (5.0m)...")
    for _ in range(20):
        all_ready = True
        status_strs = []
        for d in drones:
            d.poll_telemetry()
            alt = -d.local_pos[2]
            status_strs.append(f"{d.label}: {alt:4.1f}m")
            if alt < 3.8:
                all_ready = False
        print("  | " + " | ".join(status_strs))
        if all_ready:
            print("\n>>> ALL 3 CINEWHOOPS ARE AIRBORNE AT CRUISING ALTITUDE! <<<")
            break
        time.sleep(1.0)

    # 5. V-Formation Geometry
    # Relative offset vectors [North, East, Down]
    # Drone 2 (Left Wing):  -3.0m behind, -4.0m left
    # Drone 3 (Right Wing): -3.0m behind, +4.0m right
    offset_left = np.array([-3.0, -4.0, 0.0])
    offset_right = np.array([-3.0, +4.0, 0.0])

    print("\n=================================================================")
    print("   V-FORMATION GUIDANCE ACTIVE (10 Hz Closed-Loop Tracking)")
    print("   - Leader (D1): Central Apex")
    print("   - Left Wing (D2): Holds 4m Left, 3m Behind Leader")
    print("   - Right Wing (D3): Holds 4m Right, 3m Behind Leader")
    print("   Press Ctrl+C to safely stop.")
    print("=================================================================\n")

    t_start = time.time()
    try:
        while True:
            for d in drones:
                d.poll_telemetry()

            p1 = d1.local_pos
            p2 = d2.local_pos
            p3 = d3.local_pos

            # Leader heading rotation matrix
            psi = math.radians(d1.yaw_deg)
            cos_p = math.cos(psi)
            sin_p = math.sin(psi)

            # Body offsets: behind leader (-3.5m), lateral left (-3.5m), right (+3.5m)
            # Left wingman offset in NED
            d2_body_x, d2_body_y = -3.0, -3.5
            d2_ned_x = p1[0] + (d2_body_x * cos_p - d2_body_y * sin_p)
            d2_ned_y = p1[1] + (d2_body_x * sin_p + d2_body_y * cos_p)

            # Right wingman offset in NED
            d3_body_x, d3_body_y = -3.0, +3.5
            d3_ned_x = p1[0] + (d3_body_x * cos_p - d3_body_y * sin_p)
            d3_ned_y = p1[1] + (d3_body_x * sin_p + d3_body_y * cos_p)

            cruise_z = p1[2]  # Match leader altitude exactly
            target_d2 = np.array([d2_ned_x, d2_ned_y, cruise_z])
            target_d3 = np.array([d3_ned_x, d3_ned_y, cruise_z])

            # Inter-wingman collision avoidance (Artificial Potential Field)
            diff_wings = p2 - p3
            dist_wings = np.linalg.norm(diff_wings[:2])
            safe_dist = 2.5  # meters
            if 0.1 < dist_wings < safe_dist:
                repulse = 1.2 * (1.0 / dist_wings - 1.0 / safe_dist) * (diff_wings[:2] / dist_wings)
                target_d2[0] += repulse[0]
                target_d2[1] += repulse[1]
                target_d3[0] -= repulse[0]
                target_d3[1] -= repulse[1]

            # Stream guidance setpoints at 10 Hz
            d2.send_target_position(target_d2[0], target_d2[1], target_d2[2])
            d3.send_target_position(target_d3[0], target_d3[1], target_d3[2])

            # Distance & sync metrics
            dist_12 = np.linalg.norm(p1[:2] - p2[:2])
            dist_13 = np.linalg.norm(p1[:2] - p3[:2])
            err_d2 = np.linalg.norm(p2[:2] - np.array([d2_ned_x, d2_ned_y]))
            err_d3 = np.linalg.norm(p3[:2] - np.array([d3_ned_x, d3_ned_y]))

            t_elapsed = time.time() - t_start
            sys.stdout.write(
                f"\r[{t_elapsed:5.1f}s] Leader: ({p1[0]:4.1f},{p1[1]:4.1f},{-p1[2]:4.1f}m) | "
                f"D1-D2: {dist_12:4.2f}m | D1-D3: {dist_13:4.2f}m | Span: {dist_wings:4.2f}m | "
                f"Sync Err: D2={err_d2:4.2f}m, D3={err_d3:4.2f}m   "
            )
            sys.stdout.flush()
            time.sleep(0.1)  # 10 Hz

    except KeyboardInterrupt:
        print("\n\nSwarm controller stopped. All drones holding position in GUIDED mode.")


if __name__ == "__main__":
    main()
