"""
Autonomous Wingman: Leader-Follower Swarm Formation Controller.
Connects directly to Drone 1 (TCP 5762) and Drone 2 (TCP 5772).
- Automatically arms and launches Drone 2 to 5m.
- Actively tracks Drone 1's position and commands Drone 2 to shadow it at a fixed formation offset.
- As the user flies Drone 1 in QGroundControl, Drone 2 autonomously follows it!
"""

import math
import sys
import time
from typing import Optional, Tuple
import numpy as np
from pymavlink import mavutil


class DroneNode:
    """Manages connection and state for a single SITL vehicle over TCP."""

    def __init__(self, tcp_port: int, sysid: int, label: str):
        self.tcp_port = tcp_port
        self.sysid = sysid
        self.label = label
        self.conn = None
        self.local_pos = np.zeros(3)  # [North, East, Down] in meters
        self.yaw_deg = 0.0
        self.is_armed = False
        self.mode = "UNKNOWN"

    def connect(self) -> bool:
        print(f"Connecting to {self.label} on tcp:127.0.0.1:{self.tcp_port}...")
        self.conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.tcp_port}")
        msg = self.conn.wait_heartbeat(timeout=5.0)
        if not msg:
            print(f" [FAILED] No heartbeat from {self.label}")
            return False

        self.mode = mavutil.mode_string_v10(msg)
        self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f" [CONNECTED] {self.label}: SYSID={self.conn.target_system}, Mode={self.mode}, Armed={self.is_armed}")

        # Request high-frequency position telemetry (10 Hz)
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1,
        )
        return True

    def poll_telemetry(self) -> None:
        """Poll incoming MAVLink packets and update local NED position."""
        while True:
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg)
                self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif mtype == "LOCAL_POSITION_NED":
                self.local_pos[0] = msg.x  # North
                self.local_pos[1] = msg.y  # East
                self.local_pos[2] = msg.z  # Down (negative up)
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
            1,  # 1 = Arm
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
        """Stream position setpoint in local NED (MAV_FRAME_LOCAL_NED)."""
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


def main():
    print("=================================================================")
    print("  Swarm Drones FYP: Autonomous Leader-Follower Wingman")
    print("=================================================================")

    d1 = DroneNode(tcp_port=5762, sysid=1, label="Drone 1 (Leader)")
    d2 = DroneNode(tcp_port=5772, sysid=2, label="Drone 2 (Wingman)")

    if not d1.connect() or not d2.connect():
        print("[ERROR] Could not connect to drones on TCP ports 5762 / 5772.")
        sys.exit(1)

    # Prime telemetry
    for _ in range(10):
        d1.poll_telemetry()
        d2.poll_telemetry()
        time.sleep(0.1)

    print("\n-> Step 1: Checking Drone 2 status...")
    if not d2.is_armed:
        print("   Setting Drone 2 to GUIDED mode...")
        d2.set_mode("GUIDED")
        print("   Arming Drone 2...")
        d2.arm()
        time.sleep(1.0)
        print("   Taking off Drone 2 to 5.0 meters...")
        d2.takeoff(5.0)

        # Wait for Drone 2 to reach cruising altitude
        print("   Ascending... (waiting for 5m altitude)")
        for _ in range(18):
            d1.poll_telemetry()
            d2.poll_telemetry()
            alt2 = -d2.local_pos[2]
            print(f"    - Drone 2 Altitude: {alt2:.1f} m")
            if alt2 >= 4.5:
                print("   Drone 2 reached cruising altitude!")
                break
            time.sleep(1.0)
    else:
        print("   Drone 2 is already airborne!")

    # Formation offset in meters: [North, East, Down]
    # E.g. [0.0, 5.0, 0.0] -> 5 meters East of Leader
    offset_ned = np.array([0.0, 5.0, 0.0])

    print("\n=================================================================")
    print("  AUTONOMOUS WINGMAN ENGAGED!")
    print("  Formation Offset: 5.0 meters East of Drone 1")
    print("  Now fly Drone 1 in QGroundControl — Drone 2 will follow it!")
    print("  Press Ctrl+C to stop.")
    print("=================================================================\n")

    t_start = time.time()
    try:
        while True:
            d1.poll_telemetry()
            d2.poll_telemetry()

            # Target position for Drone 2 is Leader Position + Offset
            target_d2 = d1.local_pos + offset_ned
            # Ensure safe altitude matches leader or maintains at least 5m
            target_d2[2] = min(-4.5, d1.local_pos[2])

            # Send formation command to Drone 2 at 10 Hz
            d2.send_target_position(target_d2[0], target_d2[1], target_d2[2])

            # Calculate actual inter-drone distance and error
            actual_dist = np.linalg.norm(d1.local_pos - d2.local_pos)
            error = abs(actual_dist - 5.0)

            t_elapsed = time.time() - t_start
            sys.stdout.write(
                f"\r[{t_elapsed:5.1f}s] D1 (Lead): ({d1.local_pos[0]:5.1f}, {d1.local_pos[1]:5.1f}, {-d1.local_pos[2]:4.1f}m) | "
                f"D2 (Follow): ({d2.local_pos[0]:5.1f}, {d2.local_pos[1]:5.1f}, {-d2.local_pos[2]:4.1f}m) | "
                f"Dist: {actual_dist:4.2f}m (Error: {error:4.2f}m)   "
            )
            sys.stdout.flush()
            time.sleep(0.1)  # 10 Hz

    except KeyboardInterrupt:
        print("\n\nWingman controller stopped. Drone 2 hovering safely in place.")


if __name__ == "__main__":
    main()
