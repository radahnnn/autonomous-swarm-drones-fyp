"""
ArduPilot SITL Multi-Drone MAVLink Swarm Bridge.
Controls 2 simulated ArduCopter instances via pymavlink:
1. Connects to Drone 1 (UDP 14550) and Drone 2 (UDP 14560).
2. Sets GUIDED mode, arms both vehicles, and coordinates takeoff to 5m.
3. Streams SET_POSITION_TARGET_LOCAL_NED commands so Drone 2 holds a line formation offset from Drone 1.
4. Monitors inter-drone distance and formation tracking in real-time.
"""

import sys
import time
from typing import Dict, Optional, Tuple
import numpy as np
from pymavlink import mavutil


class SITLDroneController:
    """Manages MAVLink communication and flight commands for a single SITL vehicle."""

    def __init__(self, connection_string: str, expected_sysid: int, label: str):
        self.conn_str = connection_string
        self.expected_sysid = expected_sysid
        self.label = label
        self.master = None
        self.local_pos = np.zeros(3)  # [x, y, z] in NED (z is negative up)
        self.is_armed = False
        self.current_mode = "UNKNOWN"

    def connect(self, timeout_s: float = 15.0) -> bool:
        print(f"Connecting to {self.label} on {self.conn_str}...")
        self.master = mavutil.mavlink_connection(self.conn_str)
        msg = self.master.wait_heartbeat(timeout=timeout_s)
        if not msg:
            print(f" [ERROR] Timeout waiting for heartbeat from {self.label} on {self.conn_str}")
            return False

        self.current_mode = mavutil.mode_string_v10(msg)
        self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f" [CONNECTED] {self.label}: SYSID={self.master.target_system}, Mode={self.current_mode}, Armed={self.is_armed}")

        # Request high-rate position telemetry
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,  # 10 Hz
            1,   # Start
        )
        return True

    def update_telemetry(self) -> None:
        """Poll incoming MAVLink packets and update local state."""
        while True:
            msg = self.master.recv_match(blocking=False)
            if not msg:
                break
            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self.current_mode = mavutil.mode_string_v10(msg)
                self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == "LOCAL_POSITION_NED":
                self.local_pos[0] = msg.x
                self.local_pos[1] = msg.y
                self.local_pos[2] = msg.z  # NED z: negative is above ground

    def set_mode(self, mode_name: str = "GUIDED") -> bool:
        """Command vehicle flight mode."""
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f" [ERROR] Unknown mode {mode_name}")
            return False

        self.master.set_mode(mode_id)
        # Wait up to 3 seconds for mode confirmation
        t_start = time.time()
        while time.time() - t_start < 3.0:
            self.update_telemetry()
            if self.current_mode == mode_name:
                return True
            time.sleep(0.1)
        return False

    def arm(self) -> bool:
        """Send arm command."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # 1 = Arm
            0, 0, 0, 0, 0, 0,
        )
        t_start = time.time()
        while time.time() - t_start < 4.0:
            self.update_telemetry()
            if self.is_armed:
                return True
            time.sleep(0.1)
        return False

    def takeoff(self, target_alt: float = 5.0) -> None:
        """Send takeoff command in meters."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_alt,
        )

    def send_local_ned_target(self, x: float, y: float, z: float) -> None:
        """
        Stream local NED position setpoint (MAV_FRAME_LOCAL_NED).
        Type mask: 0b0000111111111000 (ignore velocity, accel, yaw).
        Only position (x, y, z) is commanded.
        """
        # Type mask: bits 0-2 (pos enabled), bits 3-5 (vel disabled), bits 6-8 (accel disabled), 10 (yaw disabled)
        type_mask = 0b0000111111111000
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,          # pos (x=North, y=East, z=Down)
            0, 0, 0,          # vel
            0, 0, 0,          # accel
            0, 0,             # yaw, yaw_rate
        )


def main():
    print("=================================================================")
    print("  Swarm Drones FYP: 2-Drone SITL MAVLink Formation Controller   ")
    print("=================================================================")

    # 1. Connect to both vehicles
    d1 = SITLDroneController("udpin:127.0.0.1:14550", expected_sysid=1, label="Drone 1 (Lead)")
    d2 = SITLDroneController("udpin:127.0.0.1:14560", expected_sysid=2, label="Drone 2 (Follower)")

    if not d1.connect() or not d2.connect():
        print("\n[ABORT] Could not connect to both SITL instances.")
        print("Make sure both ArduCopter instances are running in separate terminals.")
        sys.exit(1)

    print("\n-> Step 1: Switching both drones to GUIDED mode...")
    if not d1.set_mode("GUIDED"):
        print("Warning: Could not confirm GUIDED for Drone 1 (check EKF status)")
    if not d2.set_mode("GUIDED"):
        print("Warning: Could not confirm GUIDED for Drone 2 (check EKF status)")

    print("\n-> Step 2: Arming both drones...")
    d1.arm()
    d2.arm()
    time.sleep(1.0)
    d1.update_telemetry()
    d2.update_telemetry()
    print(f"   Drone 1 Armed: {d1.is_armed} | Drone 2 Armed: {d2.is_armed}")

    print("\n-> Step 3: Commanding coordinated takeoff to 5.0 meters...")
    d1.takeoff(5.0)
    d2.takeoff(5.0)

    # Monitor ascent
    print("   Ascending... (waiting for target altitude)")
    for _ in range(15):
        d1.update_telemetry()
        d2.update_telemetry()
        print(f"   Altitudes -> D1: {-d1.local_pos[2]:.1f}m | D2: {-d2.local_pos[2]:.1f}m")
        if -d1.local_pos[2] >= 4.5 and -d2.local_pos[2] >= 4.5:
            print("   Both drones reached cruising altitude!")
            break
        time.sleep(1.0)

    print("\n-> Step 4: Activating Line Formation Offset Control...")
    print("   Target: Drone 1 at (0, 0, -5m) | Drone 2 at (3m East: 0, 3, -5m)")
    print("   Streaming MAVLink SET_POSITION_TARGET_LOCAL_NED at 5 Hz...")
    print("   Press Ctrl+C to stop.\n")

    t_start = time.time()
    try:
        while True:
            d1.update_telemetry()
            d2.update_telemetry()

            # Drone 1 holds local reference origin at 5m altitude
            d1.send_local_ned_target(0.0, 0.0, -5.0)

            # Drone 2 holds 3m East offset (+Y in NED)
            d2.send_local_ned_target(0.0, 3.0, -5.0)

            # Compute relative distance and tracking error
            p1 = d1.local_pos
            p2 = d2.local_pos
            actual_dist = np.linalg.norm(p1 - p2)

            t_elapsed = time.time() - t_start
            sys.stdout.write(
                f"\r[{t_elapsed:5.1f}s] D1 Pos: ({p1[0]:4.1f}, {p1[1]:4.1f}, {-p1[2]:4.1f}m) | "
                f"D2 Pos: ({p2[0]:4.1f}, {p2[1]:4.1f}, {-p2[2]:4.1f}m) | "
                f"Inter-Drone Dist: {actual_dist:4.2f}m"
            )
            sys.stdout.flush()
            time.sleep(0.2)  # 5 Hz

    except KeyboardInterrupt:
        print("\n\nStopping formation stream. Drones hovering safely.")


if __name__ == "__main__":
    main()
