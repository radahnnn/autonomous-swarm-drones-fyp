"""
SITL Telemetry Diagnostic Tool.
Connects via pymavlink to read and verify heartbeats, GPS, and flight modes from simulated drones.
"""

import sys
import time
from typing import Optional
from pymavlink import mavutil


def check_drone_telemetry(connection_string: str, drone_label: str = "Drone", timeout_s: float = 10.0) -> bool:
    print(f"Connecting to {drone_label} on {connection_string}...")
    try:
        # Create MAVLink connection
        conn = mavutil.mavlink_connection(connection_string)
        
        # Wait for first heartbeat
        start_time = time.time()
        print(f" -> Waiting for heartbeat (timeout: {timeout_s}s)...")
        msg = conn.wait_heartbeat(timeout=timeout_s)
        
        if msg is None:
            print(f" [FAILED] No heartbeat received from {drone_label} within {timeout_s}s.")
            return False
            
        sysid = conn.target_system
        compid = conn.target_component
        mode = mavutil.mode_string_v10(msg)
        is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        
        print(f" [CONNECTED] {drone_label} responded!")
        print(f"    - System ID: {sysid}")
        print(f"    - Component ID: {compid}")
        print(f"    - Autopilot Mode: {mode}")
        print(f"    - Armed: {is_armed}")
        
        # Request data streams
        conn.mav.request_data_stream_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # 4 Hz
            1,  # Start
        )
        
        # Wait for GPS or local position
        print(" -> Reading spatial coordinates...")
        pos_msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=5.0)
        if pos_msg:
            if pos_msg.get_type() == "GLOBAL_POSITION_INT":
                lat = pos_msg.lat / 1e7
                lon = pos_msg.lon / 1e7
                alt = pos_msg.relative_alt / 1000.0  # mm to m
                print(f"    - Global Pos: Lat={lat:.6f}, Lon={lon:.6f}, Rel Alt={alt:.2f} m")
            elif pos_msg.get_type() == "LOCAL_POSITION_NED":
                print(f"    - Local NED: X={pos_msg.x:.2f}m, Y={pos_msg.y:.2f}m, Z={pos_msg.z:.2f}m")
        else:
            print("    - Position: Still acquiring GPS fix in SITL.")
            
        return True
        
    except Exception as e:
        print(f" [ERROR] Exception connecting to {drone_label}: {e}")
        return False


def main():
    endpoints = [
        ("udpin:127.0.0.1:14550", "Drone 1 (SYSID 1)"),
        ("udpin:127.0.0.1:14560", "Drone 2 (SYSID 2)"),
    ]
    
    print("=================================================================")
    print("  ArduPilot SITL Telemetry Verification Check")
    print("=================================================================")
    
    all_ok = True
    for conn_str, label in endpoints:
        ok = check_drone_telemetry(conn_str, label)
        print("-----------------------------------------------------------------")
        if not ok:
            all_ok = False
            
    if all_ok:
        print("All simulated drones verified and communicating successfully!")
    else:
        print("One or more drones failed to respond. Check SITL launch status.")


if __name__ == "__main__":
    main()
