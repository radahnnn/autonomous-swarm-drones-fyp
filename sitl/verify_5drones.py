#!/usr/bin/env python3
"""
verify_5drones.py - Quick telemetry heartbeat checker for 5 drones.
"""
import time
from pymavlink import mavutil

def check_heartbeats():
    print("Checking dedicated swarm control channels for all 5 drones...")
    drone_ports = [14552, 14562, 14572, 14582, 14592]
    drone_labels = [
        "Drone 1 (Apex Leader)",
        "Drone 2 (Left Inner)",
        "Drone 3 (Right Inner)",
        "Drone 4 (Left Outer)",
        "Drone 5 (Right Outer)"
    ]

    success = True
    for i, port in enumerate(drone_ports):
        label = drone_labels[i]
        endpoint = f"udpin:127.0.0.1:{port}"
        try:
            conn = mavutil.mavlink_connection(endpoint)
            msg = conn.wait_heartbeat(timeout=4.0)
            if msg:
                print(f"  ✓ {label} online! (SysID={conn.target_system}, Port={port})")
            else:
                print(f"  ✗ {label} TIMEOUT on {endpoint}")
                success = False
            conn.close()
        except Exception as e:
            print(f"  ✗ {label} Error on {endpoint}: {e}")
            success = False

    if success:
        print("\n>>> ALL 5 DRONES ARE FULLY ONLINE AND STREAMING MAVLINK! <<<")
    else:
        print("\n>>> WARNING: One or more drones failed to report. <<<")
    return success

if __name__ == "__main__":
    check_heartbeats()
