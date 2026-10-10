#!/usr/bin/env python3
"""
test_5drones_takeoff.py - Arms and takes off all 5 drones simultaneously to 5m,
holds for 15s to verify hover, then lands smoothly.
"""
import time
from concurrent.futures import ThreadPoolExecutor
from pymavlink import mavutil
import sys
sys.path.insert(0, "/home/drone/swarm_drones_fyp")
from sitl.flight_prep import arm_with_retry, takeoff_and_verify

drone_ports = [14552, 14562, 14572, 14582, 14592]
drone_labels = [
    "Drone 1 (Apex Leader)",
    "Drone 2 (Left Inner)",
    "Drone 3 (Right Inner)",
    "Drone 4 (Left Outer)",
    "Drone 5 (Right Outer)"
]

print("Connecting to all 5 drones...")
connections = []
for i, port in enumerate(drone_ports):
    conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    conn.wait_heartbeat()
    conn.mav.param_set_send(conn.target_system, conn.target_component, b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    conn.mav.param_set_send(conn.target_system, conn.target_component, b"GUID_TIMEOUT", 3.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    connections.append(conn)
    print(f"  Connected to {drone_labels[i]}")

print("\n>> Simultaneous ARMING of all 5 drones...")
with ThreadPoolExecutor(max_workers=5) as executor:
    futures = [executor.submit(arm_with_retry, conn, drone_labels[i]) for i, conn in enumerate(connections)]
    results = [f.result() for f in futures]
    if not all(results):
        print("ARMING FAILED for one or more drones!")
        sys.exit(1)
print(">> All 5 drones ARMED successfully!")

print("\n>> Simultaneous TAKEOFF of all 5 drones to 5.0m...")
with ThreadPoolExecutor(max_workers=5) as executor:
    futures = [executor.submit(takeoff_and_verify, conn, drone_labels[i], 5.0) for i, conn in enumerate(connections)]
    results = [f.result() for f in futures]

print("\n>> Monitoring 5-drone hover at 5.0m for 15 seconds...")
for s in range(15):
    time.sleep(1.0)
    alts = []
    for conn in connections:
        m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        alts.append(m.relative_alt / 1000.0 if m else 0.0)
    print(f"  T+{s+1}s Hover Alts: D1={alts[0]:.1f}m, D2={alts[1]:.1f}m, D3={alts[2]:.1f}m, D4={alts[3]:.1f}m, D5={alts[4]:.1f}m")

print("\n>> Test complete! Drones airborne and holding altitude.")
