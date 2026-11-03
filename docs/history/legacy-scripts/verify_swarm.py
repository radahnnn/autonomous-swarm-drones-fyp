#!/usr/bin/env python3
import time
import socket
from pymavlink import mavutil

def check_heartbeats():
    print("Connecting to UDP 14550 to check all 3 swarm vehicles...")
    try:
        conn = mavutil.mavlink_connection('udpin:127.0.0.1:14550', timeout=8)
    except Exception as e:
        print(f"Error binding to UDP 14550: {e}")
        return False

    seen_sysids = set()
    start_time = time.time()
    
    while time.time() - start_time < 12 and len(seen_sysids) < 3:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if msg:
            sysid = msg.get_srcSystem()
            if sysid in [1, 2, 3] and sysid not in seen_sysids:
                seen_sysids.add(sysid)
                role = {1: "Drone 1 (Apex Leader)", 2: "Drone 2 (Left Wing)", 3: "Drone 3 (Right Wing)"}.get(sysid, f"Drone {sysid}")
                print(f"  ✓ Found {role} (SysID={sysid}, Type={msg.type}) streaming on UDP 14550")

    conn.close()

    if len(seen_sysids) == 3:
        print("\n>>> ALL 3 DRONES ARE FULLY ONLINE AND STREAMING MAVLINK! <<<")
        return True
    else:
        print(f"\nOnly found {len(seen_sysids)}/3 drones: {seen_sysids}")
        return False

if __name__ == "__main__":
    check_heartbeats()
