#!/usr/bin/env python3
"""
qgc_swarm_wingman_5drones.py - QGroundControl Swarm Follower Daemon for 5 Drones.
Allows the pilot to control Drone 1 (Apex Leader) entirely from QGroundControl
(using "Go to location", waypoint missions, or manual map clicks).
Drones 2, 3, 4, 5 autonomously track Drone 1 in real-time at 10 Hz,
holding a perfect 5-drone Flying-V formation wherever Drone 1 flies!
"""

import math
import sys
import time
import numpy as np
from pymavlink import mavutil

# Import CommonCoordinateFrame from swarm_drones_fyp
sys.path.insert(0, "/home/drone/swarm_drones_fyp")
from sitl.common_frame import CommonCoordinateFrame

class SwarmWingmanDaemon5Drones:
    def __init__(self):
        self.frame = CommonCoordinateFrame(-35.363261, 149.165230, 584.0)
        
        # Connections for all 5 drones
        self.connections = []
        self.drone_ports = [14552, 14562, 14572, 14582, 14592]
        self.drone_labels = [
            "Drone 1 (Apex Leader)",
            "Drone 2 (Left Inner)",
            "Drone 3 (Right Inner)",
            "Drone 4 (Left Outer)",
            "Drone 5 (Right Outer)"
        ]
        
        # Frame origins (Global NED - Local NED) for each drone
        self.origins = [np.zeros(3) for _ in range(5)]
        
        # Leader live state
        self.leader_pos = np.zeros(3)  # Global NED [North, East, Down]
        self.leader_yaw_rad = 0.0
        self.running = True

    def connect_all(self):
        print(">> [WINGMAN-5D] Connecting to all 5 drones...")
        self.connections.clear()
        for i, port in enumerate(self.drone_ports):
            label = self.drone_labels[i]
            endpoint = f"udpin:127.0.0.1:{port}"
            conn = mavutil.mavlink_connection(endpoint)
            msg = conn.wait_heartbeat(timeout=5.0)
            if not msg:
                raise RuntimeError(f"Timeout connecting to {label} on {endpoint}")
            print(f"  [OK] {label} connected (SysID={conn.target_system})")
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
            )
            self.connections.append(conn)

        # Calibrate frame origins
        print(">> [WINGMAN-5D] Synchronizing reference frames across all 5 drones...")
        for i, conn in enumerate(self.connections):
            self.origins[i] = self._get_origin(conn, self.drone_labels[i])
        print(">> [WINGMAN-5D] All frames synchronized! Followers 2, 3, 4, 5 tracking Drone 1.")

    def _get_origin(self, conn, label: str) -> np.ndarray:
        t0 = time.time()
        loc, glob = None, None
        while time.time() - t0 < 5.0 and not (loc and glob):
            m = conn.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"], blocking=False)
            if m:
                if m.get_type() == "LOCAL_POSITION_NED":
                    loc = m
                elif m.get_type() == "GLOBAL_POSITION_INT":
                    glob = m
            time.sleep(0.05)
            
        if loc and glob:
            g_ned = self.frame.gps_to_global_ned(glob.lat / 1e7, glob.lon / 1e7, glob.relative_alt / 1000.0)
            l_ned = np.array([loc.x, loc.y, loc.z])
            origin = g_ned - l_ned
            print(f"  [{label}] Origin: N={origin[0]:.2f}m, E={origin[1]:.2f}m")
            return origin
        return np.zeros(3)

    def run(self):
        print("\n=================================================================")
        print("  QGROUNDCONTROL 5-DRONE SWARM WINGMAN DAEMON ACTIVE!")
        print("  Right-click anywhere on the map in QGroundControl -> 'Go to location'")
        print("  Drones 2, 3, 4, 5 will autonomously shadow Drone 1 in V-formation!")
        print("=================================================================\n")
        
        type_mask = 0b0000101111111000  # Position + Yaw
        # Formation offsets relative to leader heading [Surge (back -), Sway (right +), Heave]
        follower_offsets = [
            np.array([-3.5, -3.0, 0.0]),  # D2: Left Inner
            np.array([-3.5,  3.0, 0.0]),  # D3: Right Inner
            np.array([-7.0, -6.0, 0.0]),  # D4: Left Outer
            np.array([-7.0,  6.0, 0.0]),  # D5: Right Outer
        ]

        conn1 = self.connections[0]
        followers = self.connections[1:]
        follower_origs = self.origins[1:]

        while self.running:
            # Drain telemetry from Drone 1 (Leader)
            while True:
                m = conn1.recv_match(blocking=False)
                if not m:
                    break
                mtype = m.get_type()
                if mtype == "GLOBAL_POSITION_INT":
                    self.leader_pos = self.frame.gps_to_global_ned(
                        m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0
                    )
                elif mtype == "ATTITUDE":
                    self.leader_yaw_rad = m.yaw

            # Ensure minimum safe altitude (hover at least 3.5m, match leader if higher)
            safe_alt_ned = min(-3.5, self.leader_pos[2])

            # Calculate body-to-world rotation based on leader heading
            psi = self.leader_yaw_rad
            cos_p = math.cos(psi)
            sin_p = math.sin(psi)

            # Update each follower
            for i, conn_f in enumerate(followers):
                off = follower_offsets[i]
                # Rotate offset into global frame
                target_world = np.array([
                    self.leader_pos[0] + (off[0] * cos_p - off[1] * sin_p),
                    self.leader_pos[1] + (off[0] * sin_p + off[1] * cos_p),
                    safe_alt_ned
                ])
                # Convert to follower's local NED frame
                target_local = target_world - follower_origs[i]
                conn_f.mav.set_position_target_local_ned_send(
                    0, conn_f.target_system, conn_f.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(target_local[0]), float(target_local[1]), float(target_local[2]),
                    0, 0, 0, 0, 0, 0, psi, 0
                )

            time.sleep(0.1)

if __name__ == "__main__":
    daemon = SwarmWingmanDaemon5Drones()
    daemon.connect_all()
    daemon.run()
