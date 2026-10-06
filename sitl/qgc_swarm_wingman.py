#!/usr/bin/env python3
"""
qgc_swarm_wingman.py - QGroundControl Swarm Follower Daemon
Allows the user to pilot Drone 1 entirely from QGroundControl (using "Go to location",
missions, or manual clicks). Drones 2 & 3 autonomously track Drone 1 in real-time at 10 Hz,
holding a perfect Flying-V formation wherever Drone 1 flies!
"""

import math
import sys
import time
import numpy as np
from pymavlink import mavutil

# Import CommonCoordinateFrame from swarm_drones_fyp
sys.path.insert(0, "/home/drone/swarm_drones_fyp")
from sitl.common_frame import CommonCoordinateFrame

class SwarmWingmanDaemon:
    def __init__(self):
        self.frame = CommonCoordinateFrame(-35.363261, 149.165230, 584.0)
        
        # Connections
        self.conn1 = None  # Leader (Drone 1)
        self.conn2 = None  # Left Wing (Drone 2)
        self.conn3 = None  # Right Wing (Drone 3)
        
        # Frame origins (Global NED - Local NED)
        self.orig1 = np.zeros(3)
        self.orig2 = np.zeros(3)
        self.orig3 = np.zeros(3)
        
        # Leader live state
        self.leader_pos = np.zeros(3)  # Global NED [North, East, Down]
        self.leader_yaw_rad = 0.0
        self.running = True

    def connect_all(self):
        print(">> [WINGMAN] Connecting to all 3 drones...")
        self.conn1 = mavutil.mavlink_connection("udpin:127.0.0.1:14552")
        self.conn2 = mavutil.mavlink_connection("udpin:127.0.0.1:14562")
        self.conn3 = mavutil.mavlink_connection("udpin:127.0.0.1:14572")
        
        for c, name in [(self.conn1, "Drone 1 (Leader)"), (self.conn2, "Drone 2 (Left)"), (self.conn3, "Drone 3 (Right)")]:
            msg = c.wait_heartbeat(timeout=5.0)
            if not msg:
                raise RuntimeError(f"Timeout connecting to {name}")
            print(f"  [OK] {name} connected (SysID={c.target_system})")
            c.mav.request_data_stream_send(
                c.target_system, c.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
            )

        # Calibrate frame origins
        print(">> [WINGMAN] Synchronizing reference frames...")
        self.orig1 = self._get_origin(self.conn1, "Drone 1")
        self.orig2 = self._get_origin(self.conn2, "Drone 2")
        self.orig3 = self._get_origin(self.conn3, "Drone 3")
        print(">> [WINGMAN] All frames synchronized! Followers are tracking Drone 1.")

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
        print("  QGROUNDCONTROL SWARM WINGMAN DAEMON ACTIVE!")
        print("  Right-click anywhere on the map in QGroundControl -> 'Go to location'")
        print("  Drones 2 & 3 will autonomously shadow Drone 1 in V-formation!")
        print("=================================================================\n")
        
        type_mask = 0b0000101111111000  # Position + Yaw
        v_offset_left = np.array([-3.5, -3.0, 0.0])   # 3.5m back, 3.0m left
        v_offset_right = np.array([-3.5, +3.0, 0.0])  # 3.5m back, 3.0m right

        while self.running:
            # Drain telemetry from Drone 1
            while True:
                m = self.conn1.recv_match(blocking=False)
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

            # Rotate formation offsets to match leader's flight direction
            d2_world = np.array([
                self.leader_pos[0] + (v_offset_left[0] * cos_p - v_offset_left[1] * sin_p),
                self.leader_pos[1] + (v_offset_left[0] * sin_p + v_offset_left[1] * cos_p),
                safe_alt_ned
            ])
            d3_world = np.array([
                self.leader_pos[0] + (v_offset_right[0] * cos_p - v_offset_right[1] * sin_p),
                self.leader_pos[1] + (v_offset_right[0] * sin_p + v_offset_right[1] * cos_p),
                safe_alt_ned
            ])

            # Local NED for Drone 2
            d2_local = d2_world - self.orig2
            self.conn2.mav.set_position_target_local_ned_send(
                0, self.conn2.target_system, self.conn2.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                float(d2_local[0]), float(d2_local[1]), float(d2_local[2]),
                0, 0, 0, 0, 0, 0, psi, 0
            )

            # Local NED for Drone 3
            d3_local = d3_world - self.orig3
            self.conn3.mav.set_position_target_local_ned_send(
                0, self.conn3.target_system, self.conn3.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                float(d3_local[0]), float(d3_local[1]), float(d3_local[2]),
                0, 0, 0, 0, 0, 0, psi, 0
            )

            time.sleep(0.1)

if __name__ == "__main__":
    daemon = SwarmWingmanDaemon()
    daemon.connect_all()
    daemon.run()
