#!/usr/bin/env python3
"""
MAVLink Swarm Adapter Layer (SITL & Physical Hardware Integration).
Reuses the swarm_core controller implementations and translates guidance outputs into ArduPilot local-NED position setpoints. ArduPilot’s onboard position controller and SITL dynamics remain part of the execution path.
- Integrates Centralized, Decentralized, and Hybrid control modes with ArduPilot position setpoints
- Embeds WirelessChannel network emulator to apply packet loss & latency directly to SITL/hardware
- Uses CommonCoordinateFrame to anchor all drones to a shared WGS84 metric datum
- Supports 4 dynamic in-flight formation morphs: V-Shape, Line, Circle, Grid
"""

import sys
import os
import time
import math
import threading
from typing import Dict, List, Optional
import numpy as np
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

from swarm_core.config import get_profile
from swarm_core.drone import Drone
from swarm_core.formations import FormationGenerator, FormationType
from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.controllers.hybrid import HybridController, HybridMode
from swarm_core.network import WirelessChannel
from sitl.common_frame import CommonCoordinateFrame


DRONE_SPECS = [
    {"sysid": 1, "port": 14552, "label": "Drone 1 (Apex)"},
    {"sysid": 2, "port": 14562, "label": "Drone 2 (Left Wing)"},
    {"sysid": 3, "port": 14572, "label": "Drone 3 (Right Wing)"},
]


class MAVLinkDroneInterface:
    """Manages low-level MAVLink I/O and frame conversions for a single drone."""

    def __init__(self, sysid: int, port: int, label: str, frame: CommonCoordinateFrame):
        self.sysid = sysid
        self.port = port
        self.label = label
        self.frame = frame
        self.conn = None
        
        # State
        self.gps_pos = np.zeros(3)  # [lat, lon, rel_alt]
        self.global_ned = np.zeros(3)  # [North, East, Down] in shared datum frame
        self.local_ned = np.zeros(3)   # Vehicle's own local NED
        self.home_global_ned = None
        self.velocity = np.zeros(3)
        self.yaw_deg = 0.0
        self.mode = "DISCONN"
        self.is_armed = False
        self.connected = False

    def connect(self) -> bool:
        if mavutil is None:
            raise ImportError(
                f"pymavlink is required to connect to {self.label}, but is not installed. "
                "Install via: pip install -e '.[sitl]'"
            )
        if isinstance(self.port, str):
            endpoint = self.port
        elif self.port in [5760, 5770, 5780]:
            endpoint = f"tcp:127.0.0.1:{self.port}"
        else:
            endpoint = f"udpin:127.0.0.1:{self.port}"
        try:
            self.conn = mavutil.mavlink_connection(endpoint)
            msg = self.conn.wait_heartbeat(timeout=4.0)
            if msg:
                self.connected = True
                self.mode = mavutil.mode_string_v10(msg)
                self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                # Request telemetry streams
                self.conn.mav.request_data_stream_send(
                    self.conn.target_system, self.conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
                )
                return True
        except Exception as e:
            print(f"Error connecting to {self.label}: {e}")
        return False

    def poll_telemetry(self):
        if not self.conn:
            return
        while True:
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg) if mavutil is not None else "GUIDED"
                arm_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if (mavutil is not None and hasattr(mavutil, "mavlink")) else 128
                self.is_armed = bool(msg.base_mode & arm_flag)
            elif mtype == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                rel_alt = msg.relative_alt / 1000.0
                self.gps_pos = np.array([lat, lon, rel_alt])
                self.velocity = np.array([msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0])
                
                # Transform to Common Shared Metric Frame
                self.global_ned = self.frame.gps_to_global_ned(lat, lon, rel_alt)
                if self.home_global_ned is None:
                    # Anchor home in global datum coordinates
                    self.home_global_ned = self.global_ned.copy()
            elif mtype == "LOCAL_POSITION_NED":
                self.local_ned = np.array([msg.x, msg.y, msg.z])
            elif mtype == "ATTITUDE":
                self.yaw_deg = math.degrees(msg.yaw)

    def send_target_global(self, target_global_ned: np.ndarray):
        """Translates global NED target into drone's local NED frame and sends setpoint."""
        if not self.conn or self.home_global_ned is None:
            return
        # Calculate local target: P_local = P_global - P_home_global + P_home_local_offset
        # In ArduPilot, setpoint is relative to vehicle's EKF origin (which corresponds to its spawn point)
        local_target = self.frame.global_to_local_ned(target_global_ned, self.home_global_ned)
        
        type_mask = 0b0000111111111000  # Position setpoint only
        mav_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED if (mavutil is not None and hasattr(mavutil, 'mavlink')) else 1
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mav_frame,
            type_mask,
            local_target[0], local_target[1], local_target[2],
            0, 0, 0, 0, 0, 0, 0, 0
        )


class MAVLinkSwarmAdapter:
    """
    Directly couples the swarm_core controllers and network emulator with the SITL fleet.
    Enables unified multi-mode, multi-formation execution.
    """

    def __init__(self, control_mode: str = "hybrid", packet_loss: float = 0.0):
        self.frame = CommonCoordinateFrame()
        self.interfaces = [
            MAVLinkDroneInterface(s["sysid"], s["port"], s["label"], self.frame)
            for s in DRONE_SPECS
        ]
        
        self.profile = get_profile("sitl_fitted")

        # swarm_core Drones (2D mathematical state representations)
        self.core_drones = [
            Drone(drone_id=i, initial_position=np.zeros(2), profile="sitl_fitted")
            for i in range(len(self.interfaces))
        ]

        # Core controllers configured for SITL translation
        self.control_mode = control_mode
        self.central_ctrl = CentralizedController(
            kp=self.profile.get("centralized_kp"),
            kd=self.profile.get("centralized_kd"),
            k_repulse=self.profile.get("k_repulse"),
            collision_dist=self.profile.get("apf_activation_dist"),
        )
        self.decentral_ctrl = DecentralizedController(
            k_sep=self.profile.get("k_repulse"),
            k_align=self.profile.get("decentralized_kv"),
            k_form=self.profile.get("decentralized_k_form"),
            safe_radius=self.profile.get("apf_activation_dist"),
        )
        self.hybrid_ctrl = HybridController(
            degrade_timeout=self.profile.get("hybrid_degrade_timeout"),
            recovery_ratio_threshold=self.profile.get("hybrid_recovery_ratio"),
            min_dwell_time=self.profile.get("hybrid_dwell_time"),
            ramp_duration=self.profile.get("hybrid_ramp_duration"),
            window_size=self.profile.get("hybrid_recovery_window"),
            central_controller=self.central_ctrl,
            decentral_controller=self.decentral_ctrl,
        )
        self.channel = WirelessChannel(packet_loss_rate=packet_loss, latency_mean=0.03)
        self.hybrid_ctrl.set_nominal_latency(0.03)

        # Swarm Mission State
        self.current_formation = FormationType.V_SHAPE
        self.formation_spacing = 3.5
        self.centroid_target = np.array([0.0, 0.0, -5.05], dtype=np.float64)
        self.cruise_alt = 5.05
        
        self.running = True
        self.lock = threading.Lock()
        self.seq_num = 0

    def connect(self) -> bool:
        print("Connecting MAVLink Swarm Adapter to all SITL instances...")
        for iface in self.interfaces:
            if not iface.connect():
                print(f"Failed to connect to {iface.label}")
                return False
        
        # Warm up telemetry and home datum
        print("Synchronizing telemetry with Common Global Reference Frame...")
        for _ in range(15):
            for iface in self.interfaces:
                iface.poll_telemetry()
            time.sleep(0.05)

        # Initialize core_drones positions
        for i, iface in enumerate(self.interfaces):
            self.core_drones[i].position = iface.global_ned[:2].copy()
            self.core_drones[i].velocity = iface.velocity[:2].copy()

        print(f"Initialized! Swarm Datum Lat0={self.frame.lat0}, Lon0={self.frame.lon0}")
        for iface in self.interfaces:
            print(f"  {iface.label}: Global N={iface.global_ned[0]:.2f}m, E={iface.global_ned[1]:.2f}m, Alt={-iface.global_ned[2]:.2f}m")
        return True

    def set_formation(self, formation: FormationType):
        with self.lock:
            self.current_formation = formation
        print(f">> Formation morphed to: {formation.value.upper()}")

    def set_packet_loss(self, loss_rate: float):
        with self.lock:
            self.channel.packet_loss_rate = float(loss_rate)
        print(f">> Simulated channel packet loss set to: {loss_rate * 100:.0f}%")

    def run_control_cycle(self, current_time: float, dt: float = 0.1):
        """Executes one 10 Hz iteration of the exact swarm_core guidance pipeline."""
        # 1. Ingest telemetry and update core_drones
        for i, iface in enumerate(self.interfaces):
            iface.poll_telemetry()
            self.core_drones[i].position = iface.global_ned[:2].copy()
            self.core_drones[i].velocity = iface.velocity[:2].copy()

        n = len(self.core_drones)

        # 2. Compute Target Slots from Formation Generator
        local_offsets = FormationGenerator.get_formation_offsets(
            self.current_formation, n, spacing=self.formation_spacing
        )
        # 3D world slots
        world_slots_3d = np.zeros((n, 3))
        for i in range(n):
            world_slots_3d[i, 0] = self.centroid_target[0] + local_offsets[i, 0]
            world_slots_3d[i, 1] = self.centroid_target[1] + local_offsets[i, 1]
            world_slots_3d[i, 2] = -self.cruise_alt

        # 3. Simulate Central Heartbeat Broadcast via WirelessChannel
        self.seq_num += 1
        for i in range(n):
            payload = {
                "seq_num": self.seq_num,
                "target_pos": world_slots_3d[i, :2].copy(),
            }
            self.channel.send(
                sender_id=-1, recipient_id=i, payload=payload,
                distance=float(np.linalg.norm(self.core_drones[i].position - self.centroid_target[:2])),
                current_time=current_time
            )

        # 4. Receive and validate packets per drone
        for i in range(n):
            pkts = self.channel.receive(i, current_time)
            coord_pkt_received = False
            for pkt in pkts:
                if pkt.sender_id == -1:
                    coord_pkt_received = True
                    self.hybrid_ctrl.process_coordinator_heartbeat(
                        drone_id=i,
                        current_time=current_time,
                        send_timestamp=pkt.sent_time,
                        sequence_num=pkt.payload["seq_num"],
                        target_pos=pkt.payload["target_pos"],
                    )
            if not coord_pkt_received:
                self.hybrid_ctrl.record_heartbeat_attempt(i, False)

        # 5. Execute Selected Swarm Controller
        neighbor_states = [
            {"id": d.id, "position": d.position.copy(), "velocity": d.velocity.copy()}
            for d in self.core_drones
        ]

        target_setpoints_3d = []
        for i, d in enumerate(self.core_drones):
            if self.control_mode == "centralized":
                # Direct centralized slot assignment
                tgt = world_slots_3d[i].copy()
            elif self.control_mode == "decentralized":
                # Pure local neighbor consensus
                neighbors = [s for s in neighbor_states if s["id"] != d.id]
                u_dec = self.decentral_ctrl.compute_drone_control(
                    drone=d, neighbor_states=neighbors, goal_pos=world_slots_3d[i, :2]
                )
                # Integrate step for target setpoint
                tgt = np.array([d.position[0] + u_dec[0] * dt, d.position[1] + u_dec[1] * dt, -self.cruise_alt])
            else:
                # Proposed Hybrid Controller with smooth alpha blending and hysteresis
                neighbors = [s for s in neighbor_states if s["id"] != d.id]
                u_hyb = self.hybrid_ctrl.compute_hybrid_control(
                    drone=d,
                    current_time=current_time,
                    dt=dt,
                    neighbor_states=neighbors,
                    drag_coeff=float(self.profile.get("drag_coeff")),
                )
                tgt = np.array([d.position[0] + u_hyb[0] * dt * 2.0, d.position[1] + u_hyb[1] * dt * 2.0, -self.cruise_alt])

            target_setpoints_3d.append(tgt)

        # 6. Transmit target setpoints to MAVLink autopilots
        for i, iface in enumerate(self.interfaces):
            iface.send_target_global(target_setpoints_3d[i])

        return target_setpoints_3d

    def start_loop(self):
        t0 = time.time()
        while self.running:
            cur_time = time.time() - t0
            self.run_control_cycle(cur_time, dt=0.1)
            time.sleep(0.1)


def main():
    adapter = MAVLinkSwarmAdapter(control_mode="hybrid", packet_loss=0.0)
    if not adapter.connect():
        sys.exit(1)

    t = threading.Thread(target=adapter.start_loop, daemon=True)
    t.start()

    print("\n=================================================================")
    print("      MAVLINK SWARM ADAPTER: CORE ENGINE BRIDGED TO SITL         ")
    print("=================================================================")
    print(" Active Mode: HYBRID (swarm_core.controllers.hybrid.HybridController)")
    print(" Coordinates: Common Global Reference Frame (WGS84 Tangent Plane)")
    print(" Commands:")
    print("   [v] Switch to V-SHAPE Formation")
    print("   [l] Switch to LINE Formation")
    print("   [c] Switch to CIRCLE Formation")
    print("   [g] Switch to GRID Formation")
    print("   [0] Set Packet Loss:  0% (Healthy Link)")
    print("   [2] Set Packet Loss: 20% (Degraded Link)")
    print("   [5] Set Packet Loss: 50% (Severed Link -> Decentralized Fallback)")
    print("   [w/s/a/d] Translate Swarm Centroid (North/South/East/West)")
    print("   [q] Quit")
    print("=================================================================")

    try:
        while True:
            cmd = input("\nAction [v/l/c/g/0/2/5/w/s/a/d/q]: ").strip().lower()
            if cmd == "v":
                adapter.set_formation(FormationType.V_SHAPE)
            elif cmd == "l":
                adapter.set_formation(FormationType.LINE)
            elif cmd == "c":
                adapter.set_formation(FormationType.CIRCLE)
            elif cmd == "g":
                adapter.set_formation(FormationType.GRID)
            elif cmd == "0":
                adapter.set_packet_loss(0.0)
            elif cmd == "2":
                adapter.set_packet_loss(0.20)
            elif cmd == "5":
                adapter.set_packet_loss(0.50)
            elif cmd == "w":
                adapter.centroid_target[0] += 5.0
                print(f">> Centroid shifted North to: {adapter.centroid_target[0]:.1f}m")
            elif cmd == "s":
                adapter.centroid_target[0] -= 5.0
                print(f">> Centroid shifted South to: {adapter.centroid_target[0]:.1f}m")
            elif cmd == "a":
                adapter.centroid_target[1] -= 5.0
                print(f">> Centroid shifted West to: {adapter.centroid_target[1]:.1f}m")
            elif cmd == "d":
                adapter.centroid_target[1] += 5.0
                print(f">> Centroid shifted East to: {adapter.centroid_target[1]:.1f}m")
            elif cmd == "q":
                adapter.running = False
                break
    except KeyboardInterrupt:
        adapter.running = False
        print("\nAdapter stopped.")


if __name__ == "__main__":
    main()
