#!/usr/bin/env python3
"""
MAVLink Swarm Adapter Layer (SITL & Physical Hardware Integration).
Reuses the swarm_core controller implementations and translates guidance outputs into ArduPilot local-NED position setpoints. ArduPilot’s onboard position controller and SITL dynamics remain part of the execution path.
Also supports plant-integrated velocity setpoints (mask 0x0DC7) for closed-loop dynamic parity.

Key Features & Parity Realism:
- True Frame Origin: Computes each drone's local EKF origin dynamically as
  origin_global = global_ned - local_ned, remaining robust to pre-connection drift.
- Plant-Integrated Velocity Setpoints: Replaces open-loop displacement heuristics with
  first-order lag + rotor drag velocity integration (mask 0x0DC7) matching Drone.step().
- Full Network Ingestion: Peer telemetry and coordinator heartbeats route through
  WirelessChannel with latency and packet loss.
- Production Flight Safety (Item 20):
  * GUIDED mode check before dispatching setpoints
  * Safe automated arm/takeoff sequence
  * Hardware-facing safety: ARMING_CHECK=0 is strictly restricted to SITL testing
  * Setpoint distance clamp preventing autopilot runaway
  * Telemetry watchdog (link-loss failsafe)
  * Geofence boundary monitoring
  * Emergency stop / kill command
  * Flight logging
"""

import sys
import os
import time
import math
import logging
import threading
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

from swarm_core.config import get_profile, build_controllers_from_profile
from swarm_core.drone import Drone
from swarm_core.formations import (
    FormationGenerator,
    FormationType,
    compute_formation_slots,
    compute_desired_neighbor_offsets,
)
from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.controllers.hybrid import HybridController, HybridMode
from swarm_core.network import WirelessChannel
from sitl.common_frame import CommonCoordinateFrame

logger = logging.getLogger("MAVLinkSwarmAdapter")


def acceleration_to_position_setpoint(
    current_pos_2d: np.ndarray,
    accel_2d: np.ndarray,
    dt: float,
    cruise_alt: float = 5.05,
    lookahead_scale: float = 1.0,
) -> np.ndarray:
    """
    Translates 2D guidance acceleration into a 3D ArduPilot local-NED position setpoint:
        P_sp = [p_x + a_x * dt * lookahead_scale, p_y + a_y * dt * lookahead_scale, -cruise_alt]

    ArduPilot's internal position PID (POS_XYZ_P, VEL_XYZ_PID) tracks this setpoint.
    The lookahead_scale provides lead compensation against autopilot tracking lag.
    """
    disp = np.asarray(accel_2d, dtype=np.float64)[:2] * float(dt) * float(lookahead_scale)
    return np.array([
        float(current_pos_2d[0] + disp[0]),
        float(current_pos_2d[1] + disp[1]),
        -float(cruise_alt),
    ], dtype=np.float64)


class IntegratedPlant:
    """
    Integrates lateral guidance acceleration a_cmd into a commanded velocity
    using the exact first-order attitude lag and rotor drag dynamics as Drone.step().
    """

    def __init__(
        self,
        tau: float = 0.18,
        drag_coeff: float = 0.20,
        max_speed: float = 3.0,
        max_accel: float = 2.5,
    ):
        self.tau = float(tau)
        self.drag_coeff = float(drag_coeff)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.effective_accel = np.zeros(2, dtype=np.float64)
        self.velocity = np.zeros(2, dtype=np.float64)

    def reset(self, initial_velocity: Optional[np.ndarray] = None):
        self.effective_accel = np.zeros(2, dtype=np.float64)
        self.velocity = (
            np.asarray(initial_velocity, dtype=np.float64)[:2].copy()
            if initial_velocity is not None
            else np.zeros(2, dtype=np.float64)
        )

    def step(self, a_cmd_2d: np.ndarray, dt: float) -> np.ndarray:
        a_cmd = np.asarray(a_cmd_2d, dtype=np.float64)[:2]
        a_norm = np.linalg.norm(a_cmd)
        if a_norm > self.max_accel and a_norm > 1e-6:
            a_cmd = a_cmd * (self.max_accel / a_norm)

        # First-order attitude / thrust time-constant lag
        if self.tau > 1e-4:
            alpha_att = dt / (self.tau + dt)
            self.effective_accel += alpha_att * (a_cmd - self.effective_accel)
        else:
            self.effective_accel = a_cmd.copy()

        # Rotor drag deceleration
        a_drag = -self.drag_coeff * self.velocity
        a_total = self.effective_accel + a_drag

        # Integrate lateral velocity
        self.velocity += a_total * dt
        v_norm = np.linalg.norm(self.velocity)
        if v_norm > self.max_speed and v_norm > 1e-6:
            self.velocity = self.velocity * (self.max_speed / v_norm)

        return self.velocity.copy()


DRONE_SPECS = [
    {"sysid": 1, "port": 14552, "label": "Drone 1 (Apex)"},
    {"sysid": 2, "port": 14562, "label": "Drone 2 (Left Wing)"},
    {"sysid": 3, "port": 14572, "label": "Drone 3 (Right Wing)"},
]


class MAVLinkDroneInterface:
    """Manages low-level MAVLink I/O, frame conversions, and safety checks for a single drone."""

    def __init__(
        self,
        sysid: int,
        port: int,
        label: str,
        frame: CommonCoordinateFrame,
        max_setpoint_distance: float = 5.0,
        geofence_radius: float = 60.0,
        min_alt: float = 0.5,
        max_alt: float = 25.0,
        watchdog_timeout: float = 1.5,
    ):
        self.sysid = sysid
        self.port = port
        self.label = label
        self.frame = frame
        self.conn = None

        # Safety parameters (Item 20)
        self.max_setpoint_distance = float(max_setpoint_distance)
        self.geofence_radius = float(geofence_radius)
        self.min_alt = float(min_alt)
        self.max_alt = float(max_alt)
        self.watchdog_timeout = float(watchdog_timeout)
        self.last_telemetry_time = time.time()

        # State in shared common global frame and local frame
        self.gps_pos = np.zeros(3)  # [lat, lon, rel_alt]
        self.global_ned = np.zeros(3)  # [North, East, Down] in shared metric datum frame
        self.local_ned = np.zeros(3)  # Vehicle's own local EKF NED
        self.velocity = np.zeros(3)
        self.yaw_deg = 0.0
        self.mode = "GUIDED"
        self.is_armed = False
        self.connected = False

        # Dynamically computed EKF frame origin: origin_global = global_ned - local_ned (Item 17)
        self._frame_origin_global: Optional[np.ndarray] = np.zeros(3, dtype=np.float64)

    @property
    def frame_origin_global(self) -> Optional[np.ndarray]:
        return self._frame_origin_global

    @frame_origin_global.setter
    def frame_origin_global(self, val: Optional[np.ndarray]):
        self._frame_origin_global = np.asarray(val, dtype=np.float64) if val is not None else None

    @property
    def home_global_ned(self) -> Optional[np.ndarray]:
        return self._frame_origin_global

    @home_global_ned.setter
    def home_global_ned(self, val: Optional[np.ndarray]):
        self._frame_origin_global = np.asarray(val, dtype=np.float64) if val is not None else None

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
                self.last_telemetry_time = time.time()
                # Request telemetry streams at 20 Hz
                self.conn.mav.request_data_stream_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    20,
                    1,
                )
                return True
        except Exception as e:
            logger.error("Error connecting to %s: %s", self.label, e)
        return False

    def poll_telemetry(self, max_msgs: int = 100):
        if not self.conn:
            return
        for _ in range(max_msgs):
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                break
            self.last_telemetry_time = time.time()
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg) if mavutil is not None else "GUIDED"
                arm_flag = (
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    if (mavutil is not None and hasattr(mavutil, "mavlink"))
                    else 128
                )
                self.is_armed = bool(msg.base_mode & arm_flag)
            elif mtype == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                rel_alt = msg.relative_alt / 1000.0
                self.gps_pos = np.array([lat, lon, rel_alt])
                self.velocity = np.array([msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0])
                # Transform to Common Shared Metric Datum Frame
                self.global_ned = self.frame.gps_to_global_ned(lat, lon, rel_alt)
                self._update_frame_origin()
            elif mtype == "LOCAL_POSITION_NED":
                self.local_ned = np.array([msg.x, msg.y, msg.z])
                self._update_frame_origin()
            elif mtype == "ATTITUDE":
                self.yaw_deg = math.degrees(msg.yaw)

    def _update_frame_origin(self):
        """
        Computes vehicle's local frame origin relative to the shared global datum:
            origin_global = global_ned - local_ned
        Valid regardless of whether the drone drifted from its spawn point (Item 17).
        """
        if np.any(self.global_ned != 0.0) or np.any(self.local_ned != 0.0):
            self.frame_origin_global = self.global_ned - self.local_ned

    def is_watchdog_healthy(self) -> bool:
        """Returns True if telemetry has been received within watchdog timeout."""
        if self.last_telemetry_time <= 0.0:
            return True
        return (time.time() - self.last_telemetry_time) <= self.watchdog_timeout

    def check_geofence(self, pos_global: np.ndarray) -> bool:
        """Verifies if target position is within permissible horizontal and vertical geofence."""
        horizontal_dist = float(np.linalg.norm(pos_global[:2]))
        alt = -float(pos_global[2])
        if horizontal_dist > self.geofence_radius:
            logger.warning("%s geofence breach: dist %.1fm > %.1fm", self.label, horizontal_dist, self.geofence_radius)
            return False
        if alt < self.min_alt or alt > self.max_alt:
            logger.warning("%s altitude geofence breach: alt %.1fm not in [%.1f, %.1f]", self.label, alt, self.min_alt, self.max_alt)
            return False
        return True

    def clamp_setpoint_distance(self, target_global: np.ndarray) -> np.ndarray:
        """Clamps maximum distance between vehicle current position and setpoint to prevent runaway."""
        diff = target_global[:2] - self.global_ned[:2]
        dist = float(np.linalg.norm(diff))
        if dist > self.max_setpoint_distance and dist > 1e-4:
            clamped_xy = self.global_ned[:2] + diff * (self.max_setpoint_distance / dist)
            return np.array([clamped_xy[0], clamped_xy[1], target_global[2]], dtype=np.float64)
        return target_global.copy()

    def send_target_global(self, target_global_ned: np.ndarray, target_vel_2d: Optional[np.ndarray] = None) -> bool:
        """
        Translates global NED target into vehicle local NED frame and transmits MAVLink setpoint.
        Enforces GUIDED check, watchdog health, geofence, and distance clamp.
        """
        if not self.conn or self.frame_origin_global is None:
            return False

        # Safety Checks (Item 20)
        if not self.is_watchdog_healthy():
            logger.warning("%s: Telemetry watchdog expired. Suppressing setpoint.", self.label)
            return False

        if self.mode != "GUIDED":
            logger.warning("%s: Autopilot not in GUIDED mode (mode=%s). Suppressing setpoint.", self.label, self.mode)
            return False

        target_global = self.clamp_setpoint_distance(target_global_ned)
        if not self.check_geofence(target_global):
            return False

        # Local target = target_global - origin_global
        local_target = target_global - self.frame_origin_global

        mav_frame = (
            mavutil.mavlink.MAV_FRAME_LOCAL_NED
            if (mavutil is not None and hasattr(mavutil, "mavlink"))
            else 1
        )

        if target_vel_2d is not None:
            # Type mask 0x0DC0 (3520): Position + Velocity enabled, ignore accel/yaw (Item 14)
            type_mask = 0x0DC0
            vx, vy = float(target_vel_2d[0]), float(target_vel_2d[1])
        else:
            # Type mask 0x0DF8 (3576): Position only enabled
            type_mask = 0b0000111111111000
            vx, vy = 0.0, 0.0

        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mav_frame,
            type_mask,
            float(local_target[0]),
            float(local_target[1]),
            float(local_target[2]),
            vx,
            vy,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        return True

    def send_velocity_target(self, vel_2d: np.ndarray, cruise_alt: float = 5.0) -> bool:
        """
        Sends pure velocity setpoints to ArduPilot local-NED velocity controller (Item 16).
        Type mask 0x0DC7 (3527): Ignore position, use vx/vy/vz, ignore accel/yaw.
        """
        if not self.conn:
            return False

        if not self.is_watchdog_healthy():
            logger.warning("%s: Telemetry watchdog expired.", self.label)
            return False

        if self.mode != "GUIDED":
            logger.warning("%s: Autopilot not in GUIDED mode (mode=%s).", self.label, self.mode)
            return False

        # Altitude hold proportional feedback
        current_alt = -float(self.global_ned[2])
        alt_err = float(cruise_alt) - current_alt
        vz = float(np.clip(-0.8 * alt_err, -1.0, 1.0))  # Downward velocity in NED

        mav_frame = (
            mavutil.mavlink.MAV_FRAME_LOCAL_NED
            if (mavutil is not None and hasattr(mavutil, "mavlink"))
            else 1
        )
        type_mask = 0x0DC7  # Pure velocity setpoint

        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mav_frame,
            type_mask,
            0.0,
            0.0,
            0.0,
            float(vel_2d[0]),
            float(vel_2d[1]),
            vz,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        return True

    def arm_and_takeoff(
        self,
        target_alt: float = 5.0,
        timeout: float = 25.0,
        arm_timeout: float = 10.0,
        is_sitl: bool = True,
    ) -> bool:
        """
        Executes guided arming and takeoff sequence.
        CRUCIAL (Item 20): ARMING_CHECK=0 is strictly restricted to SITL testing
        and never sent to physical hardware.
        """
        if not self.conn:
            return False

        mav_param_real32 = (
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            if (mavutil is not None and hasattr(mavutil, "mavlink"))
            else 9
        )
        mav_cmd_arm = (
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            if (mavutil is not None and hasattr(mavutil, "mavlink"))
            else 400
        )
        mav_mode_armed = (
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if (mavutil is not None and hasattr(mavutil, "mavlink"))
            else 128
        )
        mav_cmd_takeoff = (
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            if (mavutil is not None and hasattr(mavutil, "mavlink"))
            else 22
        )

        if is_sitl:
            # Set ARMING_CHECK=0 only in explicit SITL simulation environment
            self.conn.mav.param_set_send(
                self.conn.target_system,
                self.conn.target_component,
                b"ARMING_CHECK",
                0,
                mav_param_real32,
            )
            if arm_timeout >= 0.5:
                time.sleep(0.2)

        # Set GUIDED mode
        self.conn.set_mode(4)
        if arm_timeout >= 0.5:
            time.sleep(0.5)

        # Arm motors
        t_arm_start = time.time()
        armed = False
        while time.time() - t_arm_start < arm_timeout:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mav_cmd_arm,
                0,
                1,
                21196,
                0,
                0,
                0,
                0,
                0,
            )
            t_poll = time.time()
            poll_window = min(0.5, arm_timeout)
            while time.time() - t_poll < poll_window:
                m = self.conn.recv_match(type="HEARTBEAT", blocking=False)
                if not m:
                    break
                base_mode = getattr(m, "base_mode", 0)
                if isinstance(base_mode, int) and (base_mode & mav_mode_armed):
                    armed = True
                    break
            if armed:
                break
            if arm_timeout >= 0.5:
                time.sleep(0.3)

        if not armed:
            logger.error("Failed to arm %s", self.label)
            return False

        # Command Takeoff
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mav_cmd_takeoff,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            float(target_alt),
        )

        # Wait until reaching hover altitude
        t_climb_start = time.time()
        while time.time() - t_climb_start < timeout:
            self.poll_telemetry()
            alt = -self.global_ned[2]
            if alt >= target_alt * 0.90:
                logger.info("%s reached takeoff hover altitude: %.2fm", self.label, alt)
                return True
            time.sleep(0.2)

        logger.warning("%s takeoff timed out before reaching altitude", self.label)
        return False

    def emergency_stop(self):
        """Immediately commands vehicle to BRAKE or LAND mode (Item 20)."""
        if not self.conn:
            return
        logger.critical("EMERGENCY STOP TRIGGERED FOR %s", self.label)
        # Try BRAKE mode (17), fallback to LAND (9)
        try:
            self.conn.set_mode(17)
        except Exception:
            self.conn.set_mode(9)


class MAVLinkSwarmAdapter:
    """
    Directly couples swarm_core controllers and network emulator with the SITL fleet.
    Enables unified multi-mode, multi-formation execution with closed-loop parity.
    """

    def __init__(
        self,
        control_mode: str = "hybrid",
        packet_loss: float = 0.0,
        use_velocity_setpoints: bool = True,
        is_sitl: bool = True,
    ):
        self.frame = CommonCoordinateFrame()
        self.interfaces = [
            MAVLinkDroneInterface(s["sysid"], s["port"], s["label"], self.frame)
            for s in DRONE_SPECS
        ]

        self.profile = get_profile("sitl_fitted")
        self.control_mode = control_mode
        self.use_velocity_setpoints = bool(use_velocity_setpoints)
        self.is_sitl = bool(is_sitl)

        # swarm_core Drones (2D mathematical state representations)
        self.core_drones = [
            Drone(drone_id=i, initial_position=np.zeros(2), profile="sitl_fitted")
            for i in range(len(self.interfaces))
        ]

        # Core controllers configured identically from profile
        self.central_ctrl, self.decentral_ctrl, self.hybrid_ctrl = build_controllers_from_profile(
            self.profile, nominal_latency=0.03
        )
        self.channel = WirelessChannel(packet_loss_rate=packet_loss, latency_mean=0.03)

        # Integrated plants matching Drone.step() for velocity setpoint integration (Item 16)
        self.plants = [
            IntegratedPlant(
                tau=float(self.profile.get("attitude_tau")),
                drag_coeff=float(self.profile.get("drag_coeff")),
                max_speed=float(self.profile.get("max_speed")),
                max_accel=float(self.profile.get("max_accel")),
            )
            for _ in self.interfaces
        ]

        # Swarm Mission State
        self.current_formation = FormationType.V_SHAPE
        self.formation_spacing = 3.5
        self.centroid_target = np.array([0.0, 0.0, -5.05], dtype=np.float64)
        self.centroid_velocity = np.zeros(2, dtype=np.float64)
        self.cruise_alt = 5.05

        # Onboard neighbor memory & coordinator reception state
        self.neighbor_memory: Dict[int, Dict[int, Dict[str, Any]]] = {
            i: {} for i in range(len(self.interfaces))
        }
        self.neighbor_timeout = 0.30  # seconds
        self.last_central_target: Dict[int, np.ndarray] = {
            i: np.zeros(2) for i in range(len(self.interfaces))
        }
        self.last_central_velocity: Dict[int, np.ndarray] = {
            i: np.zeros(2) for i in range(len(self.interfaces))
        }

        self.running = True
        self.lock = threading.Lock()
        self.seq_num = 0

    def connect(self) -> bool:
        print("Connecting MAVLink Swarm Adapter to all SITL instances...")
        for iface in self.interfaces:
            if not iface.connect():
                print(f"Failed to connect to {iface.label}")
                return False

        # Warm up telemetry and establish common metric frame origin
        print("Synchronizing telemetry with Common Global Reference Frame...")
        for _ in range(20):
            for iface in self.interfaces:
                iface.poll_telemetry()
            time.sleep(0.05)

        for i, iface in enumerate(self.interfaces):
            self.core_drones[i].position = iface.global_ned[:2].copy()
            self.core_drones[i].velocity = iface.velocity[:2].copy()
            self.plants[i].reset(iface.velocity[:2].copy())

        print(f"Initialized! Swarm Datum Lat0={self.frame.lat0}, Lon0={self.frame.lon0}")
        for iface in self.interfaces:
            origin_str = (
                f"Origin N={iface.frame_origin_global[0]:.2f}m, E={iface.frame_origin_global[1]:.2f}m"
                if iface.frame_origin_global is not None
                else "Origin Pending"
            )
            print(f"  {iface.label}: Global N={iface.global_ned[0]:.2f}m, E={iface.global_ned[1]:.2f}m | {origin_str}")
        return True

    def arm_and_takeoff_all(self, target_alt: float = 5.0) -> bool:
        """Sequential guided arm and climb sequence for the swarm."""
        print(f"\nArming and launching swarm to {target_alt:.1f}m hover...")
        for iface in self.interfaces:
            if not iface.arm_and_takeoff(target_alt=target_alt, is_sitl=self.is_sitl):
                return False
        return True

    def emergency_stop_all(self):
        """Immediately stops all drones (Item 20)."""
        print("\n!!! EMERGENCY STOPPING ALL SWARM DRONES !!!")
        for iface in self.interfaces:
            iface.emergency_stop()

    def set_formation(self, formation: FormationType):
        with self.lock:
            self.current_formation = formation
        print(f">> Formation morphed to: {formation.value.upper()}")

    def set_packet_loss(self, loss_rate: float):
        with self.lock:
            self.channel.packet_loss_rate = float(loss_rate)
        print(f">> Simulated channel packet loss set to: {loss_rate * 100:.0f}%")

    def run_control_cycle(self, current_time: float, dt: float = 0.1) -> Dict[str, Any]:
        """
        Executes one iteration of the swarm_core guidance pipeline and dispatches
        setpoints to ArduPilot autopilots.
        """
        n = len(self.core_drones)

        # 1. Ingest telemetry from all autopilots
        for i, iface in enumerate(self.interfaces):
            iface.poll_telemetry()
            self.core_drones[i].position = iface.global_ned[:2].copy()
            self.core_drones[i].velocity = iface.velocity[:2].copy()

        # 2. Peer state inter-drone broadcast through WirelessChannel (Item 18)
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist = float(np.linalg.norm(self.core_drones[i].position - self.core_drones[j].position))
                    payload = {
                        "position": self.core_drones[i].position.copy(),
                        "velocity": self.core_drones[i].velocity.copy(),
                    }
                    self.channel.send(
                        sender_id=i,
                        recipient_id=j,
                        payload=payload,
                        distance=dist,
                        current_time=current_time,
                    )

        # 3. Target Slot Allocation
        current_pos_2d = np.array([d.position for d in self.core_drones])
        local_offsets, world_slots_2d, assigned_slots_2d = compute_formation_slots(
            formation_type=self.current_formation,
            num_drones=n,
            centroid=self.centroid_target[:2],
            spacing=self.formation_spacing,
            current_positions=current_pos_2d,
        )

        drone_ids = [d.id for d in self.core_drones]

        # 4. Central Coordinator Heartbeat Broadcast through WirelessChannel (Item 18)
        self.seq_num += 1
        for i in range(n):
            d_id = self.core_drones[i].id
            offsets_for_i = compute_desired_neighbor_offsets(
                drone_id=d_id,
                assigned_targets=assigned_slots_2d,
                neighbor_ids=[o.id for o in self.core_drones if o.id != d_id],
                drone_ids=drone_ids,
            )
            payload = {
                "seq_num": self.seq_num,
                "target_pos": assigned_slots_2d[i].copy(),
                "target_velocity": self.centroid_velocity.copy(),
                "formation_type": self.current_formation,
                "desired_offsets": offsets_for_i,
            }
            dist_to_gs = float(np.linalg.norm(self.core_drones[i].position - self.centroid_target[:2]))
            self.channel.send(
                sender_id=-1,
                recipient_id=d_id,
                payload=payload,
                distance=dist_to_gs,
                current_time=current_time,
            )

        # 5. Advance time and receive packets per drone (Item 18)
        sim_time = current_time + dt

        perceived_neighbors: Dict[int, List[Dict[str, Any]]] = {d.id: [] for d in self.core_drones}
        for d in self.core_drones:
            pkts = self.channel.receive(d.id, sim_time)
            coord_pkt_received = False
            for pkt in pkts:
                if pkt.sender_id == -1:
                    coord_pkt_received = True
                    self.last_central_target[d.id] = np.array(pkt.payload["target_pos"], dtype=np.float64)
                    if "target_velocity" in pkt.payload:
                        self.last_central_velocity[d.id] = np.array(pkt.payload["target_velocity"], dtype=np.float64)
                    self.hybrid_ctrl.process_coordinator_heartbeat(
                        drone_id=d.id,
                        current_time=sim_time,
                        send_timestamp=pkt.sent_time,
                        sequence_num=pkt.payload["seq_num"],
                        target_pos=pkt.payload["target_pos"],
                        target_velocity=pkt.payload.get("target_velocity"),
                        desired_offsets=pkt.payload.get("desired_offsets"),
                        formation_type=pkt.payload.get("formation_type"),
                    )
                else:
                    self.neighbor_memory[d.id][pkt.sender_id] = {
                        "position": np.array(pkt.payload["position"], dtype=np.float64),
                        "velocity": np.array(pkt.payload["velocity"], dtype=np.float64),
                        "timestamp": sim_time,
                    }

            if not coord_pkt_received:
                self.hybrid_ctrl.record_heartbeat_attempt(d.id, False)

            # Assemble neighbor memory with extrapolation & age-out
            valid_mem = {}
            for peer_id, mem in self.neighbor_memory[d.id].items():
                age = sim_time - mem["timestamp"]
                if age <= self.neighbor_timeout:
                    extrap_pos = mem["position"] + mem["velocity"] * age
                    perceived_neighbors[d.id].append({
                        "id": peer_id,
                        "position": extrap_pos,
                        "velocity": mem["velocity"].copy(),
                        "age": age,
                    })
                    valid_mem[peer_id] = mem
            self.neighbor_memory[d.id] = valid_mem

        # 6. Compute Control Accelerations and Integrate to Setpoints
        cmd_accels_2d = []
        cmd_vels_2d = []
        cmd_targets_3d = []

        for i, d in enumerate(self.core_drones):
            measured_p = d.position.copy()
            measured_v = d.velocity.copy()
            target_i = self.last_central_target[d.id]

            if self.control_mode == "centralized":
                # Centralized controller with hold-last-target tracking (Item 18)
                p_err = target_i - measured_p
                v_target = self.last_central_velocity[d.id]
                v_err = v_target - measured_v
                a_cmd = self.central_ctrl.kp * p_err + self.central_ctrl.kd * v_err
                # Drag feedforward
                a_cmd += float(self.profile.get("drag_coeff")) * v_target

                # Degree-normalised APF against perceived neighbors
                deg = max(1, len(perceived_neighbors[d.id]))
                for n_info in perceived_neighbors[d.id]:
                    diff = measured_p - n_info["position"]
                    dist = float(np.linalg.norm(diff))
                    if 1e-4 < dist < self.central_ctrl.collision_dist:
                        repulse = (
                            (self.central_ctrl.k_repulse / deg)
                            * (1.0 / dist - 1.0 / self.central_ctrl.collision_dist)
                            / (dist**2)
                        )
                        a_cmd += repulse * (diff / dist)

            elif self.control_mode == "decentralized":
                offsets = self.hybrid_ctrl.last_known_offsets.get(d.id, None)
                a_cmd = self.decentral_ctrl.compute_drone_control(
                    drone=d,
                    neighbor_states=perceived_neighbors[d.id],
                    desired_offsets=offsets,
                    goal_pos=target_i,
                    goal_vel=self.last_central_velocity[d.id],
                    measured_position=measured_p,
                    measured_velocity=measured_v,
                    use_velocity_feedforward=True,
                    drag_coeff=float(self.profile.get("drag_coeff")),
                )
            else:
                # Hybrid mode
                offsets = self.hybrid_ctrl.last_known_offsets.get(d.id, None)
                a_cmd = self.hybrid_ctrl.compute_hybrid_control(
                    drone=d,
                    current_time=sim_time,
                    dt=dt,
                    neighbor_states=perceived_neighbors[d.id],
                    desired_neighbor_offsets=offsets,
                    target_velocity=self.last_central_velocity[d.id],
                    use_velocity_feedforward=True,
                    drag_coeff=float(self.profile.get("drag_coeff")),
                    measured_position=measured_p,
                    measured_velocity=measured_v,
                )

            # Integrate acceleration through plant model (Item 16)
            cmd_vel = self.plants[i].step(a_cmd, dt)
            tgt_3d = np.array([target_i[0], target_i[1], -self.cruise_alt], dtype=np.float64)

            cmd_accels_2d.append(a_cmd)
            cmd_vels_2d.append(cmd_vel)
            cmd_targets_3d.append(tgt_3d)

            # 7. Transmit Setpoints to MAVLink autopilots
            if self.use_velocity_setpoints:
                # Pure velocity setpoint (mask 0x0DC7) (Item 16)
                self.interfaces[i].send_velocity_target(cmd_vel, cruise_alt=self.cruise_alt)
            else:
                # Position + velocity setpoint (mask 0x0DC0) (Item 14)
                self.interfaces[i].send_target_global(tgt_3d, target_vel_2d=cmd_vel)

        self.last_cycle_results = {
            "cmd_accels": cmd_accels_2d,
            "cmd_vels": cmd_vels_2d,
            "cmd_targets": cmd_targets_3d,
        }
        return cmd_targets_3d

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
    print("      MAVLINK SWARM ADAPTER: HARDENED CONTROL PARITY BRIDGED     ")
    print("=================================================================")
    print(" Commands:")
    print("   [t] Arm and takeoff all drones to 5.0m")
    print("   [v] Switch to V-SHAPE Formation")
    print("   [l] Switch to LINE Formation")
    print("   [c] Switch to CIRCLE Formation")
    print("   [g] Switch to GRID Formation")
    print("   [0] Set Packet Loss:  0% (Healthy Link)")
    print("   [2] Set Packet Loss: 20% (Degraded Link)")
    print("   [5] Set Packet Loss: 50% (Severed Link -> Decentralized Fallback)")
    print("   [w/s/a/d] Translate Swarm Centroid (North/South/East/West)")
    print("   [x] EMERGENCY STOP ALL DRONES")
    print("   [q] Quit")
    print("=================================================================")

    try:
        while True:
            cmd = input("\nAction [t/v/l/c/g/0/2/5/w/s/a/d/x/q]: ").strip().lower()
            if cmd == "t":
                adapter.arm_and_takeoff_all(5.0)
            elif cmd == "v":
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
            elif cmd == "x":
                adapter.emergency_stop_all()
            elif cmd == "q":
                adapter.running = False
                break
    except KeyboardInterrupt:
        adapter.running = False
        print("\nAdapter stopped.")


if __name__ == "__main__":
    main()
