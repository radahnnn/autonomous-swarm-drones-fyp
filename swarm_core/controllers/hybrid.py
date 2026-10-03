"""
Hybrid Swarm Controller with Asymmetric Hysteresis, Sequence Tracking, and Smooth Control Blending.

Features:
1. Message Age & Sequence Tracking: Commands are validated by timestamp age and strictly increasing sequence numbers.
2. Stale Command Rejection: Packets arriving with age exceeding latency threshold are discarded.
3. Asymmetric Hysteresis:
   - Degrades to fallback after ~0.5s of silence / stale commands.
   - Recovers to centralized only after N consecutive valid heartbeats AND a minimum dwell time in fallback (2.0s).
4. Continuous Controller Blending:
   - Dynamic ramping weight alpha(t) in [0.0, 1.0] avoids velocity/acceleration step jumps during mode transitions.
   - Decentralized APF collision avoidance safety barrier remains 100% active at all times.
5. Mode Switch Logging: Tracks total switch events per drone for chattering analysis.
"""

from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.drone import Drone


class HybridMode(Enum):
    CENTRALIZED = "centralized"
    DECENTRALIZED_FALLBACK = "decentralized_fallback"


class HybridController:
    """
    Robust hybrid supervisor featuring asymmetric hysteresis, message age validation,
    stale command rejection, continuous alpha(t) blending, and switch metric tracking.
    """

    def __init__(
        self,
        degrade_timeout: float = 0.5,           # Degrade after ~0.5s of silence
        recovery_consecutive_hb: int = 5,       # Consecutive requirement fallback
        min_dwell_time: float = 2.0,            # Minimum time in fallback before recovery (2.0s)
        max_command_age: float = 0.15,          # Stale threshold: reject commands older than threshold
        ramp_duration: float = 0.8,             # Smooth ramping duration tau_ramp for alpha(t)
        window_size: int = 20,                  # Sliding window size for delivery ratio calculation
        recovery_ratio_threshold: float = 0.70, # Recover if delivery ratio >= 70% in sliding window
        fallback_strategy: str = "consensus",   # "consensus", "dead_reckon", "hold_target", "hover"
        dead_reckon_duration: float = 1.5,      # Parameter T: bounded dead reckoning duration in seconds
        central_controller: Optional[CentralizedController] = None,
        decentral_controller: Optional[DecentralizedController] = None,
    ):
        self.central_controller = central_controller if central_controller is not None else CentralizedController()
        self.decentral_controller = decentral_controller if decentral_controller is not None else DecentralizedController()

        # Thresholds
        self.degrade_timeout = float(degrade_timeout)
        self.recovery_consecutive_hb = int(recovery_consecutive_hb)
        self.min_dwell_time = float(min_dwell_time)
        self.max_command_age = float(max_command_age)
        self.ramp_duration = float(ramp_duration)
        self.window_size = int(window_size)
        self.recovery_ratio_threshold = float(recovery_ratio_threshold)
        self.fallback_strategy = str(fallback_strategy).lower()
        self.dead_reckon_duration = float(dead_reckon_duration)

        # State tracking per drone:
        self.modes: Dict[int, HybridMode] = {}
        self.last_valid_timestamp: Dict[int, float] = {}
        self.last_sequence_num: Dict[int, int] = {}
        self.consecutive_good_hb: Dict[int, int] = {}
        self.reception_window: Dict[int, List[int]] = {}
        self.fallback_entry_time: Dict[int, float] = {}
        self.fallback_entry_counts: Dict[int, int] = {}
        self.time_in_fallback: Dict[int, float] = {}
        self.recovery_times: Dict[int, List[float]] = {}
        self.last_known_target: Dict[int, np.ndarray] = {}
        self.last_known_velocity: Dict[int, np.ndarray] = {}
        self.last_known_next_waypoint: Dict[int, np.ndarray] = {}
        self.last_known_offsets: Dict[int, Dict[int, np.ndarray]] = {}
        self.last_known_formation: Dict[int, Any] = {}
        
        # Ramping weight alpha(t) per drone (1.0 = Centralized, 0.0 = Decentralized)
        self.alpha: Dict[int, float] = {}

        # Chattering and stability metrics
        self.switch_counts: Dict[int, int] = {}
        self.stale_rejected_count: Dict[int, int] = {}
        self.out_of_order_count: Dict[int, int] = {}

    def set_nominal_latency(self, latency: float) -> None:
        """
        Dynamically adapts the stale-age threshold relative to expected network latency.
        Resolves threshold clipping during wide latency sweeps (> 150ms).
        """
        self.max_command_age = max(3.0 * float(latency), 0.150)

    def record_heartbeat_attempt(self, drone_id: int, received: bool) -> None:
        """Tracks packet reception across the sliding observation window."""
        if drone_id not in self.reception_window:
            self.reception_window[drone_id] = []
        self.reception_window[drone_id].append(1 if received else 0)
        if len(self.reception_window[drone_id]) > self.window_size:
            self.reception_window[drone_id].pop(0)

    def _init_drone_if_needed(self, drone_id: int, current_time: float) -> None:
        if drone_id not in self.modes:
            self.modes[drone_id] = HybridMode.CENTRALIZED
            self.last_valid_timestamp[drone_id] = current_time
            self.last_sequence_num[drone_id] = -1
            self.consecutive_good_hb[drone_id] = 0
            self.reception_window[drone_id] = [1] * 5  # Initial healthy baseline
            self.fallback_entry_time[drone_id] = 0.0
            self.fallback_entry_counts[drone_id] = 0
            self.time_in_fallback[drone_id] = 0.0
            self.recovery_times[drone_id] = []
            self.alpha[drone_id] = 1.0
            self.switch_counts[drone_id] = 0
            self.stale_rejected_count[drone_id] = 0
            self.out_of_order_count[drone_id] = 0

    def process_coordinator_heartbeat(
        self,
        drone_id: int,
        current_time: float,
        send_timestamp: float,
        sequence_num: int,
        target_pos: np.ndarray,
        target_velocity: Optional[np.ndarray] = None,
        next_waypoint: Optional[np.ndarray] = None,
        desired_offsets: Optional[Dict[int, np.ndarray]] = None,
        formation_type: Optional[Any] = None,
    ) -> bool:
        """
        Validates an incoming coordinator heartbeat packet:
        1. Checks sequence number monotonicity (rejects out-of-order or duplicate packets).
        2. Checks message age (rejects stale commands).
        3. Updates sliding delivery window and consecutive reception counter.
        4. Caches target position, target velocity, next waypoint, formation type, and relative neighbor offsets.
        Returns True if accepted, False if rejected.
        """
        self._init_drone_if_needed(drone_id, current_time)

        # 1. Sequence number check
        if sequence_num <= self.last_sequence_num[drone_id]:
            self.out_of_order_count[drone_id] += 1
            self.record_heartbeat_attempt(drone_id, False)
            return False

        # 2. Message age check (current_time - send_timestamp)
        message_age = current_time - send_timestamp
        if message_age > self.max_command_age or message_age < -1e-4:
            self.stale_rejected_count[drone_id] += 1
            self.consecutive_good_hb[drone_id] = 0
            self.record_heartbeat_attempt(drone_id, False)
            return False

        # Accepted fresh packet!
        self.last_sequence_num[drone_id] = sequence_num
        self.last_valid_timestamp[drone_id] = current_time
        self.last_known_target[drone_id] = np.array(target_pos, dtype=np.float64)
        if target_velocity is not None:
            self.last_known_velocity[drone_id] = np.array(target_velocity, dtype=np.float64)
        if next_waypoint is not None:
            self.last_known_next_waypoint[drone_id] = np.array(next_waypoint, dtype=np.float64)
        if desired_offsets is not None:
            self.last_known_offsets[drone_id] = {
                int(k): np.array(v, dtype=np.float64) for k, v in desired_offsets.items()
            }
        if formation_type is not None:
            self.last_known_formation[drone_id] = formation_type
        self.consecutive_good_hb[drone_id] += 1
        self.record_heartbeat_attempt(drone_id, True)
        return True

    def update_state_machine(self, drone_id: int, current_time: float, dt: float) -> Tuple[HybridMode, float]:
        """
        Evaluates asymmetric hysteresis state transitions and updates continuous blending weight alpha(t).
        Uses both sliding-window delivery ratio and consecutive-packet counts for robust recovery.
        """
        self._init_drone_if_needed(drone_id, current_time)
        current_mode = self.modes[drone_id]
        time_since_valid = current_time - self.last_valid_timestamp[drone_id]

        target_mode = current_mode

        if current_mode == HybridMode.CENTRALIZED:
            # Degrade condition: silence/stale timeout exceeded
            if time_since_valid > self.degrade_timeout:
                target_mode = HybridMode.DECENTRALIZED_FALLBACK
                self.fallback_entry_time[drone_id] = current_time
                self.fallback_entry_counts[drone_id] += 1
                self.consecutive_good_hb[drone_id] = 0
                self.switch_counts[drone_id] += 1

        elif current_mode == HybridMode.DECENTRALIZED_FALLBACK:
            self.time_in_fallback[drone_id] += dt
            # Recovery condition: Requires:
            # 1. Sufficient delivery ratio (>= 70% in sliding window) OR N consecutive good heartbeats
            # 2. Minimum dwell time in fallback state (prevents chattering)
            # 3. Recent valid reception within degrade_timeout
            window = self.reception_window.get(drone_id, [])
            delivery_ratio = (sum(window) / len(window)) if len(window) >= 5 else 0.0

            time_in_fallback = current_time - self.fallback_entry_time[drone_id]
            is_delivery_healthy = (
                self.consecutive_good_hb[drone_id] >= self.recovery_consecutive_hb
                or delivery_ratio >= self.recovery_ratio_threshold
            )

            if (
                is_delivery_healthy
                and time_in_fallback >= self.min_dwell_time
                and time_since_valid <= self.degrade_timeout
            ):
                target_mode = HybridMode.CENTRALIZED
                recovery_dur = current_time - self.fallback_entry_time[drone_id]
                self.recovery_times[drone_id].append(recovery_dur)
                self.switch_counts[drone_id] += 1

        self.modes[drone_id] = target_mode

        # Update smooth ramping weight alpha(t)
        # alpha -> 1.0 in Centralized, alpha -> 0.0 in Decentralized Fallback
        rate = dt / self.ramp_duration
        if target_mode == HybridMode.CENTRALIZED:
            self.alpha[drone_id] = float(np.clip(self.alpha[drone_id] + rate, 0.0, 1.0))
        else:
            self.alpha[drone_id] = float(np.clip(self.alpha[drone_id] - rate, 0.0, 1.0))

        return target_mode, self.alpha[drone_id]

    def compute_hybrid_control(
        self,
        drone: Drone,
        current_time: float,
        dt: float,
        neighbor_states: List[Dict[str, np.ndarray]],
        desired_neighbor_offsets: Optional[Dict[int, np.ndarray]] = None,
        target_velocity: Optional[np.ndarray] = None,
        use_velocity_feedforward: bool = True,
        drag_coeff: float = 0.20,
        measured_position: Optional[np.ndarray] = None,
        measured_velocity: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Computes smoothly blended control command:
            u(t) = alpha(t) * u_central + (1 - alpha(t)) * u_decentral + u_safe_apf
        Safety barrier APF is always applied at 100% gain regardless of alpha(t).
        Optional target velocity feedforward and drag compensation eliminates steady-state lag.
        Uses measured_position and measured_velocity for realistic feedback control when provided.
        """
        mode, alpha = self.update_state_machine(drone.id, current_time, dt)

        pos = np.array(measured_position, dtype=np.float64) if measured_position is not None else drone.position
        vel = np.array(measured_velocity, dtype=np.float64) if measured_velocity is not None else drone.velocity

        # Determine commanded velocity from heartbeat or cached history
        if target_velocity is not None:
            v_cmd = np.array(target_velocity, dtype=np.float64)
        else:
            v_cmd = self.last_known_velocity.get(drone.id, np.zeros(drone.dim, dtype=np.float64))

        # 1. Centralized guidance component (tracked using last validated target slot and measured state)
        target_pos = self.last_known_target.get(drone.id, pos).copy()

        # Apply fallback navigation strategy if degraded / in fallback
        if alpha < 0.999:
            if self.fallback_strategy == "hover":
                # Immediately brake to hover at position where loss was entered
                v_cmd = np.zeros(drone.dim, dtype=np.float64)
                target_pos = pos
                decentral_goal_pos = None
                decentral_goal_vel = np.zeros(drone.dim, dtype=np.float64)
            elif self.fallback_strategy == "hold_target":
                # Maintain position hold at the last received target waypoint
                v_cmd = np.zeros(drone.dim, dtype=np.float64)
                target_pos = self.last_known_target.get(drone.id, pos).copy()
                decentral_goal_pos = target_pos
                decentral_goal_vel = np.zeros(drone.dim, dtype=np.float64)
            elif self.fallback_strategy == "dead_reckon":
                # Continue along last received velocity for bounded time T, then brake
                t_in_fb = current_time - self.fallback_entry_time.get(drone.id, current_time)
                v_last = self.last_known_velocity.get(drone.id, np.zeros(drone.dim, dtype=np.float64))
                p_base = self.last_known_target.get(drone.id, pos)
                if t_in_fb <= self.dead_reckon_duration:
                    target_pos = p_base + v_last * t_in_fb
                    v_cmd = v_last.copy()
                    decentral_goal_pos = target_pos
                    decentral_goal_vel = v_cmd
                else:
                    target_pos = p_base + v_last * self.dead_reckon_duration
                    v_cmd = np.zeros(drone.dim, dtype=np.float64)
                    decentral_goal_pos = target_pos
                    decentral_goal_vel = v_cmd
            else:  # "consensus" (default proposed hybrid fallback)
                # Bounded behavior: consensus flocking with gentle deceleration to hover
                v_cmd = np.zeros(drone.dim, dtype=np.float64)
                target_pos = self.last_known_target.get(drone.id, pos).copy()
                decentral_goal_pos = None
                decentral_goal_vel = np.zeros(drone.dim, dtype=np.float64)
        else:
            decentral_goal_pos = None
            decentral_goal_vel = None

        p_err = target_pos - pos

        if use_velocity_feedforward and np.linalg.norm(v_cmd) > 1e-6:
            v_err = v_cmd - vel
            u_ff = drag_coeff * v_cmd
        else:
            v_err = -vel
            u_ff = np.zeros(drone.dim, dtype=np.float64)

        u_central = self.central_controller.kp * p_err + self.central_controller.kd * v_err + u_ff

        # 2. Decentralized flocking & consensus component
        offsets = desired_neighbor_offsets if desired_neighbor_offsets is not None else self.last_known_offsets.get(drone.id, None)
        u_decentral = self.decentral_controller.compute_drone_control(
            drone=drone,
            neighbor_states=neighbor_states,
            desired_offsets=offsets,
            goal_pos=decentral_goal_pos,
            goal_vel=decentral_goal_vel,
            measured_position=pos,
            measured_velocity=vel,
            use_velocity_feedforward=use_velocity_feedforward,
            drag_coeff=drag_coeff,
        )

        # 3. Always-on local decentralized APF safety barrier (degree-normalised)
        u_safe_apf = np.zeros(drone.dim, dtype=np.float64)
        deg = max(1, len(neighbor_states))
        for n_state in neighbor_states:
            diff = pos - n_state["position"]
            dist = np.linalg.norm(diff)
            if 1e-4 < dist < self.decentral_controller.safe_radius:
                repulse = (
                    self.decentral_controller.k_sep
                    * (1.0 / dist - 1.0 / self.decentral_controller.safe_radius)
                    / (dist**2)
                )
                u_safe_apf += repulse * (diff / dist)
        u_safe_apf = u_safe_apf / deg

        # 4. Smooth continuous blending without double APF:
        # u_decentral already contains (1 - alpha) * f_sep.
        # Adding alpha * u_safe_apf ensures total separation is exactly 1.0 * f_sep for all alpha in [0, 1].
        blended_control = alpha * u_central + (1.0 - alpha) * u_decentral + alpha * u_safe_apf
        return blended_control

    def get_total_mode_switches(self) -> int:
        """Total number of mode transitions across all drones."""
        return sum(self.switch_counts.values())

    def get_drone_mode(self, drone_id: int) -> HybridMode:
        return self.modes.get(drone_id, HybridMode.CENTRALIZED)

    def get_alpha(self, drone_id: int) -> float:
        return self.alpha.get(drone_id, 1.0)

    def get_fallback_stats(self) -> Dict[str, float]:
        """Aggregate fallback entries, total time in fallback, and average recovery times."""
        total_entries = sum(self.fallback_entry_counts.values())
        total_time_fallback = sum(self.time_in_fallback.values())
        all_recovery = []
        for rec_list in self.recovery_times.values():
            all_recovery.extend(rec_list)
        avg_recovery = float(np.mean(all_recovery)) if all_recovery else 0.0
        return {
            "total_fallback_entries": float(total_entries),
            "total_time_in_fallback_s": float(total_time_fallback),
            "avg_recovery_time_s": float(avg_recovery),
            "recovery_events_count": float(len(all_recovery)),
        }
