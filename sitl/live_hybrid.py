"""Host-side hybrid guidance for the unified SITL console.

This uses the research HybridController with per-vehicle state and fresh MAVLink
telemetry. Peer observations are relayed by the console process; this is not an
onboard or radio-distributed controller.
"""

import time

import numpy as np

from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.controllers.hybrid import HybridController
from swarm_core.drone import Drone


class LiveHybridGuidance:
    MIN_DISTANCE = 3.0   # commanded clearance; measured acceptance criterion remains 2.5 m
    INTERACTION_DISTANCE = 5.0
    MAX_SPEED = 1.5
    LOOKAHEAD_S = 0.7

    def __init__(self, count: int):
        self.count = count
        self.controller = HybridController(
            fallback_strategy="hold_target",
            decentral_controller=DecentralizedController(safe_radius=3.0, k_sep=7.0),
        )
        self.drones = [Drone(i, np.zeros(2), max_speed=self.MAX_SPEED, max_accel=2.5,
                             attitude_tau=0.2, drag_coeff=0.0, measurement_noise_std=0.0)
                       for i in range(count)]
        self.last_position = [None] * count
        self.velocity = [np.zeros(2) for _ in range(count)]
        self.last_time = None
        self.sequence = 0
        self.last_modes = ["centralized"] * count

    @staticmethod
    def _limit(vector, maximum):
        norm = float(np.linalg.norm(vector))
        return vector * (maximum / norm) if norm > maximum else vector

    def _safety_filter(self, index, velocity, positions, fresh, current_velocity, exclusions):
        """Project the horizontal command away from nearby measured peers.

        This is a sampled velocity constraint, not a collision-free guarantee for
        a delayed vehicle position loop. The final speed remains bounded.
        """
        result = velocity.copy()
        for _ in range(3):
            for peer in range(self.count):
                if peer == index or (not fresh[peer] and peer not in exclusions):
                    continue
                if peer not in exclusions and abs(positions[index][2] - positions[peer][2]) > 2.0:
                    continue
                delta = positions[index][:2] - positions[peer][:2]
                distance_sq = float(np.dot(delta, delta))
                minimum = 4.2 if peer in exclusions else self.MIN_DISTANCE
                interaction = 6.0 if peer in exclusions else self.INTERACTION_DISTANCE
                if distance_sq >= interaction**2:
                    continue
                if distance_sq < 0.01:
                    delta = np.array([1.0, 0.0]) if index < peer else np.array([-1.0, 0.0])
                    distance_sq = 1.0
                h = distance_sq - minimum**2
                required = float(np.dot(delta, current_velocity[peer]) - 0.5 * h)
                actual = float(np.dot(delta, result))
                if actual < required:
                    result += ((required - actual) / distance_sq) * delta
            result = self._limit(result, self.MAX_SPEED)
        return result

    def step(self, positions, nominal_targets, active, fresh, coordinator_enabled, exclusions=(), now=None):
        now = time.monotonic() if now is None else now
        dt = 0.1 if self.last_time is None else min(0.25, max(0.02, now - self.last_time))
        self.last_time = now
        self.sequence += 1
        pos = [np.asarray(p, dtype=float) for p in positions]
        for i in range(self.count):
            if fresh[i]:
                if self.last_position[i] is not None:
                    measured = (pos[i][:2] - self.last_position[i]) / dt
                    self.velocity[i] = self._limit(0.7 * self.velocity[i] + 0.3 * measured, 3.0)
                self.last_position[i] = pos[i][:2].copy()

        outputs = [None] * self.count
        transitions = []
        alphas = [None] * self.count
        modes = [None] * self.count
        for i in range(self.count):
            if not active[i] or not fresh[i] or nominal_targets[i] is None:
                continue
            target = np.asarray(nominal_targets[i], dtype=float)
            offsets = {j: target[:2] - np.asarray(nominal_targets[j], dtype=float)[:2]
                       for j in range(self.count) if j != i and active[j] and nominal_targets[j] is not None}
            if coordinator_enabled:
                self.controller.process_coordinator_heartbeat(
                    i, now, now, self.sequence, target[:2],
                    target_velocity=np.zeros(2), desired_offsets=offsets,
                )
            else:
                self.controller.record_heartbeat_attempt(i, False)
            neighbors = [{"id": j, "position": pos[j][:2], "velocity": self.velocity[j]}
                         for j in range(self.count) if j != i and active[j] and fresh[j]]
            accel = self.controller.compute_hybrid_control(
                self.drones[i], now, dt, neighbors,
                desired_neighbor_offsets=offsets if coordinator_enabled else None,
                measured_position=pos[i][:2], measured_velocity=self.velocity[i],
                use_velocity_feedforward=False, drag_coeff=0.0,
            )
            proposed = self._limit(self.velocity[i] + np.asarray(accel) * dt, self.MAX_SPEED)
            safe = self._safety_filter(i, proposed, pos, fresh, self.velocity, exclusions)
            output = pos[i].copy()
            output[:2] += safe * self.LOOKAHEAD_S
            output[2] = target[2] if coordinator_enabled else pos[i][2]
            outputs[i] = output.tolist()
            mode = self.controller.get_drone_mode(i).value
            modes[i] = mode
            alphas[i] = round(self.controller.get_alpha(i), 3)
            if mode != self.last_modes[i]:
                transitions.append({"drone": i + 1, "from": self.last_modes[i], "to": mode})
                self.last_modes[i] = mode
        return outputs, modes, alphas, transitions
