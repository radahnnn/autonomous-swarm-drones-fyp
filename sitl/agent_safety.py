"""Agent safety layer: barrier-filter neighbours, position-estimate health, geofence, obstacle sensing/avoidance, peer failsafe.

Part of `IndependentAgentNode` (independent_agent_node.py); split out for size, behaviour unchanged."""
from __future__ import annotations
import time
import numpy as np


class AgentSafetyMixin:
    def _safety_neighbors(self, wall_now: float, max_age_s: float = 1.5) -> list:
        """Every recently heard peer (not just the two ring neighbours) for the barrier safety filter."""
        peers = []
        degraded = getattr(self, "_peer_degraded", {})
        for peer_id, (position, velocity, age_at_receipt, received_wall) in getattr(self, "_peer_memory", {}).items():
            age = age_at_receipt + max(0.0, wall_now - received_wall)
            if age <= max_age_s:
                # A peer whose position estimate is degraded may be metres from where it says: keep 2 m more.
                peers.append({"position": position + velocity * age, "velocity": velocity,
                              "extra_m": self.DEGRADED_PEER_EXTRA_M if degraded.get(peer_id) else 0.0})
        return peers

    DEGRADED_PEER_EXTRA_M = 2.0

    def _tick_estimate_health(self, now: float) -> None:
        """Watch this drone's own position estimate; freeze on a glitch, land if it does not recover."""
        health = getattr(self, "estimate_health", None)
        if health is None or self.position is None or not self.position_at or not self.airborne:
            return
        before = health.state
        upset = getattr(self, "_estimator_upset_at", None)
        if upset is not None:
            self._estimator_upset_at = None
            health.note_estimator_upset(self.position_at)
        health.update(self.position[:2], self.position_at)
        state = health.check_tracking(self.position[:2], getattr(self, "last_position_target_ne", None),
                                      self.position_at)
        if state != before:
            print(f"[NODE {self.drone_id + 1}] position estimate {before} -> {state} "
                  f"{{'glitches': {health.glitches}, 'estimator_upsets': {health.estimator_upsets}}}", flush=True)
            if state == health.DEGRADED:
                self._frozen_target = getattr(self, "last_position_target_ne", None)
            elif state == health.OK:
                self._frozen_target = None
        if state == health.LAND and not self.landing:
            self.safety_state = "estimate_degraded"
            self.safety_reason = "position_estimate_or_flyaway"
            self._command_land(self.safety_reason)

    def _tick_geofence(self) -> None:
        """Detection: report inside/warn/breach and land if a breach persists."""
        fence = getattr(self, "geofence", None)
        if fence is None or self.position is None:
            return
        state = fence.status(self.position[:2], -float(self.position[2]))
        if state != self.fence_state:
            if state == "breach":
                self.fence_breaches += 1
            print(f"[NODE {self.drone_id + 1}] geofence {self.fence_state} -> {state}", flush=True)
            self.fence_state = state
        if state != "breach":
            self.fence_breach_since = None
            return
        now = time.monotonic()
        if self.fence_breach_since is None:
            self.fence_breach_since = now
        elif now - self.fence_breach_since > self.FENCE_BREACH_LAND_S and not self.landing:
            self._command_land("geofence_breach")

    def _check_peer_failsafe(self, now: float, fresh_peer_ids, peer_states=()) -> None:
        """Degrade around one lost edge; land only when this node is isolated."""
        if not self.airborne:
            # Learn which configured peers were actually reachable before takeoff, but do
            # not age a missing link into a preflight emergency.
            self.peer_watchdog.update(now, fresh_peer_ids)
            self.peer_watchdog.deactivate()
            self.peer_timeout_ids = []
            return
        # A peer's local LAND is state information, not a swarm-wide command.  Copying
        # it through the graph made one failed directed link cascade into a fleet-wide
        # landing.  Keep its slot reserved and let each remaining node independently
        # assess whether it still has a live route.
        peers_landing = sorted({int(peer["id"]) for peer in peer_states
                                if peer.get("landing") is True})
        if peers_landing and not self.landing:
            ids = ",".join(str(peer_id + 1) for peer_id in peers_landing)
            self.safety_state = "peer_landing_observed"
            self.safety_reason = f"peer_landing_observed:{ids}"
        if not self.peer_watchdog.monitoring:
            self.peer_watchdog.activate(now, fresh_peer_ids)
        self.peer_timeout_ids = self.peer_watchdog.update(
            now, fresh_peer_ids,
        )
        if self.peer_timeout_ids and not self.landing:
            missing = ",".join(str(peer_id + 1) for peer_id in self.peer_timeout_ids)
            required_ids = getattr(self, "peer_ids", self.peer_watchdog.peer_ids)
            live_required = sorted(set(required_ids) & set(int(value)
                                                             for value in fresh_peer_ids))
            if live_required:
                # Guidance already consumes only fresh received neighbors, so this is
                # the local topology reconfiguration: remove the failed edge and keep
                # controlling through the remaining one-hop links.
                self.safety_state = "peer_link_degraded"
                self.safety_reason = f"peer_link_degraded:{missing}"
                self.last_command = self.safety_reason
            else:
                self.safety_state = "peer_isolation_failsafe"
                self.safety_reason = f"isolated_peer_timeout:{missing}"
                self._command_land(self.safety_reason)
        elif (not self.peer_timeout_ids and not peers_landing and not self.landing and
              self.safety_state in {"peer_link_degraded", "peer_landing_observed"}):
            self.safety_state = "nominal"
            self.safety_reason = None
            self.last_command = "peer_links_restored"

    def _all_obstacles(self) -> list:
        """Obstacles given in advance plus those this drone or a peer has sensed."""
        sensor = getattr(self, "sensor", None)
        return list(getattr(self, "obstacles", ())) + (list(sensor.known) if sensor is not None else [])

    def _tick_obstacles(self, wall_now: float) -> None:
        """Sense obstacles from the current position, learn peers' detections, and share what is known."""
        sensor = getattr(self, "sensor", None)
        if sensor is None:
            return
        found = sensor.update(self.position[:2]) if self.position is not None else []
        for message in self.peers.take_obstacle_messages():
            for obstacle in sensor.learn(message["items"]):
                print(f"[NODE {self.drone_id + 1}] obstacle learned from D{message['sender'] + 1} "
                      f"{{'north': {obstacle[0]:.1f}, 'east': {obstacle[1]:.1f}}}", flush=True)
        for obstacle in found:
            print(f"[NODE {self.drone_id + 1}] obstacle detected "
                  f"{{'north': {obstacle[0]:.1f}, 'east': {obstacle[1]:.1f}, 'radius': {obstacle[2]:.1f}}}", flush=True)
        known = len(sensor.known)
        if known and (known != self._obstacles_published_count or wall_now - self._obstacles_published_at >= 2.0):
            self.peers.publish_obstacles(self.discovery_ids, wall_now, sensor.shareable())
            self._obstacles_published_count, self._obstacles_published_at = known, wall_now

    def _avoid_obstacles(self, target_ne: np.ndarray) -> np.ndarray:
        """Apply a bounded local repulsion from configured circular keep-out zones."""
        target = np.asarray(target_ne, dtype=float).copy()
        if self.position is None:
            return target
        position = np.asarray(self.position[:2], dtype=float)
        correction = np.zeros(2, dtype=float)
        for north, east, radius in self._all_obstacles():
            delta = position - np.array([north, east], dtype=float)
            distance = float(np.linalg.norm(delta))
            influence = radius + 3.0
            if distance >= influence:
                continue
            direction = delta / max(distance, 0.10)
            correction += direction * min(2.0, influence - distance)
        return target + correction
