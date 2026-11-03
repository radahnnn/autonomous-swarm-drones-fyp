"""Agent mission handling: leases, acting-coordinator election, coordinator-loss policy, routes, adaptive membership.

Part of `IndependentAgentNode` (independent_agent_node.py); split out for size, behaviour unchanged."""
from __future__ import annotations
import time
import numpy as np
from sitl.mission_intent import MissionIntent


class AgentMissionMixin:
    def _refresh_mission(self, intent_dict: dict, wall_now: float, mono: float,
                         from_coordinator: bool = False) -> None:
        """Extend the mission lease AND the agent's cached copy of the intent.

        The cached intent is re-validated on every guidance step and expires with its own TTL; extending
        only the lease would make a long autonomous period end in an "invalid mission intent" landing.
        """
        operator_now = wall_now - self.operator_clock_offset_s
        self.mission_lease.accept(intent_dict, operator_now, mono)
        self.current_mission = dict(intent_dict)
        self.agent.cached_mission_intent = dict(intent_dict)
        if from_coordinator:
            self.last_coordinator_message_monotonic = mono
            self.coordinator_connected = True

    def _renewed_copy(self, base: dict, wall_now: float, ttl_s: float = 30.0) -> dict:
        """The accepted mission with fresh issue/expiry times (route and goal unchanged)."""
        operator_now = wall_now - self.operator_clock_offset_s
        intent = dict(base)
        intent.update(issued_at=operator_now, expires_at=operator_now + ttl_s, formation="configured",
                      formation_slots=None, formation_start_slots=None, formation_transition_s=0.0)
        return intent

    def _renew_lease_if_autonomous(self) -> None:
        """Decentralized/hybrid agents keep flying the last mission on peers when the coordinator is gone."""
        if (self.control_mode not in ("decentralized", "hybrid") or self.current_mission is None or
                self.coordinator_connected or not self.last_coordinator_message_monotonic):
            return
        if time.monotonic() - self.last_coordinator_message_monotonic > self.AUTONOMY_MAX_S:
            return
        wall = time.time()
        if not (set(self.peer_ids) & set(self.peers.fresh_link_ids(wall))):
            return          # alone: let the lease lapse and the existing failsafes act
        if self.mission_lease.active(time.monotonic()) is None:
            return          # already lapsed
        self._refresh_mission(self._renewed_copy(self.current_mission, wall), wall, time.monotonic())

    # ---- acting-coordinator failover (sitl/coordinator_election.py)
    def _election_events(self, actions) -> None:
        for action in actions:
            if action[0] == "event":
                print(f"[NODE {self.drone_id + 1}] {action[1].replace('_', ' ')} {action[2]}", flush=True)

    LOCAL_PAUSE_S = 2.0   # a longer gap between election ticks means this agent, not the operator, went quiet

    def _tick_election(self, wall_now: float) -> None:
        mono = time.monotonic()
        last = getattr(self, "_last_election_tick", None)
        if last is not None and mono - last > self.LOCAL_PAUSE_S:
            self.election.resume_after_local_pause(mono)
        self._last_election_tick = mono
        for message in self.peers.take_coordinator_messages():
            self._handle_acting_intent(message, wall_now, mono)
        if getattr(self, "membership", None) is None:
            return                                      # failover needs the adaptive leader election
        live = len(self.peers.fresh_link_ids(wall_now))
        for action in self.election.tick(mono, self.elected_leader_id, live):
            if action[0] == "publish":
                self._publish_acting_intent(action[1], wall_now, mono)
            else:
                self._election_events([action])

    def _publish_acting_intent(self, term: int, wall_now: float, mono: float) -> None:
        if self.current_mission is None:
            return
        self._acting_sequence += 1
        intent = self._renewed_copy(self.current_mission, wall_now)
        intent["sequence"] = self._acting_sequence
        self.peers.publish_coordinator(self.discovery_ids, wall_now, term, intent)
        self._refresh_mission(intent, wall_now, mono, from_coordinator=True)

    def _handle_acting_intent(self, message: dict, wall_now: float, mono: float) -> None:
        # Validate the carried mission and its order BEFORE it can refresh the election's view of the leader
        # (Codex review #10: an invalid or replayed mission used to keep an acting coordinator looking alive).
        operator_now = wall_now - self.operator_clock_offset_s
        try:
            intent = MissionIntent.from_dict(message["intent"], operator_now)
        except (TypeError, ValueError, KeyError):
            return
        key = (message["sender"], message["term"])
        if intent.sequence <= self.last_acting_sequence.get(key, 0):
            return
        accept, actions = self.election.on_acting_message(message["sender"], message["term"], mono)
        self._election_events(actions)
        if not accept:
            return
        self.last_acting_sequence[key] = intent.sequence
        self._refresh_mission(message["intent"], wall_now, mono, from_coordinator=True)

    def _centralized_fail_stop(self, now: float) -> bool:
        """Centralized mode depends on the coordinator: hold position, then land. True = handled."""
        if self.control_mode != "centralized" or self.coordinator_connected:
            self.central_hold_since = None
            return False
        if self.central_hold_since is None:
            self.central_hold_since = now
            print(f"[NODE {self.drone_id + 1}] centralized: coordinator lost; holding position", flush=True)
        if self.position is not None:
            self._send_position_target(np.array(self.position, dtype=float), -float(self.position[2]),
                                       apply_layer=False)
        if now - self.central_hold_since > self.CENTRAL_HOLD_LAND_S:
            self._command_land("coordinator_lost_centralized")
        return True

    def _route_goal(self, intent, operator_now: float):
        """Goal along the mission route (a pure function of the shared clock), or None."""
        route = intent.route() if hasattr(intent, "route") else None
        return None if route is None else route.position(operator_now)

    def _goal_override(self, intent, operator_now: float):
        """A formation transition owns the swarm centre; otherwise the route (if any) drives the goal."""
        held = self._transition_goal(operator_now)
        return held if held is not None else self._route_goal(intent, operator_now)

    def _update_adaptive_membership(self, now: float, fresh_ids) -> None:
        if self.membership is None:
            return
        if self.membership.update(now, fresh_ids):
            self.membership_revision += 1
        slot_map = self.membership.slot_map()
        self.peer_ids = self.membership.neighbors_for()
        membership_slots = np.asarray([
            slot_map.get(agent_id, self.all_slots[agent_id])
            for agent_id in range(self.count)
        ], dtype=float)
        self.all_slots = (self.operator_formation_slots.copy()
                          if self.operator_formation_slots is not None else membership_slots)
        self.slot = self.all_slots[self.drone_id].copy()
        self.elected_leader_id = self.membership.leader_id
        if len(self.membership.active_ids) == 1 and self.airborne and not self.landing:
            self.safety_state = "peer_isolation_failsafe"
            self.safety_reason = "adaptive_membership_isolated"
            self._command_land(self.safety_reason)
        elif self.safety_state in {"peer_link_degraded", "peer_landing_observed"}:
            self.safety_state = "nominal"
            self.safety_reason = None
