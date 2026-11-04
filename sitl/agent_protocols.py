"""Agent side of the formation-transition and mode-switch protocols.

Part of `IndependentAgentNode` (independent_agent_node.py); split out for size, behaviour unchanged."""
from __future__ import annotations
import time
import numpy as np
from sitl.mode_switch import AgentModeSwitch, ModeContext
from sitl.transition_protocol import AgentContext, AgentTransition, PeerObservation


class AgentProtocolMixin:
    def _remember_peers(self, discovered, wall_now: float) -> None:
        """Keep each peer's latest state; ``discovered`` only holds packets received this tick."""
        for peer in discovered:
            try:
                position = np.asarray(peer["position"], dtype=float)[:2]
                velocity = np.asarray(peer.get("velocity", (0.0, 0.0)), dtype=float)[:2]
                if position.shape != (2,) or velocity.shape != (2,):
                    continue
                self._peer_memory[int(peer["id"])] = (
                    position, velocity, float(peer.get("age", 0.0)), float(wall_now))
                if not hasattr(self, "_peer_degraded"):
                    self._peer_degraded = {}
                self._peer_degraded[int(peer["id"])] = bool(peer.get("degraded", False))
            except (KeyError, TypeError, ValueError, IndexError):
                continue

    def _transition_context(self, wall_now: float) -> AgentContext:
        """Snapshot of what this agent knows, for the transition state machine."""
        fresh = set(self.peers.fresh_link_ids(wall_now))
        peers = {}
        for peer_id, (position, velocity, age_at_receipt, received_wall) in self._peer_memory.items():
            age = age_at_receipt + max(0.0, wall_now - received_wall)
            extrapolated = position + velocity * age
            peers[peer_id] = PeerObservation((float(extrapolated[0]), float(extrapolated[1])), age)
        own_fresh = (self.position is not None and bool(self.position_at) and
                     0.0 <= wall_now - self.position_at <= 1.0)
        coordinator_age = (time.monotonic() - self.last_coordinator_message_monotonic
                           if self.last_coordinator_message_monotonic else float("inf"))
        return AgentContext(
            airborne_armed=bool(self.armed and self.airborne and not self.landing),
            own_fresh=own_fresh,
            peers_live=set(self.peer_ids) <= fresh,
            own_position_ne=(None if self.position is None
                             else (float(self.position[0]), float(self.position[1]))),
            peers=peers, coordinator_age_s=coordinator_age,
        )

    def _handle_transition_message(self, kind: str, message: dict) -> None:
        epoch = message.get("epoch")
        if not isinstance(epoch, str) or epoch != self.clock_sync_epoch:
            return
        wall = time.time()
        operator_now = wall - self.operator_clock_offset_s
        if kind == "transition_prepare" and self.mode_switch.state != AgentModeSwitch.IDLE:
            reply = self.transition._ack(message.get("transition_id"), message.get("plan_hash"),
                                         False, "mode_switch_active")
            reply["epoch"] = epoch
            self._send_to_operator(reply)
        elif kind == "transition_prepare":
            reply = self.transition.on_prepare(message, operator_now,
                                               self._transition_context(wall))
            reply["epoch"] = epoch
            self._send_to_operator(reply)
        elif kind == "transition_go":
            reply = self.transition.on_go(message, operator_now)
            if reply is not None:
                reply["epoch"] = epoch
                self._send_to_operator(reply)
        elif kind == "transition_abort":
            self.transition.on_abort(message, operator_now)
        elif kind == "transition_done":
            plan = self.transition.plan
            self.transition.on_done(message)
            if plan is not None and self.transition.state == AgentTransition.IDLE:
                self._finish_transition(plan)

    def _mode_context(self, wall_now: float) -> ModeContext:
        fresh = (self.position is not None and bool(self.position_at) and
                 0.0 <= wall_now - self.position_at <= 1.0)
        return ModeContext(healthy=bool(fresh and not self.landing),
                           formation_idle=self.transition.state == AgentTransition.IDLE)

    def _handle_mode_message(self, kind: str, message: dict) -> None:
        epoch = message.get("epoch")
        if not isinstance(epoch, str) or epoch != self.clock_sync_epoch:
            return
        wall = time.time()
        operator_now = wall - self.operator_clock_offset_s
        if kind == "mode_prepare":
            reply = self.mode_switch.on_prepare(message, operator_now, self._mode_context(wall))
            reply["epoch"] = epoch
            self._send_to_operator(reply)
        elif kind == "mode_go":
            reply = self.mode_switch.on_go(message, operator_now)
            if reply is not None:
                reply["epoch"] = epoch
                self._send_to_operator(reply)
        elif kind == "mode_abort":
            self.mode_switch.on_abort(message, operator_now)
        elif kind == "mode_done":
            self.mode_switch.on_done(message)

    def _tick_mode_switch(self, wall_now: float) -> None:
        operator_now = wall_now - self.operator_clock_offset_s
        for action in self.mode_switch.tick(operator_now, self._mode_context(wall_now)):
            if action[0] == "send":
                message = dict(action[1])
                message["epoch"] = self.clock_sync_epoch
                self._send_to_operator(message)
        mode = self.mode_switch.dominant_mode(operator_now)
        if mode != self.control_mode:
            print(f"[NODE {self.drone_id + 1}] control mode {self.control_mode} -> {mode}", flush=True)
            self.control_mode = mode
            self.agent.mode = mode

    def _transition_goal(self, operator_now: float):
        """Swarm centre to hold while a transition owns the slots, else None (use the mission goal)."""
        transition = getattr(self, "transition", None)
        if transition is None or transition.plan is None:
            return None
        if transition.state == AgentTransition.HOLD or (
                transition.state in (AgentTransition.RUNNING, AgentTransition.ARRIVED) and
                operator_now >= transition.plan.start_at):
            return np.asarray(transition.plan.centroid_ne, dtype=float)
        return None

    def _finish_transition(self, plan) -> None:
        """The morph is complete: adopt the target formation exactly."""
        self.formation = plan.formation
        if self.membership is not None:
            self.membership.formation = plan.formation
        target = np.asarray(plan.target_slots, dtype=float)
        self.operator_formation_slots = target.copy()
        self.all_slots = target.copy()
        self.slot = self.all_slots[self.drone_id].copy()
        self.formation_transition_progress = 1.0
        self.membership_revision += 1
        # Keep holding the plan's centre until a mission intent issued after completion arrives (the operator re-bases
        # its goal there). Resuming the older goal made every target jump: ~11 m after the tower-gap column (2026-11-04).
        self._post_transition_centre = (np.asarray(plan.centroid_ne, dtype=float).copy(),
                                        time.time() - getattr(self, "operator_clock_offset_s", 0.0))

    def _tick_transition(self, wall_now: float) -> None:
        if self.transition.state == AgentTransition.IDLE:
            return
        plan = self.transition.plan
        operator_now = wall_now - self.operator_clock_offset_s
        for action in self.transition.tick(operator_now, self._transition_context(wall_now)):
            if action[0] == "send":
                message = dict(action[1])
                message["epoch"] = self.clock_sync_epoch
                self._send_to_operator(message)
            elif action[1] == "arrived_timeout_close" and plan is not None:
                self._finish_transition(plan)
        if self.transition.state in (AgentTransition.RUNNING, AgentTransition.ARRIVED,
                                     AgentTransition.HOLD) and (
                self.transition.state == AgentTransition.HOLD or
                operator_now >= self.transition.plan.start_at):
            slots = self.transition.current_slots(operator_now)
            self.all_slots = slots.copy()
            self.slot = self.all_slots[self.drone_id].copy()
            self.formation_transition_progress = float(
                self.transition.status(operator_now)["progress"] or 0.0)
