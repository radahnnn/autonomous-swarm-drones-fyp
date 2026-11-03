"""Coordinator side of the mode-switch and formation-transition protocols, and formation commands.

Part of `IndependentSwarmConsole` (independent_swarm_console.py); split out for size, behaviour unchanged."""
from __future__ import annotations
import math
import threading
import time
import numpy as np
from sitl.formation_library import STYLES
from sitl.formation_transition import plan_formation_transition
from sitl.mode_switch import CONTROL_MODES, ModeView
from sitl.transition_protocol import AgentView, TransitionPlan


class ProtocolMixin:
    # ---- synchronized in-flight control-mode switching
    def _dispatch_mode(self, actions) -> None:
        epoch = (self.clock_sync_message or {}).get("epoch")
        for action in actions:
            if action[0] == "send":
                if not epoch:
                    continue
                message = dict(action[1])
                message["epoch"] = epoch
                try:
                    self._send_to_agents(message)
                except OSError as exc:
                    self.event("mode_send_error", error=str(exc), type=message.get("type"))
            elif action[0] == "event":
                self.event(action[1], **action[2])
                self._on_mode_event(action[1], action[2])

    def _on_mode_event(self, name: str, data: dict) -> None:
        if name in ("mode_switch_complete", "mode_switch_aborted", "mode_switch_rolled_back"):
            self._note_metric(name, **{k: v for k, v in data.items() if isinstance(v, (str, int, float, bool))})
        if name == "mode_switch_complete":
            self.control_mode = data["mode"]
            print(f">> Control mode is now {data['mode'].upper()} on all {self.count} drones.", flush=True)
        elif name == "mode_switch_aborted":
            print(f"!! Mode switch aborted ({data.get('reason')}); "
                  f"{'rolling back to ' + self.mode_switch.plan.from_mode.upper() if data.get('started') else 'nothing changed'}.",
                  flush=True)
        elif name == "mode_switch_rolled_back":
            print(f">> Rolled back: all drones are in {self.mode_switch.plan.from_mode.upper()} mode.", flush=True)
        elif name == "mode_rollback_unverified":
            print("!! Rollback not confirmed by every drone; check status.", flush=True)

    def _on_agent_mode_message(self, index: int, message: dict) -> None:
        epoch = (self.clock_sync_message or {}).get("epoch")
        if message.get("epoch") != epoch:
            return
        sender = message.get("drone_id") if message.get("type") == "mode_ack" else message.get("from")
        if sender != index:
            return
        with self.mode_lock:
            if message["type"] == "mode_ack":
                actions = self.mode_switch.on_ack(message, time.time())
            else:
                actions = self.mode_switch.on_agent_abort(message, time.time())
        self._dispatch_mode(actions)

    def _mode_views(self, now: float) -> dict:
        with self.lock:
            states = {index: state.copy() for index, state in self.latest.items()}
        views = {}
        for index, state in states.items():
            try:
                age = now - float(state.get("operator_received_at", state.get("timestamp")))
                position = np.asarray(state.get("position"), dtype=float)
                info = state.get("mode_switch") or {}
                views[index] = ModeView(
                    age_s=age, mode=str(info.get("mode", state.get("control_mode", ""))),
                    state=str(info.get("state", "IDLE")), switch_id=info.get("switch_id"),
                    healthy=state.get("safety_state") == "nominal",
                    position_ne=((float(position[0]), float(position[1]))
                                 if position.shape == (3,) and np.isfinite(position).all() else None),
                )
            except (TypeError, ValueError):
                continue
        return views

    def _tick_mode_switch_protocol(self) -> None:
        if not self.mode_switch.busy:
            return
        with self.mode_lock:
            actions = self.mode_switch.tick(time.time(), self._mode_views(time.time()))
        self._dispatch_mode(actions)

    def set_control_mode(self, mode: str) -> dict:
        """Switch the whole swarm to ``mode`` in flight (synchronized, bumpless, with rollback)."""
        def result(accepted: bool, message: str, **details):
            self.last_command_result = {"accepted": accepted, "message": message, **details}
            print((">> " if accepted else ">> Mode switch blocked: ") + message, flush=True)
            return self.last_command_result

        mode = str(mode).strip().lower()
        if mode not in CONTROL_MODES:
            return result(False, f"Unknown mode {mode!r}; choose from {', '.join(CONTROL_MODES)}.")
        if mode == self.mode_switch.mode and not self.mode_switch.busy:
            return result(True, f"{mode.upper()} is already active.", unchanged=True)
        if self.mode_switch.busy:
            return result(False, "A mode switch is already in progress.", code="mode_switch_active")
        # A HOLD after an aborted transition is a stationary hover: switching the control mode there is safe and
        # must stay possible (a live Gazebo run under GPS noise was otherwise stuck). Only a running transition blocks.
        if (getattr(self, "formation_transition_deadline", None) is not None or
                (self.transition_protocol and self.transition.busy)):
            return result(False, "Finish or abort the formation transition first.",
                          code="transition_active")
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        now = time.time()
        if any(not state.get("operator_received_at") or
               not 0.0 <= now - float(state.get("operator_received_at", 0.0)) <= 2.0
               for state in states):
            return result(False, "Every drone needs a fresh status heartbeat.")
        if any(state.get("safety_state") != "nominal" for state in states):
            return result(False, "Every drone must report nominal safety status.")
        problem = self._lead_too_short(self.mode_switch.params.lead_s)
        if problem:
            return result(False, problem)
        with self.mode_lock:
            self.mode_switch.mode = self.control_mode if self.control_mode in CONTROL_MODES else self.mode_switch.mode
            actions = self.mode_switch.begin(mode, now)
        self._dispatch_mode(actions)
        plan = self.mode_switch.plan
        return result(True, f"Switching {plan.from_mode.upper()} -> {mode.upper()}: waiting for all "
                      f"{self.count} drones to acknowledge, common start in "
                      f"{self.mode_switch.params.lead_s:.0f}s, {plan.ramp_s:.0f}s blend.",
                      mode=mode, switch_id=plan.switch_id)

    def _dispatch_transition(self, actions) -> None:
        """Send protocol messages and react to protocol events (called without holding locks)."""
        epoch = (self.clock_sync_message or {}).get("epoch")
        for action in actions:
            if action[0] == "send":
                if not epoch:
                    continue
                message = dict(action[1])
                message["epoch"] = epoch
                try:
                    self._send_to_agents(message)
                except OSError as exc:
                    self.event("transition_send_error", error=str(exc), type=message.get("type"))
            elif action[0] == "event":
                self.event(action[1], **action[2])
                self._on_transition_event(action[1], action[2])

    def _on_transition_event(self, name: str, data: dict) -> None:
        if name in ("transition_complete", "transition_aborted", "needs_operator"):
            self._note_metric(name, **{k: v for k, v in data.items() if isinstance(v, (str, int, float, bool))})
        pending = self.transition_pending
        if name == "transition_complete" and pending is not None:
            style, slots, _ = pending
            self.formation = style
            self.formation_slots = slots
            self.transition_pending = None
            self.formation_hold = False
            self.event("formation_transition_complete", formation=style)
            getattr(self, "_transition_retries", {}).pop(style, None)
            print(f">> Formation transition to {style.upper()} complete.", flush=True)
        elif name == "transition_aborted" and pending is not None:
            if data.get("moved"):
                # Drones froze mid-transition at the plan's centroid; the operator re-plans or lands.
                self.formation_hold = True
                print(f"!! Formation transition aborted ({data.get('reason')}); drones are HOLDING "
                      "position. Re-issue a formation command to re-plan, or LAND.", flush=True)
            else:
                self.transition_pending = None
                style = pending[0]
                reason = str(data.get("reason", ""))
                retries = getattr(self, "_transition_retries", {})
                used = retries.get(style, 0)
                if any(word in reason for word in self.TRANSIENT_ABORTS) and used < self.TRANSITION_RETRIES:
                    retries[style] = used + 1
                    self._transition_retries = retries
                    delay = 3.0 / self.time_scale
                    print(f">> Formation transition cancelled before start ({reason}); no drone moved. "
                          f"Transient on a lossy link: retrying in {delay:.0f} s "
                          f"({used + 1}/{self.TRANSITION_RETRIES}).", flush=True)
                    self.event("formation_transition_retry", formation=style, reason=reason, attempt=used + 1)
                    timer = threading.Timer(delay, self._set_formation, args=(style,))
                    timer.daemon = True
                    timer.start()
                else:
                    print(f">> Formation transition cancelled before start ({reason}); "
                          "no drone moved.", flush=True)
        elif name == "needs_operator":
            print("!! Formation HOLD has lasted too long; command a new formation or LAND.", flush=True)

    def _on_agent_transition_message(self, index: int, message: dict) -> None:
        if not self.transition_protocol:
            return
        epoch = (self.clock_sync_message or {}).get("epoch")
        if message.get("epoch") != epoch:
            return
        sender = message.get("drone_id") if message.get("type") == "transition_ack" else message.get("from")
        if sender != index:
            return
        with self.transition_lock:
            if message["type"] == "transition_ack":
                actions = self.transition.on_ack(message, time.time())
            else:
                actions = self.transition.on_agent_abort(message, time.time())
        self._dispatch_transition(actions)

    def _transition_views(self, now: float) -> dict:
        with self.lock:
            states = {index: state.copy() for index, state in self.latest.items()}
        views = {}
        for index, state in states.items():
            try:
                age = now - float(state.get("operator_received_at", state.get("timestamp")))
                position = np.asarray(state.get("position"), dtype=float)
                if position.shape != (3,) or not np.isfinite(position).all():
                    continue
                info = state.get("transition") or {}
                views[index] = AgentView(
                    age_s=age,
                    armed_airborne=state.get("armed") is True and state.get("airborne") is True,
                    safety_nominal=state.get("safety_state") == "nominal",
                    position_ne=(float(position[0]), float(position[1])),
                    state=str(info.get("state", "IDLE")),
                    transition_id=info.get("transition_id"),
                )
            except (TypeError, ValueError):
                continue
        return views

    def _tick_transition_protocol(self) -> None:
        now = time.time()
        views = self._transition_views(now)
        with self.transition_lock:
            actions = self.transition.tick(now, views)
        self._dispatch_transition(actions)

    def _begin_protocol_transition(self, style, plan, positions, centroid, now, result):
        problem = self._lead_too_short(self.transition.params.lead_s)
        if problem:
            return result(False, problem)
        start_slots = np.column_stack((positions - centroid, np.zeros(self.count)))
        try:
            tplan = TransitionPlan.build(
                self.transition.next_id(), style, centroid, start_slots, plan.slots_by_drone,
                plan.duration_s, now + self.transition.params.lead_s)
        except ValueError as exc:
            return result(False, f"Transition plan rejected: {exc}")
        # The mission goal is left untouched: agents hold the plan's centroid themselves while the
        # transition owns the slots, then resume this goal when it completes.
        self.transition_pending = (style, np.asarray(plan.slots_by_drone, dtype=float).copy(),
                                   self.goal_ne.copy())
        self.formation_hold = False
        with self.transition_lock:
            actions = self.transition.begin(tplan, now)
        self._dispatch_transition(actions)
        return result(True, f"Transition to {style.upper()} prepared; waiting for all {self.count} "
                      f"drones to acknowledge, then a common start in "
                      f"{self.transition.params.lead_s:.0f}s ({plan.duration_s:.1f}s morph, "
                      f"path clearance {plan.minimum_path_separation_m:.2f}m).",
                      formation=style, transition_id=tplan.transition_id,
                      duration_s=plan.duration_s,
                      minimum_path_separation_m=plan.minimum_path_separation_m)

    def _formation_transition_abort_reason(self, now: float) -> str | None:
        """Abort a morph if live telemetry loses the clearance assumed by its plan."""
        if self.formation_transition_deadline is None:
            return None
        if now > self.formation_transition_deadline + 12.0:
            return "formation did not settle within 12 s of the planned transition"
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        positions = []
        for index, state in enumerate(states):
            try:
                # Agents publish status once per second. Use the time this
                # operator received it, rather than the agent's clock-adjusted
                # timestamp, and allow one missed publication before aborting.
                status_age = now - float(state.get("operator_received_at", state.get("timestamp")))
                position_age = float(state.get("position_age_s"))
                position = np.asarray(state.get("position"), dtype=float)
            except (TypeError, ValueError):
                return f"D{index + 1} has invalid transition telemetry"
            if (not 0.0 <= status_age <= 2.0 or
                    not 0.0 <= position_age <= 1.0 or
                    status_age + position_age > 2.0):
                return f"D{index + 1} transition telemetry is stale"
            if state.get("airborne") is not True or state.get("armed") is not True:
                return f"D{index + 1} is no longer airborne and armed"
            if state.get("safety_state") != "nominal":
                return f"D{index + 1} reports {state.get('safety_state')} safety"
            if position.shape != (3,) or not np.isfinite(position).all():
                return f"D{index + 1} has invalid transition position"
            positions.append(position[:2])
        minimum = min(
            float(np.linalg.norm(positions[i] - positions[j]))
            for i in range(self.count) for j in range(i + 1, self.count)
        )
        if minimum < 2.8:
            return f"live separation fell to {minimum:.2f} m"
        return None

    def _formation_transition_arrived(self, now: float) -> bool:
        """All agents must actually reach their assigned slots before the morph ends."""
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        slots = np.asarray(self.formation_slots, dtype=float)
        if slots.shape != (self.count, 3):
            return False
        for index, state in enumerate(states):
            try:
                age = now - float(state.get("operator_received_at", state.get("timestamp")))
                position_age = float(state.get("position_age_s"))
                position = np.asarray(state.get("position"), dtype=float)
                progress = float(state.get("formation_transition_progress", 0.0))
            except (TypeError, ValueError):
                return False
            if (not 0.0 <= age <= 2.0 or not 0.0 <= position_age <= 1.0 or
                    position.shape != (3,) or not np.isfinite(position).all() or
                    state.get("formation") != self.formation or
                    state.get("formation_transition_active", True) or progress < 1.0 or
                    np.linalg.norm(position[:2] - self.goal_ne - slots[index, :2]) > 0.75):
                return False
        return True

    def _set_formation(self, style: str, heading_rad: float | None = None,
                       min_spacing_m: float | None = None) -> None:
        def result(accepted: bool, message: str, **details):
            self.last_formation_result = {
                "accepted": accepted, "message": message, **details,
            }
            print((">> " if accepted else ">> Formation change blocked: ") + message)
            return self.last_formation_result

        if style not in STYLES:
            return result(False, f"Unknown formation {style!r}; choose from {', '.join(STYLES)}.")
        if not getattr(self.args, "adaptive_swarm", False):
            return result(False, "Formation changes require adaptive swarm mode.")
        if self.route is not None:
            return result(False, "A route is active; stop it first (route stop) before changing formation.",
                          code="route_active")
        if self.mode_switch is not None and self.mode_switch.busy:
            return result(False, "A control-mode switch is in progress; wait for it to finish.",
                          code="mode_switch_active")
        if self.transition_protocol and self.transition.busy:
            return result(False, "A formation transition is already active; wait for it to finish.",
                          code="transition_active", transition_id=self.transition.plan.transition_id)
        if style == self.formation and not self.formation_hold:
            return result(True, f"Formation {style.upper()} is already active; no transition needed.",
                          formation=style, unchanged=True)
        if getattr(self, "formation_transition_deadline", None) is not None:
            return result(False, "A formation transition is already active; wait for it to finish.")
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        now = time.time()
        if any(not state.get("operator_received_at") or
               not 0.0 <= now - float(state.get("operator_received_at", 0.0)) <= 2.0
               for state in states):
            return result(False, "Every drone needs a fresh status heartbeat.")
        grounded = all(state.get("armed") is False and state.get("airborne") is False
                       for state in states)
        airborne = all(state.get("armed") is True and state.get("airborne") is True
                       for state in states)
        if not grounded and not airborne:
            return result(False, "All drones must be grounded or all airborne and armed.")

        if grounded:
            if any(not 0.0 <= float(state.get("position_age_s", float("inf"))) <= 1.0 or
                   state.get("safety_state") != "nominal"
                   for state in states):
                return result(False, "Every grounded drone needs fresh position telemetry and nominal safety status.")
            try:
                positions = np.asarray([state.get("position", [])[:2] for state in states], dtype=float)
            except (TypeError, ValueError):
                positions = np.asarray([], dtype=float)
            if positions.shape != (self.count, 2) or not np.isfinite(positions).all():
                return result(False, "Position reports are incomplete or invalid.")
            centroid = positions.mean(axis=0)
            try:
                plan = plan_formation_transition(positions, style, centroid_ne=centroid,
                                                 time_scale=self.time_scale,
                                                 position_sigma_m=self.position_sigma_m, heading_rad=heading_rad,
                                                 min_spacing_m=min_spacing_m)
            except (TypeError, ValueError) as exc:
                return result(False, str(exc))
            self.formation_resume_goal = self.goal_ne.copy()
            self.goal_ne = centroid.copy()
            self.formation = style
            self.formation_slots = plan.slots_by_drone
            self.formation_start_slots = None
            self.formation_transition_s = 0.0
            self._publish_intent()
            self.event("formation_command", formation=style, airborne=False,
                       minimum_path_separation_m=plan.minimum_path_separation_m)
            return result(True, f"Formation set to {style.upper()}; launch paths screen at "
                          f"{plan.minimum_path_separation_m:.2f} m clearance.",
                          formation=style, airborne=False,
                          minimum_path_separation_m=plan.minimum_path_separation_m)

        if any(not 0.0 <= float(state.get("position_age_s", float("inf"))) <= 1.0 or
               state.get("safety_state") != "nominal" for state in states):
            return result(False, "Every airborne drone needs fresh position telemetry and nominal safety status.")

        settled, worst, error = self._wait_settled()
        if not settled:
            return result(False, f"The swarm is still settling (D{worst + 1} is {error:.1f} m from its target); "
                          "try again in a few seconds.", code="settling")
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        positions = np.asarray([state["position"][:2] for state in states], dtype=float)
        if positions.shape != (self.count, 2) or not np.isfinite(positions).all():
            return result(False, "Position reports are incomplete or invalid.")
        stale = self._stale_link_drone(states)
        # A lossy radio drops links for a moment (live: one stale link refused a whole V re-form); wait a few seconds.
        for _ in range(int(self.LINK_RECOVERY_WAIT_S / 0.5)):
            if stale is None:
                break
            time.sleep(0.5)
            with self.lock:
                states = [self.latest.get(index, {}).copy() for index in range(self.count)]
            stale = self._stale_link_drone(states)
        if stale is not None:
            return result(False, f"D{stale + 1} has stale peer links.")
        obstacles = [tuple(float(value) for value in str(item).split(","))
                     for item in getattr(self.args, "obstacle", ())]
        centroid = positions.mean(axis=0)
        try:
            plan = plan_formation_transition(
                positions, style, centroid_ne=centroid, obstacles=obstacles, time_scale=self.time_scale,
                position_sigma_m=self.position_sigma_m, heading_rad=heading_rad, min_spacing_m=min_spacing_m,
            )
        except (TypeError, ValueError) as exc:
            return result(False, str(exc))
        problem = self._fence_problem(self._slot_points(centroid, slots=plan.slots_by_drone))
        if problem:
            return result(False, f"Geofence: {problem}", code="geofence")
        if self.transition_protocol:
            return self._begin_protocol_transition(style, plan, positions, centroid, now, result)
        self.formation_resume_goal = self.goal_ne.copy()
        self.goal_ne = centroid.copy()  # hold the swarm center during the morph
        self.formation = style
        self.formation_slots = plan.slots_by_drone
        self.formation_start_slots = np.column_stack((positions - centroid, np.zeros(self.count)))
        self.formation_transition_s = plan.duration_s
        self.formation_transition_deadline = now + plan.duration_s
        self.formation_transition_started_at = now
        self.formation_abort_issued = False
        self.formation_settled_since = None
        self._publish_intent()
        self.event("formation_transition_started", formation=style,
                   duration_s=plan.duration_s,
                   min_planned_separation_m=plan.minimum_path_separation_m,
                   target_slots=plan.slots_by_drone.tolist())
        message = (f"Smooth transition to {style.upper()} started over {plan.duration_s:.1f}s; "
                   f"planned path clearance {plan.minimum_path_separation_m:.2f}m. "
                   "The swarm center will resume its previous target when complete.")
        result(True, message, formation=style, duration_s=plan.duration_s,
               minimum_path_separation_m=plan.minimum_path_separation_m)

    SETTLE_TIMEOUT_S = 30.0

    # Pre-start cancellations that a lossy radio or a still-settling drone cause; retried automatically (nothing moved).
    TRANSIENT_ABORTS = ("peer_off_plan", "stale_peer_links", "ack_timeout", "late_acks", "start_off_plan")

    TRANSITION_RETRIES = 2

    LINK_RECOVERY_WAIT_S = 5.0

    @staticmethod
    def _stale_link_drone(states):
        """Index of the first drone whose expected peer links are not all live (None if every link is fine)."""
        for index, state in enumerate(states):
            expected = {int(peer) for peer in state.get("peer_ids", [])}
            live = {int(peer) for peer in state.get("peer_live_ids", [])}
            if expected - live:
                return index
        return None

    def _settle_errors(self):
        """Distance of every drone from its current position target (missing data counts as settled)."""
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        errors = []
        for state in states:
            position, target = state.get("position"), state.get("position_target_ne")
            try:
                error = float(np.linalg.norm(np.asarray(position[:2], float) - np.asarray(target, float)))
            except (TypeError, ValueError, IndexError):
                error = 0.0
            errors.append(error if math.isfinite(error) else 0.0)
        return errors

    def _wait_settled(self, timeout_s: float | None = None, poll_s: float = 0.5):
        """Wait until every drone is near its target before planning from measured positions.

        Live finding: 10 s after a 50 m wide LINE completed, the tail drone was still drifting into its slot, so a plan
        built from its position was 'off plan' by the time the start check ran and every drone refused to start.
        Returns (settled, worst drone index, its error in metres).
        """
        tolerance = 0.75 + self.position_sigma_m
        # Counted in polling steps, not clock readings: tests replace the clock with one that does not advance.
        steps = max(1, int((self.SETTLE_TIMEOUT_S / self.time_scale if timeout_s is None else timeout_s) / poll_s))
        for step in range(steps + 1):
            errors = self._settle_errors()
            worst = int(np.argmax(errors)) if errors else 0
            if not errors or errors[worst] <= tolerance:
                return True, worst, errors[worst] if errors else 0.0
            if step < steps:
                time.sleep(poll_s)
        return False, worst, errors[worst]
