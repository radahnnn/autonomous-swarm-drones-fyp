"""Pure state machines for synchronized formation transitions.

Implements docs/TRANSITION_PROTOCOL_SPEC.md: prepare -> ack -> go (common start) -> run -> settle,
with HOLD (freeze, never LAND) on any abort. No I/O, no wall clock: callers pass ``now`` in the
shared operator clock. Callers must authenticate messages and check the clock-sync epoch first.
Every handler validates a message completely before it changes any state.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from sitl.formation_library import MIN_SPACING

VERSION = 1
Action = Tuple[Any, ...]  # ("send", message) or ("event", name, data)


@dataclass(frozen=True)
class TransitionParams:
    lead_s: float = 4.0
    ack_timeout_s: float = 1.5
    prepare_retry_s: float = 0.5
    min_lead_s: float = 1.0
    go_late_tolerance_s: float = 0.25
    no_go_grace_s: float = 1.0
    start_tol_m: float = 1.0
    arrival_tol_m: float = 0.75
    dwell_s: float = 2.0
    max_status_age_s: float = 2.0
    abort_separation_m: float = 2.8
    local_hold_separation_m: float = 2.6
    peer_track_tol_m: float = 1.5
    peer_stale_s: float = 1.0
    overrun_s: float = 12.0
    coord_loss_s: float = 3.0
    hold_timeout_s: float = 20.0
    agent_start_check_s: float = 1.0
    arrived_close_s: float = 7.0


def _ease(alpha: float) -> float:
    """Quintic smoothstep (zero velocity and acceleration at both ends)."""
    alpha = min(1.0, max(0.0, alpha))
    return alpha ** 3 * (10.0 - 15.0 * alpha + 6.0 * alpha ** 2)


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _finite_array(value: Any, shape: Tuple[int, ...], name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} must be finite with shape {shape}")
    return array


@dataclass(frozen=True)
class TransitionPlan:
    transition_id: int
    formation: str
    centroid_ne: Tuple[float, float]
    start_slots: Tuple[Tuple[float, float, float], ...]
    target_slots: Tuple[Tuple[float, float, float], ...]
    duration_s: float
    start_at: float

    def canonical(self) -> Dict[str, Any]:
        return {"transition_id": self.transition_id, "formation": self.formation,
                "centroid_ne": list(self.centroid_ne),
                "start_slots": [list(s) for s in self.start_slots],
                "target_slots": [list(s) for s in self.target_slots],
                "duration_s": self.duration_s, "start_at": self.start_at}

    @property
    def plan_hash(self) -> str:
        blob = json.dumps(self.canonical(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()

    def to_prepare_message(self) -> Dict[str, Any]:
        message = {"version": VERSION, "type": "transition_prepare",
                   "plan_hash": self.plan_hash}
        message.update(self.canonical())
        return message

    @classmethod
    def build(cls, transition_id: int, formation: str, centroid_ne, start_slots,
              target_slots, duration_s: float, start_at: float) -> "TransitionPlan":
        """Validate and freeze a plan; raises ValueError for anything unsafe or malformed."""
        if not _is_int(transition_id) or transition_id < 1:
            raise ValueError("transition_id must be a positive integer")
        if not isinstance(formation, str) or not formation or len(formation) > 64:
            raise ValueError("invalid formation identifier")
        centroid = _finite_array(centroid_ne, (2,), "centroid_ne")
        target = np.asarray(target_slots, dtype=float) if target_slots is not None else None
        if target is None or target.ndim != 2 or target.shape[1] != 3 or not 1 <= len(target) <= 10:
            raise ValueError("target_slots must be 1-10 North/East/Down triples")
        target = _finite_array(target, target.shape, "target_slots")
        start = _finite_array(start_slots, target.shape, "start_slots")
        if not math.isfinite(duration_s) or not 0.1 <= duration_s <= 60.0:
            raise ValueError("duration_s must be between 0.1 and 60")
        if not math.isfinite(start_at):
            raise ValueError("start_at must be finite")
        for name, slots in (("start_slots", start), ("target_slots", target)):
            for i in range(len(slots)):
                for j in range(i + 1, len(slots)):
                    if np.linalg.norm(slots[i, :2] - slots[j, :2]) < MIN_SPACING - 1e-9:
                        raise ValueError(f"{name} violate the minimum horizontal spacing")
        return cls(int(transition_id), formation, (float(centroid[0]), float(centroid[1])),
                   tuple(tuple(float(c) for c in row) for row in start),
                   tuple(tuple(float(c) for c in row) for row in target),
                   float(duration_s), float(start_at))

    @classmethod
    def from_message(cls, message: Dict[str, Any]) -> "TransitionPlan":
        if not isinstance(message, dict) or message.get("version") != VERSION:
            raise ValueError("unsupported transition message")
        plan = cls.build(message.get("transition_id"), message.get("formation"),
                         message.get("centroid_ne"), message.get("start_slots"),
                         message.get("target_slots"), _num(message.get("duration_s")),
                         _num(message.get("start_at")))
        if message.get("plan_hash") != plan.plan_hash:
            raise ValueError("plan_hash does not match the plan contents")
        return plan

    @property
    def count(self) -> int:
        return len(self.target_slots)

    def alpha(self, now: float) -> float:
        return min(1.0, max(0.0, (now - self.start_at) / self.duration_s))

    def slots_at(self, alpha: float) -> np.ndarray:
        e = _ease(alpha)
        return (np.asarray(self.start_slots) * (1.0 - e) + np.asarray(self.target_slots) * e)

    def position_at(self, drone_id: int, alpha: float) -> np.ndarray:
        return np.asarray(self.centroid_ne) + self.slots_at(alpha)[drone_id, :2]


def _num(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected a number")
    return float(value)


# --------------------------------------------------------------------------- agent side

@dataclass
class PeerObservation:
    position_ne: Tuple[float, float]
    age_s: float


@dataclass
class AgentContext:
    """What the node knows at this instant (all in the operator clock domain)."""
    airborne_armed: bool = True
    own_fresh: bool = True
    peers_live: bool = True
    own_position_ne: Optional[Tuple[float, float]] = None
    peers: Dict[int, PeerObservation] = field(default_factory=dict)
    coordinator_age_s: float = 0.0


class AgentTransition:
    IDLE, STAGED, RUNNING, ARRIVED, HOLD = "IDLE", "STAGED", "RUNNING", "ARRIVED", "HOLD"

    def __init__(self, drone_id: int, count: int, params: TransitionParams = TransitionParams()):
        self.drone_id, self.count, self.params = drone_id, count, params
        self.state = self.IDLE
        self.plan: Optional[TransitionPlan] = None
        self.last_closed_id = 0
        self.hold_reason: Optional[str] = None
        self.frozen_alpha: Optional[float] = None

    # -- helpers
    def _ack(self, transition_id, plan_hash, ok: bool, reason: Optional[str],
             error: Optional[float] = None) -> Dict[str, Any]:
        return {"version": VERSION, "type": "transition_ack", "drone_id": self.drone_id,
                "transition_id": transition_id, "plan_hash": plan_hash, "ok": ok,
                "reason": reason, "own_position_error_m": error}

    def _matches(self, message: Dict[str, Any]) -> bool:
        return (self.plan is not None and message.get("transition_id") == self.plan.transition_id
                and message.get("plan_hash") == self.plan.plan_hash)

    def _close(self) -> None:
        if self.plan is not None:
            self.last_closed_id = max(self.last_closed_id, self.plan.transition_id)
        self.state, self.plan = self.IDLE, None
        self.hold_reason = self.frozen_alpha = None

    def _hold(self, reason: str, alpha: float) -> None:
        self.state, self.hold_reason, self.frozen_alpha = self.HOLD, reason, alpha

    # -- message handlers
    def on_prepare(self, message: Dict[str, Any], now: float,
                   ctx: AgentContext) -> Dict[str, Any]:
        transition_id = message.get("transition_id") if isinstance(message, dict) else None
        plan_hash = message.get("plan_hash") if isinstance(message, dict) else None
        try:
            plan = TransitionPlan.from_message(message)
        except ValueError:
            return self._ack(transition_id, plan_hash, False, "plan_invalid")
        if plan.count != self.count:
            return self._ack(transition_id, plan_hash, False, "plan_invalid")
        if plan.transition_id <= self.last_closed_id:
            return self._ack(transition_id, plan_hash, False, "stale_transition")
        if self.state in (self.STAGED, self.RUNNING, self.ARRIVED):
            if self._matches(message):
                return self._ack(transition_id, plan_hash, True, None)  # idempotent resend
            return self._ack(transition_id, plan_hash, False, "already_in_transition")
        if self.state == self.HOLD and self.plan is not None and plan.transition_id <= self.plan.transition_id:
            return self._ack(transition_id, plan_hash, False, "stale_transition")
        if not ctx.airborne_armed:
            return self._ack(transition_id, plan_hash, False, "not_airborne_armed")
        if not ctx.own_fresh or ctx.own_position_ne is None:
            return self._ack(transition_id, plan_hash, False, "stale_own_telemetry")
        if not ctx.peers_live:
            return self._ack(transition_id, plan_hash, False, "stale_peer_links")
        if plan.start_at - now < self.params.min_lead_s:
            return self._ack(transition_id, plan_hash, False, "start_too_soon")
        expected = plan.position_at(self.drone_id, 0.0)
        error = float(np.linalg.norm(np.asarray(ctx.own_position_ne, dtype=float) - expected))
        if error > self.params.start_tol_m:
            return self._ack(transition_id, plan_hash, False, "position_mismatch", error)
        self.plan, self.state = plan, self.STAGED
        self.hold_reason = self.frozen_alpha = None
        return self._ack(transition_id, plan_hash, True, None, error)

    def on_go(self, message: Dict[str, Any], now: float) -> Optional[Dict[str, Any]]:
        """Return a NACK-style reply only if a staged plan has to be dropped."""
        if not isinstance(message, dict) or self.state != self.STAGED or not self._matches(message):
            return None
        if message.get("start_at") != self.plan.start_at:
            return None
        if now > self.plan.start_at + self.params.go_late_tolerance_s:
            reply = self._ack(self.plan.transition_id, self.plan.plan_hash, False, "go_late")
            self._close()
            return reply
        self.state = self.RUNNING
        return None

    def on_abort(self, message: Dict[str, Any], now: float) -> None:
        if (not isinstance(message, dict) or self.plan is None or
                message.get("transition_id") != self.plan.transition_id):
            return
        reason = message.get("reason")
        reason = reason if isinstance(reason, str) else "abort"
        if self.state == self.STAGED:
            self._close()  # nothing moved
        elif self.state in (self.RUNNING, self.ARRIVED):
            freeze_at = message.get("freeze_at")
            freeze_at = float(freeze_at) if isinstance(freeze_at, (int, float)) and \
                not isinstance(freeze_at, bool) and math.isfinite(freeze_at) else now
            self._hold(f"abort:{reason}", self.plan.alpha(min(freeze_at, now)))

    def on_done(self, message: Dict[str, Any]) -> None:
        if isinstance(message, dict) and self.state == self.ARRIVED and self._matches(message):
            self._close()

    # -- periodic
    def tick(self, now: float, ctx: AgentContext) -> List[Action]:
        actions: List[Action] = []
        plan = self.plan
        if plan is None:
            return actions
        if self.state == self.STAGED:
            if now > plan.start_at + self.params.no_go_grace_s:
                actions.append(("event", "no_go", {"transition_id": plan.transition_id}))
                self._close()
            return actions
        if self.state not in (self.RUNNING, self.ARRIVED):
            return actions
        alpha = plan.alpha(now)
        if self.state == self.RUNNING and alpha >= 1.0:
            self.state = self.ARRIVED
        if (self.state == self.ARRIVED and
                now >= plan.start_at + plan.duration_s + self.params.arrived_close_s):
            actions.append(("event", "arrived_timeout_close", {"transition_id": plan.transition_id}))
            self._close()  # lost transition_done must not leave the agent in a transition forever
            return actions
        reason = self._local_violation(now, ctx, alpha)
        if reason is not None:
            self._hold(reason, alpha)
            actions.append(("send", {"version": VERSION, "type": "transition_abort",
                                     "transition_id": plan.transition_id, "reason": reason,
                                     "freeze_at": now, "from": self.drone_id}))
        return actions

    def _local_violation(self, now: float, ctx: AgentContext, alpha: float) -> Optional[str]:
        p = self.params
        if not ctx.airborne_armed or not ctx.own_fresh or ctx.own_position_ne is None:
            return "own_fault"
        if ctx.coordinator_age_s > p.coord_loss_s:
            return "coordinator_lost"
        own = np.asarray(ctx.own_position_ne, dtype=float)
        for peer_id in range(self.count):
            if peer_id == self.drone_id:
                continue
            obs = ctx.peers.get(peer_id)
            if obs is None or obs.age_s > p.peer_stale_s:
                return f"peer_stale:{peer_id}"
            position = np.asarray(obs.position_ne, dtype=float)
            if np.linalg.norm(position - self.plan.position_at(peer_id, alpha)) > p.peer_track_tol_m:
                return f"peer_off_plan:{peer_id}"
            if np.linalg.norm(position - own) < p.local_hold_separation_m:
                return f"separation:{peer_id}"
        return None

    # -- outputs
    def current_slots(self, now: float) -> Optional[np.ndarray]:
        """Slot table the guidance should use, or None when no transition owns the slots."""
        if self.plan is None:
            return None
        if self.state == self.STAGED:
            return np.asarray(self.plan.start_slots, dtype=float)
        if self.state == self.HOLD:
            return self.plan.slots_at(self.frozen_alpha or 0.0)
        return self.plan.slots_at(self.plan.alpha(now))

    def status(self, now: float) -> Dict[str, Any]:
        progress = None
        if self.plan is not None:
            progress = (self.frozen_alpha if self.state == self.HOLD else
                        0.0 if self.state == self.STAGED else self.plan.alpha(now))
        return {"state": self.state,
                "transition_id": None if self.plan is None else self.plan.transition_id,
                "progress": progress, "hold_reason": self.hold_reason}


# ----------------------------------------------------------------------- coordinator side

@dataclass
class AgentView:
    """Coordinator's latest knowledge of one agent."""
    age_s: float
    armed_airborne: bool = True
    safety_nominal: bool = True
    position_ne: Optional[Tuple[float, float]] = None
    state: str = "IDLE"
    transition_id: Optional[int] = None


class CoordinatorTransition:
    IDLE, PREPARING, COMMITTED, RUNNING, SETTLING, HOLDING = (
        "IDLE", "PREPARING", "COMMITTED", "RUNNING", "SETTLING", "HOLDING")

    def __init__(self, count: int, params: TransitionParams = TransitionParams()):
        self.count, self.params = count, params
        self.state = self.IDLE
        self.plan: Optional[TransitionPlan] = None
        self.acks: Dict[int, bool] = {}
        self.last_id = 0
        self.abort_reason: Optional[str] = None
        self._prepared_at = self._last_send = self._settled_since = self._held_at = 0.0
        self._settled_since_set = False
        self._needs_operator_sent = False

    @property
    def busy(self) -> bool:
        """True while any command that could conflict must be rejected."""
        return self.state in (self.PREPARING, self.COMMITTED, self.RUNNING, self.SETTLING)

    def next_id(self) -> int:
        return self.last_id + 1

    def begin(self, plan: TransitionPlan, now: float) -> List[Action]:
        if self.busy:
            raise RuntimeError("transition_active")
        if plan.count != self.count or plan.transition_id <= self.last_id:
            raise ValueError("plan does not match the swarm or is not newer")
        self.plan, self.state = plan, self.PREPARING
        self.last_id = plan.transition_id
        self.acks, self.abort_reason = {}, None
        self._prepared_at = self._last_send = now
        self._settled_since_set = self._needs_operator_sent = False
        return [("send", plan.to_prepare_message()),
                ("event", "transition_prepare", {"transition_id": plan.transition_id})]

    def abort(self, reason: str, now: float) -> List[Action]:
        """Operator or supervision abort. Pre-start: nothing moved. Post-start: HOLD."""
        if self.plan is None or self.state in (self.IDLE, self.HOLDING):
            return []
        moved = self.state in (self.RUNNING, self.SETTLING) or now >= self.plan.start_at
        self.abort_reason = reason
        self.state = self.HOLDING if moved else self.IDLE
        self._held_at = now
        return [("send", {"version": VERSION, "type": "transition_abort",
                          "transition_id": self.plan.transition_id, "reason": reason,
                          "freeze_at": now}),
                ("event", "transition_aborted", {"transition_id": self.plan.transition_id,
                                                 "reason": reason, "moved": moved})]

    def on_ack(self, message: Dict[str, Any], now: float) -> List[Action]:
        if (self.plan is None or not isinstance(message, dict) or
                message.get("transition_id") != self.plan.transition_id or
                message.get("plan_hash") != self.plan.plan_hash):
            return []
        drone_id = message.get("drone_id")
        if not _is_int(drone_id) or not 0 <= drone_id < self.count or \
                not isinstance(message.get("ok"), bool):
            return []
        if not message["ok"]:
            if self.state in (self.PREPARING, self.COMMITTED, self.RUNNING):
                return self.abort(f"nack:{drone_id}:{message.get('reason')}", now)
            return []
        if self.state == self.PREPARING:
            self.acks[drone_id] = True
        return []

    def on_agent_abort(self, message: Dict[str, Any], now: float) -> List[Action]:
        if (self.plan is None or not isinstance(message, dict) or
                message.get("transition_id") != self.plan.transition_id):
            return []
        return self.abort(f"agent_abort:{message.get('from')}:{message.get('reason')}", now)

    def tick(self, now: float, views: Dict[int, AgentView]) -> List[Action]:
        p, plan = self.params, self.plan
        if plan is None:
            return []
        if self.state == self.PREPARING:
            if len(self.acks) == self.count:
                if plan.start_at - now < p.min_lead_s:
                    return self.abort("late_acks", now)
                self.state, self._last_send = self.COMMITTED, now
                return [("send", self._go()), ("event", "transition_committed",
                                               {"transition_id": plan.transition_id})]
            if now - self._prepared_at > p.ack_timeout_s:
                missing = sorted(set(range(self.count)) - set(self.acks))
                return self.abort(f"ack_timeout:{missing}", now)
            if now - self._last_send >= p.prepare_retry_s:
                self._last_send = now
                return [("send", plan.to_prepare_message())]
            return []
        if self.state == self.COMMITTED:
            if now >= plan.start_at:
                self.state = self.RUNNING
                return [("event", "transition_started", {"transition_id": plan.transition_id})]
            if now - self._last_send >= p.prepare_retry_s:
                self._last_send = now
                return [("send", self._go())]
            return []
        if self.state in (self.RUNNING, self.SETTLING):
            reason = self._supervision_violation(now, views)
            if reason is not None:
                return self.abort(reason, now)
            if self.state == self.RUNNING and now >= plan.start_at + plan.duration_s:
                self.state = self.SETTLING
            if self.state == self.SETTLING:
                return self._settle(now, views)
            return []
        if self.state == self.HOLDING and not self._needs_operator_sent and \
                now - self._held_at > p.hold_timeout_s:
            self._needs_operator_sent = True
            return [("event", "needs_operator", {"transition_id": plan.transition_id,
                                                 "reason": self.abort_reason})]
        return []

    def _go(self) -> Dict[str, Any]:
        return {"version": VERSION, "type": "transition_go",
                "transition_id": self.plan.transition_id, "plan_hash": self.plan.plan_hash,
                "start_at": self.plan.start_at}

    def _supervision_violation(self, now: float, views: Dict[int, AgentView]) -> Optional[str]:
        p, plan = self.params, self.plan
        positions = []
        for i in range(self.count):
            view = views.get(i)
            if view is None or not 0.0 <= view.age_s <= p.max_status_age_s:
                return f"stale_status:{i}"
            if not view.armed_airborne:
                return f"not_airborne:{i}"
            if not view.safety_nominal:
                return f"unsafe:{i}"
            if now > plan.start_at + p.agent_start_check_s and (
                    view.transition_id != plan.transition_id or
                    view.state not in (AgentTransition.RUNNING, AgentTransition.ARRIVED)):
                return f"agent_not_running:{i}"
            if view.position_ne is None:
                return f"no_position:{i}"
            positions.append(np.asarray(view.position_ne, dtype=float))
        for i in range(self.count):
            for j in range(i + 1, self.count):
                if np.linalg.norm(positions[i] - positions[j]) < p.abort_separation_m:
                    return f"separation:{i},{j}"
        if now > plan.start_at + plan.duration_s + p.overrun_s:
            return "overrun"
        return None

    def _settle(self, now: float, views: Dict[int, AgentView]) -> List[Action]:
        p, plan = self.params, self.plan
        arrived = True
        for i in range(self.count):
            view = views[i]
            target = plan.position_at(i, 1.0)
            if (view.state != AgentTransition.ARRIVED or
                    np.linalg.norm(np.asarray(view.position_ne, dtype=float) - target) > p.arrival_tol_m):
                arrived = False
                break
        if not arrived:
            self._settled_since_set = False
            return []
        if not self._settled_since_set:
            self._settled_since, self._settled_since_set = now, True
        if now - self._settled_since < p.dwell_s:
            return []
        done = {"version": VERSION, "type": "transition_done",
                "transition_id": plan.transition_id, "plan_hash": plan.plan_hash}
        self.state = self.IDLE
        return [("send", done), ("event", "transition_complete",
                                 {"transition_id": plan.transition_id})]
