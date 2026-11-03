"""Pure state machines for synchronized in-flight control-mode switching.

The swarm changes between ``centralized``, ``decentralized`` and ``hybrid`` together:

  coordinator: IDLE -> PREPARING -> COMMITTED -> SWITCHING -> VERIFYING -> IDLE
                                  any failure -> ROLLING_BACK -> IDLE
  agent:       IDLE -> STAGED -> SCHEDULED -> SWITCHING -> VERIFY -> IDLE
                                  any failure -> ROLLBACK -> IDLE (old mode restored)

Every agent acknowledges the plan before anyone switches; all agents start the blend at one common
time and ramp with a smooth weight, so the control law is a blend of the outgoing and incoming
controllers (bumpless transfer). An agent keeps the right to roll back until the coordinator has
verified that every agent reports the new mode (``done``) or a self-close timeout expires.

No I/O and no wall clock: callers pass ``now`` in the shared operator clock. Handlers validate a
message completely before changing state (same rule as ``sitl/transition_protocol.py``).
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

VERSION = 1
CONTROL_MODES = ("centralized", "decentralized", "hybrid")
Action = Tuple[Any, ...]  # ("send", message) or ("event", name, data)


@dataclass(frozen=True)
class ModeParams:
    lead_s: float = 3.0
    ack_timeout_s: float = 1.5
    prepare_retry_s: float = 0.5
    min_lead_s: float = 1.0
    go_late_tolerance_s: float = 0.25
    no_go_grace_s: float = 1.0
    ramp_s: float = 2.0
    verify_dwell_s: float = 1.0
    verify_close_s: float = 10.0    # agent self-commits this long after the ramp if the coordinator is silent
    verify_timeout_s: float = 4.0   # coordinator gives up verifying (and rolls back) this long after the ramp;
                                    # must stay well below verify_close_s so an abort can always land first
    max_status_age_s: float = 2.0
    min_separation_abort_m: float = 2.6
    overrun_s: float = 8.0
    agent_start_check_s: float = 1.0


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _num(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected a number")
    return float(value)


def smooth_weight(progress: float) -> float:
    """Cubic smoothstep: zero slope at both ends so the blend adds no step in the command."""
    p = min(1.0, max(0.0, progress))
    return p * p * (3.0 - 2.0 * p)


@dataclass(frozen=True)
class ModePlan:
    switch_id: int
    from_mode: str
    to_mode: str
    start_at: float
    ramp_s: float

    def canonical(self) -> Dict[str, Any]:
        return {"switch_id": self.switch_id, "from_mode": self.from_mode,
                "to_mode": self.to_mode, "start_at": self.start_at, "ramp_s": self.ramp_s}

    @property
    def plan_hash(self) -> str:
        blob = json.dumps(self.canonical(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()

    def to_prepare_message(self) -> Dict[str, Any]:
        message = {"version": VERSION, "type": "mode_prepare", "plan_hash": self.plan_hash}
        message.update(self.canonical())
        return message

    @classmethod
    def build(cls, switch_id, from_mode, to_mode, start_at, ramp_s) -> "ModePlan":
        if not _is_int(switch_id) or switch_id < 1:
            raise ValueError("switch_id must be a positive integer")
        if from_mode not in CONTROL_MODES or to_mode not in CONTROL_MODES:
            raise ValueError("unknown control mode")
        if from_mode == to_mode:
            raise ValueError("source and target mode are the same")
        start_at, ramp_s = _num(start_at), _num(ramp_s)
        if not math.isfinite(start_at) or not math.isfinite(ramp_s) or not 0.2 <= ramp_s <= 10.0:
            raise ValueError("start time must be finite and the ramp between 0.2 and 10 s")
        return cls(int(switch_id), from_mode, to_mode, start_at, ramp_s)

    @classmethod
    def from_message(cls, message: Dict[str, Any]) -> "ModePlan":
        if not isinstance(message, dict) or message.get("version") != VERSION:
            raise ValueError("unsupported mode message")
        plan = cls.build(message.get("switch_id"), message.get("from_mode"), message.get("to_mode"),
                         message.get("start_at"), message.get("ramp_s"))
        if message.get("plan_hash") != plan.plan_hash:
            raise ValueError("plan_hash does not match the plan contents")
        return plan

    def weight(self, now: float) -> float:
        return smooth_weight((now - self.start_at) / self.ramp_s)


# --------------------------------------------------------------------------- agent side

@dataclass
class ModeContext:
    healthy: bool = True          # armed/airborne as required, own telemetry fresh
    formation_idle: bool = True   # no formation transition in progress


class AgentModeSwitch:
    IDLE, STAGED, SCHEDULED, SWITCHING, VERIFY, ROLLBACK = (
        "IDLE", "STAGED", "SCHEDULED", "SWITCHING", "VERIFY", "ROLLBACK")

    def __init__(self, drone_id: int, initial_mode: str, params: ModeParams = ModeParams()):
        if initial_mode not in CONTROL_MODES:
            raise ValueError("unknown control mode")
        self.drone_id, self.params = drone_id, params
        self.mode = initial_mode            # the committed mode when no switch is active
        self.state = self.IDLE
        self.plan: Optional[ModePlan] = None
        self.last_closed_id = 0
        self._rollback_from_w = 0.0
        self._rollback_started = 0.0

    # -- helpers
    def _ack(self, switch_id, plan_hash, ok: bool, reason: Optional[str]) -> Dict[str, Any]:
        return {"version": VERSION, "type": "mode_ack", "drone_id": self.drone_id,
                "switch_id": switch_id, "plan_hash": plan_hash, "ok": ok, "reason": reason}

    def _matches(self, message: Dict[str, Any]) -> bool:
        return (self.plan is not None and message.get("switch_id") == self.plan.switch_id
                and message.get("plan_hash") == self.plan.plan_hash)

    def _close(self, new_mode: Optional[str] = None) -> None:
        if self.plan is not None:
            self.last_closed_id = max(self.last_closed_id, self.plan.switch_id)
        if new_mode is not None:
            self.mode = new_mode
        self.state, self.plan = self.IDLE, None

    # -- handlers
    def on_prepare(self, message: Dict[str, Any], now: float, ctx: ModeContext) -> Dict[str, Any]:
        switch_id = message.get("switch_id") if isinstance(message, dict) else None
        plan_hash = message.get("plan_hash") if isinstance(message, dict) else None
        try:
            plan = ModePlan.from_message(message)
        except ValueError:
            return self._ack(switch_id, plan_hash, False, "plan_invalid")
        if plan.switch_id <= self.last_closed_id:
            return self._ack(switch_id, plan_hash, False, "stale_switch")
        if self.state != self.IDLE:
            if self._matches(message):
                return self._ack(switch_id, plan_hash, True, None)   # idempotent resend
            return self._ack(switch_id, plan_hash, False, "already_switching")
        if plan.from_mode != self.mode:
            return self._ack(switch_id, plan_hash, False, "mode_mismatch")
        if not ctx.healthy:
            return self._ack(switch_id, plan_hash, False, "unhealthy")
        if not ctx.formation_idle:
            return self._ack(switch_id, plan_hash, False, "formation_transition_active")
        if plan.start_at - now < self.params.min_lead_s:
            return self._ack(switch_id, plan_hash, False, "start_too_soon")
        self.plan, self.state = plan, self.STAGED
        return self._ack(switch_id, plan_hash, True, None)

    def on_go(self, message: Dict[str, Any], now: float) -> Optional[Dict[str, Any]]:
        if not isinstance(message, dict) or self.state != self.STAGED or not self._matches(message):
            return None
        if message.get("start_at") != self.plan.start_at:
            return None
        if now > self.plan.start_at + self.params.go_late_tolerance_s:
            reply = self._ack(self.plan.switch_id, self.plan.plan_hash, False, "go_late")
            self._close()
            return reply
        self.state = self.SCHEDULED
        return None

    def on_abort(self, message: Dict[str, Any], now: float) -> None:
        if (not isinstance(message, dict) or self.plan is None or
                message.get("switch_id") != self.plan.switch_id):
            return
        if self.state in (self.STAGED, self.SCHEDULED):
            self._close()                       # nothing changed yet: discard the plan
        elif self.state in (self.SWITCHING, self.VERIFY):
            self._begin_rollback(now)

    def on_done(self, message: Dict[str, Any]) -> None:
        if isinstance(message, dict) and self.state == self.VERIFY and self._matches(message):
            self._close(self.plan.to_mode)

    def _begin_rollback(self, now: float) -> None:
        self._rollback_from_w = self.plan.weight(now) if self.state == self.SWITCHING else 1.0
        self._rollback_started = now
        self.state = self.ROLLBACK

    # -- periodic
    def tick(self, now: float, ctx: ModeContext) -> List[Action]:
        actions: List[Action] = []
        plan = self.plan
        if plan is None:
            return actions
        p = self.params
        if self.state == self.STAGED:
            if now > plan.start_at + p.no_go_grace_s:
                actions.append(("event", "mode_no_go", {"switch_id": plan.switch_id}))
                self._close()
            return actions
        if self.state == self.SCHEDULED and now >= plan.start_at:
            self.state = self.SWITCHING
        if self.state == self.SWITCHING:
            if not ctx.healthy or not ctx.formation_idle:
                reason = "own_fault" if not ctx.healthy else "formation_transition_started"
                actions.append(("send", {"version": VERSION, "type": "mode_abort",
                                         "switch_id": plan.switch_id, "reason": reason,
                                         "from": self.drone_id}))
                self._begin_rollback(now)
            elif now >= plan.start_at + plan.ramp_s:
                self.state = self.VERIFY
        if self.state == self.VERIFY and now >= plan.start_at + plan.ramp_s + p.verify_close_s:
            actions.append(("event", "mode_verify_timeout_commit", {"switch_id": plan.switch_id}))
            self._close(plan.to_mode)
        elif self.state == self.ROLLBACK:
            duration = max(0.5, plan.ramp_s * self._rollback_from_w)
            if now - self._rollback_started >= duration:
                actions.append(("event", "mode_rolled_back", {"switch_id": plan.switch_id}))
                self._close(plan.from_mode)
        return actions

    # -- outputs
    def weights(self, now: float) -> Tuple[str, str, float]:
        """(outgoing mode, incoming mode, weight of the incoming mode)."""
        plan = self.plan
        if plan is None or self.state in (self.IDLE, self.STAGED, self.SCHEDULED):
            return self.mode, self.mode, 1.0
        if self.state == self.SWITCHING:
            return plan.from_mode, plan.to_mode, plan.weight(now)
        if self.state == self.VERIFY:
            return plan.to_mode, plan.to_mode, 1.0
        progress = (now - self._rollback_started) / max(0.5, plan.ramp_s * self._rollback_from_w)
        return plan.from_mode, plan.to_mode, self._rollback_from_w * (1.0 - smooth_weight(progress))

    def dominant_mode(self, now: float) -> str:
        a, b, w = self.weights(now)
        return b if w >= 0.5 else a

    def status(self, now: float) -> Dict[str, Any]:
        a, b, w = self.weights(now)
        return {"mode": self.dominant_mode(now), "committed_mode": self.mode, "state": self.state,
                "switch_id": None if self.plan is None else self.plan.switch_id,
                "weight": round(w, 3), "target_mode": None if self.plan is None else self.plan.to_mode}


# ----------------------------------------------------------------------- coordinator side

@dataclass
class ModeView:
    age_s: float
    mode: str
    state: str = "IDLE"
    switch_id: Optional[int] = None
    healthy: bool = True
    position_ne: Optional[Tuple[float, float]] = None


class CoordinatorModeSwitch:
    IDLE, PREPARING, COMMITTED, SWITCHING, VERIFYING, ROLLING_BACK = (
        "IDLE", "PREPARING", "COMMITTED", "SWITCHING", "VERIFYING", "ROLLING_BACK")

    def __init__(self, count: int, initial_mode: str, params: ModeParams = ModeParams()):
        if initial_mode not in CONTROL_MODES:
            raise ValueError("unknown control mode")
        self.count, self.params = count, params
        self.mode = initial_mode
        self.state = self.IDLE
        self.plan: Optional[ModePlan] = None
        self.acks: Dict[int, bool] = {}
        self.last_id = 0
        self.abort_reason: Optional[str] = None
        self._prepared_at = self._last_send = self._verified_since = self._rolled_since = 0.0
        self._verified_set = False
        self._done_sent, self._done_at = False, 0.0

    @property
    def busy(self) -> bool:
        return self.state != self.IDLE

    def next_id(self) -> int:
        return self.last_id + 1

    def begin(self, to_mode: str, now: float) -> List[Action]:
        if self.busy:
            raise RuntimeError("mode_switch_active")
        plan = ModePlan.build(self.next_id(), self.mode, to_mode,
                              now + self.params.lead_s, self.params.ramp_s)
        self.plan, self.state, self.last_id = plan, self.PREPARING, plan.switch_id
        self.acks, self.abort_reason, self._verified_set = {}, None, False
        self._done_sent, self._done_at = False, 0.0
        self._prepared_at = self._last_send = now
        return [("send", plan.to_prepare_message()),
                ("event", "mode_prepare", {"switch_id": plan.switch_id, "from_mode": plan.from_mode,
                                           "to_mode": plan.to_mode})]

    def abort(self, reason: str, now: float) -> List[Action]:
        if self.plan is None or self.state in (self.IDLE, self.ROLLING_BACK):
            return []
        if self._done_sent:
            # Irreversible commit (Codex review #1): once DONE may have reached any agent, a rollback would split the
            # fleet between modes. Keep driving everyone to the new mode instead.
            return [("event", "mode_abort_ignored_after_commit", {"switch_id": self.plan.switch_id, "reason": reason})]
        started = self.state in (self.SWITCHING, self.VERIFYING) or now >= self.plan.start_at
        self.abort_reason = reason
        self.state = self.ROLLING_BACK if started else self.IDLE
        self._rolled_since, self._verified_set, self._last_send = now, False, now
        return [("send", self._abort_message(reason)),
                ("event", "mode_switch_aborted", {"switch_id": self.plan.switch_id,
                                                  "reason": reason, "started": started})]

    def on_ack(self, message: Dict[str, Any], now: float) -> List[Action]:
        if (self.plan is None or not isinstance(message, dict) or
                message.get("switch_id") != self.plan.switch_id or
                message.get("plan_hash") != self.plan.plan_hash):
            return []
        drone_id = message.get("drone_id")
        if not _is_int(drone_id) or not 0 <= drone_id < self.count or \
                not isinstance(message.get("ok"), bool):
            return []
        if not message["ok"]:
            if self.state in (self.PREPARING, self.COMMITTED, self.SWITCHING):
                return self.abort(f"nack:{drone_id}:{message.get('reason')}", now)
            return []
        if self.state == self.PREPARING:
            self.acks[drone_id] = True
        return []

    def on_agent_abort(self, message: Dict[str, Any], now: float) -> List[Action]:
        if (self.plan is None or not isinstance(message, dict) or
                message.get("switch_id") != self.plan.switch_id):
            return []
        return self.abort(f"agent_abort:{message.get('from')}:{message.get('reason')}", now)

    def _abort_message(self, reason: Optional[str]) -> Dict[str, Any]:
        return {"version": VERSION, "type": "mode_abort", "switch_id": self.plan.switch_id,
                "reason": reason}

    def _go(self) -> Dict[str, Any]:
        return {"version": VERSION, "type": "mode_go", "switch_id": self.plan.switch_id,
                "plan_hash": self.plan.plan_hash, "start_at": self.plan.start_at}

    def tick(self, now: float, views: Dict[int, ModeView]) -> List[Action]:
        p, plan = self.params, self.plan
        if plan is None:
            return []
        if self.state == self.PREPARING:
            if len(self.acks) == self.count:
                if plan.start_at - now < p.min_lead_s:
                    return self.abort("late_acks", now)
                self.state, self._last_send = self.COMMITTED, now
                return [("send", self._go()),
                        ("event", "mode_committed", {"switch_id": plan.switch_id})]
            if now - self._prepared_at > p.ack_timeout_s:
                missing = sorted(set(range(self.count)) - set(self.acks))
                return self.abort(f"ack_timeout:{missing}", now)
            if now - self._last_send >= p.prepare_retry_s:
                self._last_send = now
                return [("send", plan.to_prepare_message())]
            return []
        if self.state == self.COMMITTED:
            if now >= plan.start_at:
                self.state = self.SWITCHING
                return [("event", "mode_switch_started", {"switch_id": plan.switch_id})]
            if now - self._last_send >= p.prepare_retry_s:
                self._last_send = now
                return [("send", self._go())]
            return []
        if self.state == self.VERIFYING and self._done_sent:
            return self._verify(now, views)        # committed: only confirm, never abort (see abort())
        if self.state in (self.SWITCHING, self.VERIFYING):
            reason = self._violation(now, views)
            if reason is not None:
                return self.abort(reason, now)
            if self.state == self.SWITCHING and now >= plan.start_at + plan.ramp_s:
                self.state = self.VERIFYING
            if self.state == self.VERIFYING:
                return self._verify(now, views)
            return []
        if self.state == self.ROLLING_BACK:
            if len(views) >= self.count and all(
                    v.mode == plan.from_mode and v.state == AgentModeSwitch.IDLE for v in views.values()):
                self.state = self.IDLE
                return [("event", "mode_switch_rolled_back", {"switch_id": plan.switch_id,
                                                              "reason": self.abort_reason})]
            if now - self._rolled_since > p.overrun_s * 2:
                self.state = self.IDLE
                return [("event", "mode_rollback_unverified", {"switch_id": plan.switch_id})]
            if now - self._last_send >= p.prepare_retry_s:
                self._last_send = now          # keep telling agents until they all confirm
                return [("send", self._abort_message(self.abort_reason))]
        return []

    def _violation(self, now: float, views: Dict[int, ModeView]) -> Optional[str]:
        p, plan = self.params, self.plan
        positions = []
        for i in range(self.count):
            view = views.get(i)
            if view is None or not 0.0 <= view.age_s <= p.max_status_age_s:
                return f"stale_status:{i}"
            if not view.healthy:
                return f"unhealthy:{i}"
            if now > plan.start_at + p.agent_start_check_s and (
                    view.switch_id != plan.switch_id and view.mode != plan.to_mode):
                return f"agent_not_switching:{i}"
            if view.position_ne is not None:
                positions.append(np.asarray(view.position_ne, dtype=float))
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                if np.linalg.norm(positions[i] - positions[j]) < p.min_separation_abort_m:
                    return f"separation:{i},{j}"
        if now > plan.start_at + plan.ramp_s + p.verify_timeout_s:
            return "verify_timeout"
        return None

    def _verify(self, now: float, views: Dict[int, ModeView]) -> List[Action]:
        plan = self.plan
        if self._done_sent:
            # `done` is retransmitted until every agent has closed the switch (state IDLE, new mode). Agents that never
            # hear it self-commit to the new mode after verify_close_s, so the fleet converges either way.
            if all(i in views and views[i].mode == plan.to_mode and views[i].state == AgentModeSwitch.IDLE
                   for i in range(self.count)):
                self.mode, self.state = plan.to_mode, self.IDLE
                return [("event", "mode_switch_complete",
                         {"switch_id": plan.switch_id, "mode": plan.to_mode})]
            if now - self._done_at > self.params.verify_close_s + self.params.overrun_s:
                self.mode, self.state = plan.to_mode, self.IDLE
                return [("event", "mode_commit_unconfirmed", {"switch_id": plan.switch_id, "mode": plan.to_mode})]
            if now - self._last_send >= self.params.prepare_retry_s:
                self._last_send = now
                return [("send", self._done())]
            return []
        ok = all(views[i].mode == plan.to_mode and views[i].state in
                 (AgentModeSwitch.VERIFY, AgentModeSwitch.IDLE) for i in range(self.count))
        if not ok:
            self._verified_set = False
            return []
        if not self._verified_set:
            self._verified_since, self._verified_set = now, True
        if now - self._verified_since < self.params.verify_dwell_s:
            return []
        self._done_sent, self._last_send, self._done_at = True, now, now
        return [("send", self._done())]

    def _done(self) -> Dict[str, Any]:
        return {"version": VERSION, "type": "mode_done", "switch_id": self.plan.switch_id,
                "plan_hash": self.plan.plan_hash}
