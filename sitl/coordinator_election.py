"""Acting-coordinator failover for the independent swarm (pure logic, no I/O).

The operator console is the swarm's coordinator: it publishes the mission intent about once a second.
If that stream goes silent, the drone that the swarm already agrees is its leader (lowest live ID, from
``sitl/adaptive_membership.py``) becomes the **acting coordinator**: it republishes the last mission
(including its route) to its peers so centralized and hybrid drones see a coordinator again.

This is a term-numbered, lease-based takeover, *not* a full Raft vote:

* A takeover increments the **term**. Among acting coordinators, the highest term wins; at equal terms
  the lower drone ID wins. This resolves a partitioned swarm deterministically when it heals.
* The **operator always wins**: any operator mission message ends every acting role immediately.
* The acting role expires after ``acting_max_s``; after that the normal coordinator-loss policies
  (hold then land for centralized, autonomy window for decentralized/hybrid) take over again.
* There is no vote round, so under a partition each side may elect its own acting coordinator.

Times are monotonic seconds supplied by the caller.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

Action = Tuple[Any, ...]


@dataclass(frozen=True)
class ElectionParams:
    election_timeout_s: float = 3.0     # operator silence before a takeover is considered
    publish_period_s: float = 1.0       # acting coordinator republishes the mission this often
    acting_fresh_s: float = 2.5         # an acting coordinator heard this recently counts as present
    acting_max_s: float = 300.0         # acting role expires after this long


class CoordinatorElection:
    FOLLOWER, ACTING = "FOLLOWER", "ACTING"

    def __init__(self, own_id: int, params: ElectionParams = ElectionParams()):
        self.own_id, self.params = int(own_id), params
        self.state = self.FOLLOWER
        self.term = 0                                 # highest term seen or used
        self.leader: Optional[Tuple[int, int]] = None # (term, drone id) of the acting coordinator we follow
        self.last_operator_at: Optional[float] = None
        self.last_acting_at: Optional[float] = None
        self.acting_started_at: Optional[float] = None
        self._last_publish = float("-inf")
        self.takeovers = 0
        # The acting period is a hard limit per operator-loss episode (Codex review #10): once used up, no new takeover
        # until a real operator message arrives.
        self.exhausted = False

    # ------------------------------------------------------------------ inputs
    def note_operator(self, now: float) -> List[Action]:
        """An operator mission message arrived: the operator is back (or never left)."""
        self.last_operator_at = now
        self.exhausted = False
        actions: List[Action] = []
        if self.state == self.ACTING:
            actions.append(("event", "acting_coordinator_stepped_down",
                            {"reason": "operator_returned", "term": self.term}))
            self.state, self.acting_started_at = self.FOLLOWER, None
        self.leader, self.last_acting_at = None, None
        return actions

    def on_acting_message(self, sender: int, term: int, now: float) -> Tuple[bool, List[Action]]:
        """Decide whether to follow an acting coordinator's message. Returns (accept, actions)."""
        if (isinstance(sender, bool) or not isinstance(sender, int) or isinstance(term, bool) or
                not isinstance(term, int) or term < 1 or sender == self.own_id):
            return False, []
        if self.last_operator_at is not None and not self.operator_silent(now):
            return False, []                          # the operator is present and always wins
        if self.leader is not None and (term, -sender) < (self.leader[0], -self.leader[1]):
            return False, []                          # a worse claim than the one we follow
        actions: List[Action] = []
        if self.state == self.ACTING and (term, -sender) > (self.term, -self.own_id):
            actions.append(("event", "acting_coordinator_stepped_down",
                            {"reason": f"higher_claim:D{sender + 1}", "term": term}))
            self.state, self.acting_started_at = self.FOLLOWER, None
        elif self.state == self.ACTING:
            return False, actions                     # we outrank the sender
        self.leader, self.last_acting_at = (term, sender), now
        self.term = max(self.term, term)
        return True, actions

    def resume_after_local_pause(self, now: float) -> None:
        """This agent was blocked (arming, takeoff, a stalled loop) and was not listening: silence proves
        nothing, so restart the operator and acting-coordinator silence timers instead of taking over."""
        if self.last_operator_at is not None:
            self.last_operator_at = now
        if self.last_acting_at is not None:
            self.last_acting_at = now

    # ---------------------------------------------------------------- periodic
    def operator_silent(self, now: float) -> bool:
        return (self.last_operator_at is not None and
                now - self.last_operator_at > self.params.election_timeout_s)

    def acting_present(self, now: float) -> bool:
        return (self.last_acting_at is not None and
                now - self.last_acting_at <= self.params.acting_fresh_s)

    def tick(self, now: float, elected_leader_id: Optional[int], live_peer_count: int) -> List[Action]:
        """Possibly take over, keep publishing, or expire. Actions: ("publish", term) / ("event", ...)."""
        actions: List[Action] = []
        p = self.params
        if self.state == self.FOLLOWER:
            if (self.operator_silent(now) and elected_leader_id == self.own_id and live_peer_count >= 1
                    and not self.acting_present(now) and not self.exhausted):
                self.term += 1
                self.state, self.acting_started_at, self._last_publish = self.ACTING, now, float("-inf")
                self.takeovers += 1
                actions.append(("event", "acting_coordinator_started",
                                {"term": self.term, "drone": self.own_id + 1}))
            else:
                return actions
        if self.state == self.ACTING:
            if now - (self.acting_started_at or now) > p.acting_max_s:
                actions.append(("event", "acting_coordinator_expired", {"term": self.term}))
                self.state, self.acting_started_at = self.FOLLOWER, None
                self.exhausted = True                 # no renewal until the operator really returns
                return actions
            if now - self._last_publish >= p.publish_period_s:
                self._last_publish = now
                actions.append(("publish", self.term))
        return actions

    # ----------------------------------------------------------------- outputs
    def source(self, now: float) -> str:
        """Who the agent currently follows: operator | acting:<id> | self | none."""
        if self.state == self.ACTING:
            return "self"
        if self.last_operator_at is not None and not self.operator_silent(now):
            return "operator"
        if self.acting_present(now) and self.leader is not None:
            return f"acting:{self.leader[1]}"
        return "none"

    def status(self, now: float) -> Dict[str, Any]:
        return {"source": self.source(now), "term": self.term, "acting": self.state == self.ACTING,
                "leader": None if self.leader is None else self.leader[1],
                "takeovers": self.takeovers}
