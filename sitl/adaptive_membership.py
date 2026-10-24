"""Local, deterministic membership and topology repair for independent swarm nodes."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from sitl.formation_library import slots_for


class AdaptiveMembership:
    """Track live agents and derive the same ring topology from a shared live view.

    Discovery uses authenticated liveness packets.  Guidance remains one hop: each
    member selects its predecessor and successor in sorted active-ID order.  A lost
    member is removed after a debounce interval and can rejoin after stable liveness.
    """

    def __init__(self, own_id: int, count: int, formation: str,
                 loss_timeout_s: float = 3.0, rejoin_stable_s: float = 1.0):
        if not 0 <= own_id < count:
            raise ValueError("own ID must be within the swarm")
        if not math.isfinite(loss_timeout_s) or loss_timeout_s <= 0:
            raise ValueError("loss timeout must be positive")
        if not math.isfinite(rejoin_stable_s) or rejoin_stable_s < 0:
            raise ValueError("rejoin stability must be non-negative")
        self.own_id = int(own_id)
        self.count = int(count)
        self.formation = str(formation)
        self.loss_timeout_s = float(loss_timeout_s)
        self.rejoin_stable_s = float(rejoin_stable_s)
        self.active_ids = set(range(count))
        self.last_seen = {}
        self.seen_since = {}
        self.started_at = None

    @property
    def discovery_ids(self) -> list[int]:
        return [i for i in range(self.count) if i != self.own_id]

    def resume_after_local_pause(self, now: float) -> None:
        """Restart timeout anchors after this agent intentionally blocked its I/O loop.

        Arm/takeoff helpers can occupy the process longer than the membership timeout.
        That local scheduling pause is not evidence that every remote member failed.
        """
        now = float(now)
        self.started_at = now
        for peer_id in self.active_ids - {self.own_id}:
            self.last_seen[peer_id] = now

    def update(self, now: float, fresh_ids: Iterable[int]) -> bool:
        now = float(now)
        if self.started_at is None:
            self.started_at = now
        fresh = {int(i) for i in fresh_ids if 0 <= int(i) < self.count}
        fresh.discard(self.own_id)
        for peer_id in fresh:
            # A caller may have captured its loop time before a blocking flight
            # operation. Never let that stale value move a liveness anchor back.
            self.last_seen[peer_id] = max(now, self.last_seen.get(peer_id, now))
            self.seen_since.setdefault(peer_id, now)
        changed = False
        for peer_id in self.discovery_ids:
            if peer_id in self.active_ids:
                anchor = self.last_seen.get(peer_id, self.started_at)
                if now - anchor >= self.loss_timeout_s:
                    self.active_ids.discard(peer_id)
                    self.seen_since.pop(peer_id, None)
                    changed = True
            elif peer_id in fresh:
                stable_from = self.seen_since.setdefault(peer_id, now)
                if now - stable_from >= self.rejoin_stable_s:
                    self.active_ids.add(peer_id)
                    changed = True
            else:
                # Rejoin means continuous liveness, not two isolated packets whose
                # timestamps happen to span the debounce interval.
                self.seen_since.pop(peer_id, None)
        self.active_ids.add(self.own_id)
        return changed

    @property
    def leader_id(self) -> int:
        return min(self.active_ids)

    def neighbors_for(self, agent_id: int | None = None) -> list[int]:
        agent_id = self.own_id if agent_id is None else int(agent_id)
        members = sorted(self.active_ids)
        if agent_id not in members or len(members) <= 1:
            return []
        if len(members) == 2:
            return [members[1 - members.index(agent_id)]]
        rank = members.index(agent_id)
        return sorted({members[(rank - 1) % len(members)],
                       members[(rank + 1) % len(members)]})

    def slot_map(self) -> dict[int, np.ndarray]:
        members = sorted(self.active_ids)
        slots = slots_for(self.formation, len(members))
        return {agent_id: np.asarray(slots[rank], dtype=float)
                for rank, agent_id in enumerate(members)}
