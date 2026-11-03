"""Simulated range sensing of obstacles, and sharing them between drones (pure logic).

What this is: a *simulated* sensor. The agent is given the world's true obstacle list (the stand-in for the physical
world) and "senses" an obstacle when it is within ``range_m`` of the obstacle's surface. Nothing is detected from
camera images. A detection adds the obstacle to the drone's *known* list (what the avoidance acts on) and is shared with
peers, so a drone can avoid a pole another drone found. A real sensor would add noise, field of view, occlusion and
misses; none of that is modelled here.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

Obstacle = Tuple[float, float, float]            # north, east, radius (m)
MAX_SHARED = 16
MERGE_TOLERANCE_M = 0.75                         # two reports of the same pole closer than this are one pole


def validate(item) -> Obstacle:
    try:
        north, east, radius = (float(v) for v in item)
    except (TypeError, ValueError) as exc:
        raise ValueError("obstacle must be north,east,radius") from exc
    if not all(math.isfinite(v) for v in (north, east, radius)) or radius <= 0:
        raise ValueError("obstacle must be finite with a positive radius")
    return north, east, radius


class ObstacleSensor:
    def __init__(self, world: Iterable[Sequence[float]] = (), range_m: float = 8.0):
        if not math.isfinite(range_m) or range_m <= 0:
            raise ValueError("sensing range must be positive")
        self.world: List[Obstacle] = [validate(item) for item in world]
        self.range_m = float(range_m)
        self.known: List[Obstacle] = []
        self.detected_by_self = 0           # poles this drone found with its own sensor
        self.learned_from_peers = 0

    def _find(self, obstacle: Obstacle):
        for index, other in enumerate(self.known):
            if math.hypot(other[0] - obstacle[0], other[1] - obstacle[1]) < MERGE_TOLERANCE_M:
                return index
        return None

    def update(self, position_ne: Sequence[float]) -> List[Obstacle]:
        """Sense from ``position_ne``; returns the obstacles newly added to the known list by this drone."""
        north, east = float(position_ne[0]), float(position_ne[1])
        if not (math.isfinite(north) and math.isfinite(east)):
            return []
        found = []
        for obstacle in self.world:
            if math.hypot(obstacle[0] - north, obstacle[1] - east) - obstacle[2] <= self.range_m \
                    and self._find(obstacle) is None:
                self.known.append(obstacle)
                self.detected_by_self += 1
                found.append(obstacle)
        return found

    def learn(self, items: Iterable[Sequence[float]]) -> List[Obstacle]:
        """Merge obstacles reported by a peer; malformed items are ignored. Returns those that were new."""
        added = []
        for item in list(items)[:MAX_SHARED]:
            try:
                obstacle = validate(item)
            except ValueError:
                continue
            if self._find(obstacle) is None and len(self.known) < 4 * MAX_SHARED:
                self.known.append(obstacle)
                self.learned_from_peers += 1
                added.append(obstacle)
        return added

    def shareable(self) -> List[List[float]]:
        return [list(item) for item in self.known[:MAX_SHARED]]

    def status(self) -> dict:
        return {"known": len(self.known), "detected": self.detected_by_self,
                "learned": self.learned_from_peers, "range_m": self.range_m,
                "items": [[round(v, 2) for v in item] for item in self.known[:MAX_SHARED]]}
