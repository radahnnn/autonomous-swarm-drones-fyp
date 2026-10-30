"""A mission route that every drone carries, so the swarm can keep flying it without a coordinator.

A route is a list of North/East waypoints, a start time and a speed on the shared operator clock. The
swarm goal at any time is a pure function of that clock: ``position(t)``. Every drone evaluates the
same function, so the swarm stays in step; if the coordinator disappears, drones that are allowed to
continue autonomously (decentralized and hybrid; see ``sitl/independent_agent_node.py``) simply keep
evaluating it.

The goal is continuous in time (no jumps): it holds at the first waypoint before the start, follows
the legs at constant speed, and either holds at the last waypoint or, if ``loop_start`` is set, repeats
the legs from that waypoint index onwards (so a patrol = approach leg + a repeating loop).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

MAX_WAYPOINTS = 16
MIN_SPEED_M_S = 0.1
MAX_SPEED_M_S = 1.5


@dataclass(frozen=True)
class Route:
    waypoints: Tuple[Tuple[float, float], ...]
    start_at: float
    speed_m_s: float
    loop_start: Optional[int] = None      # index of the waypoint where the repeating part begins

    def __post_init__(self):
        if not 2 <= len(self.waypoints) <= MAX_WAYPOINTS:
            raise ValueError(f"a route needs between 2 and {MAX_WAYPOINTS} waypoints")
        if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in self.waypoints):
            raise ValueError("route waypoints must be finite North/East pairs")
        if not math.isfinite(self.start_at):
            raise ValueError("route start time must be finite")
        if not math.isfinite(self.speed_m_s) or not MIN_SPEED_M_S <= self.speed_m_s <= MAX_SPEED_M_S:
            raise ValueError(f"route speed must be between {MIN_SPEED_M_S} and {MAX_SPEED_M_S} m/s")
        if self.loop_start is not None:
            if isinstance(self.loop_start, bool) or not isinstance(self.loop_start, int) or \
                    not 0 <= self.loop_start < len(self.waypoints) - 1:
                raise ValueError("loop_start must be a waypoint index before the last waypoint")
            if self._loop_length() <= 1e-6:
                raise ValueError("the repeating part of the route must have non-zero length")
        if self.length_m <= 1e-6:
            raise ValueError("route must have non-zero length")

    # ------------------------------------------------------------------ geometry
    def _legs(self) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        pts = [np.asarray(p, dtype=float) for p in self.waypoints]
        return [(a, b, float(np.linalg.norm(b - a))) for a, b in zip(pts, pts[1:])]

    @property
    def length_m(self) -> float:
        """Length of one pass (approach plus one lap if looping)."""
        return float(sum(leg[2] for leg in self._legs()))

    def _loop_length(self) -> float:
        return float(sum(leg[2] for leg in self._legs()[self.loop_start:]))

    @property
    def duration_s(self) -> float:
        return self.length_m / self.speed_m_s

    def position(self, t: float) -> np.ndarray:
        """Goal at operator time ``t`` (continuous in ``t``)."""
        travelled = max(0.0, t - self.start_at) * self.speed_m_s
        legs = self._legs()
        total = self.length_m
        if travelled >= total:
            if self.loop_start is None:
                return np.asarray(self.waypoints[-1], dtype=float)
            loop = self._loop_length()
            travelled = total - loop + ((travelled - (total - loop)) % loop)
        for a, b, length in legs:
            if travelled <= length:
                return a if length == 0 else a + (b - a) * (travelled / length)
            travelled -= length
        return np.asarray(self.waypoints[-1], dtype=float)

    def finished(self, t: float) -> bool:
        return self.loop_start is None and (t - self.start_at) * self.speed_m_s >= self.length_m

    # ------------------------------------------------------------------ transport
    def to_fields(self) -> Dict[str, Any]:
        return {"route_waypoints": [list(p) for p in self.waypoints], "route_start_at": self.start_at,
                "route_speed_m_s": self.speed_m_s,
                "route_loop_start": -1 if self.loop_start is None else self.loop_start}

    @classmethod
    def from_fields(cls, waypoints: Optional[Sequence], start_at: float, speed_m_s: float,
                    loop_start: int) -> Optional["Route"]:
        if waypoints is None:
            return None
        try:
            points = tuple((float(p[0]), float(p[1])) for p in waypoints)
        except (TypeError, ValueError, IndexError) as exc:
            raise ValueError("route waypoints must be North/East pairs") from exc
        if isinstance(loop_start, bool) or not isinstance(loop_start, int):
            raise ValueError("route loop start must be an integer (-1 for none)")
        return cls(points, float(start_at), float(speed_m_s), None if loop_start < 0 else loop_start)


def square_patrol(center_ne: Sequence[float], side_m: float, start_at: float, speed_m_s: float) -> Route:
    """Approach from the centre (no jump) to the first corner, then loop the four corners forever."""
    if not math.isfinite(side_m) or side_m < 2.0:
        raise ValueError("patrol side must be at least 2 m")
    n, e = float(center_ne[0]), float(center_ne[1])
    h = side_m / 2.0
    corners = ((n + h, e + h), (n + h, e - h), (n - h, e - h), (n - h, e + h))
    return Route(((n, e),) + corners + (corners[0],), start_at, speed_m_s, loop_start=1)
