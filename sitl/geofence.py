"""Software geofence and altitude layers for the independent agents (pure logic).

Positions are in the shared local North/East frame (metres) used everywhere else. A fence is either a
circle or a North/East box, with an optional maximum altitude. It has two layers:

* **Prevention** (``clamp``): every position target an agent sends is projected into the fence shrunk
  by ``margin_m``, so a goal moved too far (or a formation that does not fit) never commands a
  vehicle outside the boundary.
* **Detection** (``status``): ``inside`` / ``warn`` (within the margin of the edge) / ``breach``
  (actually outside). The agent holds at the nearest inside point on breach and lands if the breach
  persists.

This is a software fence on the commanded and measured positions. It is not a certified geofence and
is no substitute for the flight controller's own fence or for RC override on real hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Geofence:
    kind: str                                   # "circle" | "box"
    center_ne: Tuple[float, float] = (0.0, 0.0)
    radius_m: float = 0.0
    box: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)   # north_min, east_min, north_max, east_max
    max_alt_m: Optional[float] = None
    margin_m: float = 2.0

    def __post_init__(self):
        if self.kind not in ("circle", "box"):
            raise ValueError("geofence kind must be 'circle' or 'box'")
        if not math.isfinite(self.margin_m) or self.margin_m < 0:
            raise ValueError("geofence margin must be finite and non-negative")
        if self.kind == "circle":
            if not math.isfinite(self.radius_m) or self.radius_m <= self.margin_m:
                raise ValueError("geofence radius must be finite and larger than the margin")
        else:
            n0, e0, n1, e1 = self.box
            if not all(math.isfinite(v) for v in self.box) or n1 - n0 <= 2 * self.margin_m or e1 - e0 <= 2 * self.margin_m:
                raise ValueError("geofence box must be finite and larger than twice the margin on both axes")
        if self.max_alt_m is not None and (not math.isfinite(self.max_alt_m) or self.max_alt_m <= 0.5):
            raise ValueError("geofence maximum altitude must be finite and above 0.5 m")

    # ------------------------------------------------------------------ geometry
    def _inside(self, point: np.ndarray, shrink: float) -> bool:
        if self.kind == "circle":
            return float(np.linalg.norm(point - np.asarray(self.center_ne))) <= self.radius_m - shrink + 1e-9
        n0, e0, n1, e1 = self.box
        return (n0 + shrink - 1e-9 <= point[0] <= n1 - shrink + 1e-9 and
                e0 + shrink - 1e-9 <= point[1] <= e1 - shrink + 1e-9)

    def contains(self, ne: Sequence[float], inner: bool = False) -> bool:
        """True inside the fence (``inner``: inside the fence shrunk by the margin)."""
        return self._inside(np.asarray(ne, dtype=float)[:2], self.margin_m if inner else 0.0)

    def clamp(self, ne: Sequence[float]) -> np.ndarray:
        """Nearest point inside the margin-shrunk fence (the point itself if already inside)."""
        point = np.asarray(ne, dtype=float)[:2].copy()
        if self.kind == "circle":
            center = np.asarray(self.center_ne, dtype=float)
            delta = point - center
            limit = self.radius_m - self.margin_m
            distance = float(np.linalg.norm(delta))
            return point if distance <= limit else center + delta * (limit / distance)
        n0, e0, n1, e1 = self.box
        return np.array([min(max(point[0], n0 + self.margin_m), n1 - self.margin_m),
                         min(max(point[1], e0 + self.margin_m), e1 - self.margin_m)])

    def clamp_altitude(self, altitude_m: float) -> float:
        if self.max_alt_m is None:
            return float(altitude_m)
        return min(float(altitude_m), self.max_alt_m - min(self.margin_m, 0.5 * self.max_alt_m))

    def distance_to_edge(self, ne: Sequence[float]) -> float:
        """Signed distance to the boundary: positive inside, negative outside (horizontal only)."""
        point = np.asarray(ne, dtype=float)[:2]
        if self.kind == "circle":
            return self.radius_m - float(np.linalg.norm(point - np.asarray(self.center_ne)))
        n0, e0, n1, e1 = self.box
        return float(min(point[0] - n0, n1 - point[0], point[1] - e0, e1 - point[1]))

    def status(self, ne: Sequence[float], altitude_m: Optional[float] = None) -> str:
        edge = self.distance_to_edge(ne)
        too_high = (self.max_alt_m is not None and altitude_m is not None and altitude_m > self.max_alt_m)
        if edge < 0 or too_high:
            return "breach"
        if edge < self.margin_m or (self.max_alt_m is not None and altitude_m is not None and
                                    altitude_m > self.max_alt_m - min(self.margin_m, 0.5 * self.max_alt_m)):
            return "warn"
        return "inside"

    def describe(self) -> dict:
        info = {"kind": self.kind, "margin_m": self.margin_m, "max_alt_m": self.max_alt_m}
        if self.kind == "circle":
            info.update(center=list(self.center_ne), radius_m=self.radius_m)
        else:
            info.update(box=list(self.box))
        return info

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_options(cls, radius: Optional[float] = None, box: Optional[str] = None,
                     max_alt: Optional[float] = None, margin: float = 2.0) -> Optional["Geofence"]:
        """Build from CLI-style options; None when no fence is requested."""
        if radius is not None and box is not None:
            raise ValueError("choose either a geofence radius or a box, not both")
        if radius is not None:
            return cls("circle", radius_m=float(radius), max_alt_m=max_alt, margin_m=margin)
        if box is not None:
            try:
                values = tuple(float(v) for v in str(box).split(","))
            except ValueError as exc:
                raise ValueError("geofence box must be north_min,east_min,north_max,east_max") from exc
            if len(values) != 4:
                raise ValueError("geofence box must be north_min,east_min,north_max,east_max")
            return cls("box", box=values, max_alt_m=max_alt, margin_m=margin)
        if max_alt is not None:
            # altitude-only fence: an effectively unbounded horizontal circle
            return cls("circle", radius_m=1.0e6, max_alt_m=max_alt, margin_m=margin)
        return None


def formation_layer_index(slots, drone_id: int, layers: int) -> int:
    """Layer of a drone such that neighbours in the starting formation fly at different heights (2 layers).

    Live finding 2026-11-04 (campaign stats-20261104-layers): with layer = ID mod 2 the close pairs of the V (D1-D2,
    D5-D7, D7-D9, ...) were often on the SAME layer, so the layers did not separate them. Two-colour the starting
    formation's proximity graph (slots within 1.2 x the closest slot spacing are neighbours) breadth-first from drone 0;
    for a V this alternates along each arm. Fixed per drone for the flight (no height change during transitions).
    Other layer counts keep ID mod layers."""
    if layers != 2:
        return int(drone_id) % layers
    import numpy as np
    points = np.asarray(slots, dtype=float)[:, :2]
    n = len(points)
    if n < 2:
        return 0
    distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    closest = float(np.min(distances[np.triu_indices(n, 1)]))
    adjacent = (distances <= 1.2 * closest) & ~np.eye(n, dtype=bool)
    colour = [-1] * n
    for root in range(n):
        if colour[root] != -1:
            continue
        colour[root], queue = 0, [root]
        while queue:
            i = queue.pop(0)
            for j in np.flatnonzero(adjacent[i]):
                if colour[j] == -1:
                    colour[j] = 1 - colour[i]
                    queue.append(int(j))
    return colour[int(drone_id)]


def altitude_layer_offset(drone_id: int, layers: int, spacing_m: float, slots=None) -> float:
    """Constant altitude offset (m, positive up) for a drone: layer = id mod layers.

    Layers are assigned by drone ID so a drone never changes altitude during a formation transition;
    the planner and the separation guard stay purely horizontal (conservative), the layers only add
    vertical separation on top of them.
    """
    if layers < 1 or layers > 5:
        raise ValueError("altitude layers must be between 1 and 5")
    if not math.isfinite(spacing_m) or spacing_m < 0 or spacing_m > 5.0:
        raise ValueError("altitude layer spacing must be between 0 and 5 m")
    index = formation_layer_index(slots, drone_id, layers) if slots is not None else int(drone_id) % layers
    return float(index * spacing_m)
