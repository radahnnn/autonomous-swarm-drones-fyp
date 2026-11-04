"""
Formation slot templates and slot assignment for the swarm flight console.

Slots are offsets from the swarm centroid in the NED frame: [North, East, Down] metres.
Every style works for any number of drones (1..10+). For the sizes the project was
developed with (up to 5 drones) the hand-tuned slot tables are kept exactly; larger
swarms use generators that extend the same pattern. All templates keep neighbours at
least MIN_SPACING apart.
"""

import math
import os
from typing import Dict, List

import numpy as np
from swarm_core.assignment import assign, squared_cost

MIN_SPACING = 2.5  # metres, smallest allowed distance between two slots
RING_SPACING_M = 4.0  # adjacent-slot spacing of the ring template
STYLES = ("v", "line", "column", "diamond", "ring", "echelon", "triangle", "grid", "pairs", "pairs_b")
GRID_SPACING_M = 4.0  # row and column spacing of the grid template (as the ring: room to leave it under GPS error)
# World-anchored styles: the compass orientation is part of the layout (sectors tied to places on the map), so the
# transition planner must not rotate them to improve clearance.
WORLD_ANCHORED = frozenset({"pairs", "pairs_b"})
PAIR_PARTNER_SPACING_M = 6.0    # wide enough for ~1 m GPS error plus the planner's clearance
PAIR_SECTOR_RADIUS_M = 9.0

_V_TABLE: Dict[int, List[List[float]]] = {
    1: [[0.0, 0.0]],
    2: [[0.0, -2.5], [0.0, 2.5]],
    3: [[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0]],
    4: [[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0], [-5.0, 0.0]],
    5: [[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0], [-6.0, -6.0], [-6.0, 6.0]],
}
_DIAMOND_TABLE: Dict[int, List[List[float]]] = {
    1: [[0.0, 0.0]],
    2: [[0.0, -2.5], [0.0, 2.5]],
    3: [[2.5, 0.0], [-2.5, -3.0], [-2.5, 3.0]],
    4: [[3.5, 0.0], [0.0, -3.5], [0.0, 3.5], [-3.5, 0.0]],
    5: [[4.0, 0.0], [0.0, -4.0], [0.0, 4.0], [-4.0, 0.0], [0.0, 0.0]],
}


def _to_3d(rows) -> np.ndarray:
    pts = np.asarray(rows, dtype=float).reshape(-1, 2)
    return np.hstack([pts, np.zeros((len(pts), 1))])


def _v(n: int) -> np.ndarray:
    if n in _V_TABLE:
        return _to_3d(_V_TABLE[n])
    slots = [[0.0, 0.0]]  # apex, then wing pairs fanning out behind it, left before right
    rank = 1
    while len(slots) < n:
        slots.append([-3.0 * rank, -3.0 * rank])
        if len(slots) < n:
            slots.append([-3.0 * rank, 3.0 * rank])
        rank += 1
    return _to_3d(slots)


def _abreast(n: int, axis: int) -> np.ndarray:
    """Evenly spaced row (axis=1, East) or file (axis=0, North), centred on the centroid."""
    if n == 1:
        return _to_3d([[0.0, 0.0]])
    spacing = 5.0 if n == 2 else 3.0
    pts = np.zeros((n, 2))
    for k in range(n):
        offset = (k - (n - 1) / 2.0) * spacing
        pts[k, axis] = -offset if axis == 0 else offset  # column: leader (k=0) in front (+North)
    return _to_3d(pts)


def _diamond(n: int) -> np.ndarray:
    if n in _DIAMOND_TABLE:
        return _to_3d(_DIAMOND_TABLE[n])
    slots: List[List[float]] = []
    d = 4.0
    while len(slots) < n:
        slots += [[d, 0.0], [0.0, -d], [0.0, d], [-d, 0.0]]
        if d == 4.0:
            slots.append([0.0, 0.0])
        d += 4.0
    return _to_3d(slots[:n])


def _ring(n: int) -> np.ndarray:
    if n == 1:
        return _to_3d([[0.0, 0.0]])
    # Adjacent drones are >= 4.0 m apart (like the other formations). With 3.0 m the 10-drone ring
    # had too little room: no live transition out of it could keep the planner's path clearance.
    radius = max(4.0, RING_SPACING_M / (2.0 * math.sin(math.pi / n)))
    ang = [2.0 * math.pi * k / n for k in range(n)]
    return _to_3d([[radius * math.cos(a), radius * math.sin(a)] for a in ang])


def _echelon(n: int) -> np.ndarray:
    step = 2.5
    return _to_3d([[((n - 1) / 2.0 - k) * step, -((n - 1) / 2.0 - k) * step] for k in range(n)])


def _triangle(n: int) -> np.ndarray:
    pts: List[List[float]] = []
    row = 0
    while len(pts) < n:
        for j in range(row + 1):
            if len(pts) < n:
                pts.append([-2.5 * row, (j - row / 2.0) * 3.0])
        row += 1
    arr = np.asarray(pts, dtype=float)
    arr[:, 0] -= arr[:, 0].mean()
    return _to_3d(arr)


def _grid(n: int) -> np.ndarray:
    """Rectangular grid (promised in the proposal): ceil(sqrt(n)) columns across the direction of travel, rows behind
    each other; a short last row is centred. 10 drones -> 4 + 4 + 2, 6 -> 3 + 3. Centred on the centroid."""
    cols = max(1, math.ceil(math.sqrt(n)))
    pts: List[List[float]] = []
    row = 0
    while len(pts) < n:
        in_row = min(cols, n - len(pts))
        for j in range(in_row):
            pts.append([-GRID_SPACING_M * row, (j - (in_row - 1) / 2.0) * GRID_SPACING_M])
        row += 1
    arr = np.asarray(pts, dtype=float)
    arr -= arr.mean(axis=0)
    return _to_3d(arr)


def pair_sector_centres(pairs: int) -> np.ndarray:
    """North/East centres of the ``2 * pairs`` survey sectors, evenly spaced around the origin (sector 0 due North)."""
    sectors = 2 * pairs
    radius = max(PAIR_SECTOR_RADIUS_M, 13.0 / (2.0 * math.sin(math.pi / pairs))) if pairs >= 3 else PAIR_SECTOR_RADIUS_M
    phi = 2.0 * math.pi * np.arange(sectors) / sectors
    return np.column_stack((radius * np.cos(phi), radius * np.sin(phi)))


def scaled_pair_sector_centres(pairs: int) -> np.ndarray:
    """Sector centres as the pair formations actually place them (with formation_scale() applied)."""
    return pair_sector_centres(pairs) * formation_scale()


def _pairs(n: int, phase: int) -> np.ndarray:
    """Drones in pairs on alternate survey sectors: phase 0 uses sectors 0, 2, 4 ..., phase 1 uses 1, 3, 5 ...

    Slots 2k and 2k+1 form pair k, ``PAIR_PARTNER_SPACING_M`` apart across the sector's direction. The two phases
    are different position sets, so moving from one to the other really moves every pair, and together they cover
    every sector.
    """
    if n == 1:
        return _to_3d([[0.0, 0.0]])
    pair_count = (n + 1) // 2
    centres = pair_sector_centres(pair_count)
    pts: List[List[float]] = []
    for k in range(pair_count):
        sector = (2 * k + phase) % (2 * pair_count)
        north, east = centres[sector]
        phi = 2.0 * math.pi * sector / (2 * pair_count)
        tangent = (-math.sin(phi), math.cos(phi))
        for sign in (-1.0, 1.0):
            if len(pts) < n:
                half = 0.5 * PAIR_PARTNER_SPACING_M
                pts.append([north + sign * half * tangent[0], east + sign * half * tangent[1]])
    return _to_3d(pts)


_GENERATORS = {
    "pairs": lambda n: _pairs(n, 0),
    "pairs_b": lambda n: _pairs(n, 1),
    "v": _v,
    "line": lambda n: _abreast(n, axis=1),
    "column": lambda n: _abreast(n, axis=0),
    "diamond": _diamond,
    "ring": _ring,
    "echelon": _echelon,
    "triangle": _triangle,
    "grid": _grid,
}


FORMATION_SCALE_ENV = "FYP_FORMATION_SCALE"


def formation_scale() -> float:
    """Global spacing multiplier for every formation (1.0 = the templates as designed).

    Read from the environment so one setting reaches the console, every agent process it starts, the standalone start
    positions and the Gazebo spawn layout alike. Larger spacing is the answer to GPS-level position error: with ~1 m of
    wander per drone, 4.2 m neighbour spacing made the protocols' safety checks trip on measurement noise alone.
    """
    raw = os.environ.get(FORMATION_SCALE_ENV, "1.0")
    try:
        scale = float(raw)
    except ValueError as exc:
        raise ValueError(f"{FORMATION_SCALE_ENV} must be a number") from exc
    if not math.isfinite(scale) or not 1.0 <= scale <= 3.0:
        raise ValueError(f"{FORMATION_SCALE_ENV} must be between 1.0 and 3.0")
    return scale


def slots_for(style: str, n: int) -> np.ndarray:
    """Return the (n, 3) slot offsets [N, E, D] for `n` drones in the given formation style (times formation_scale())."""
    if style not in _GENERATORS:
        raise ValueError(f"Unknown formation style '{style}'. Valid: {list(STYLES)}")
    if n < 1:
        raise ValueError("A formation needs at least one drone")
    slots = np.asarray(_GENERATORS[style](n), dtype=float)
    scale = formation_scale()
    if scale != 1.0:
        slots = slots.copy()
        slots[:, :2] *= scale
    return slots


def assign_slots(current_offsets: np.ndarray, slots: np.ndarray, z_weight: float = 2.0) -> List[int]:
    """
    Optimal drone-to-slot matching (Hungarian algorithm) minimising total squared travel,
    which prevents crossing paths. Returns slot_index[i] for every drone i.
    """
    current = np.asarray(current_offsets, dtype=float)
    slots = np.asarray(slots, dtype=float)
    if len(current) != len(slots):
        raise ValueError("Need exactly one slot per drone")
    return [int(c) for c in assign(squared_cost(current, slots, (1.0, 1.0, z_weight)))]
