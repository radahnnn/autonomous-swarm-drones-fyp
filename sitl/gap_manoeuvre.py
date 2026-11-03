"""Squeeze through a gap between two obstacles: V -> column -> through -> V (pure decision logic).

The user's idea (2026-11-02): two towers stand on the swarm's way; the drones sense them; the formation does not fit
between them, so the swarm stops short, turns into a column aligned with the direction of travel (one behind another),
flies through, and once every drone is past the towers turns back into a V and carries on.

Geometry is in shared North/East metres. A *gap* is a pair of known obstacles whose midpoint lies ahead on the path and
whose connecting line crosses it; its free width is the distance between the obstacles' surfaces. A formation fits if
its width across the direction of travel, plus a clearance on each side, is below that free width. The clearance is the
drones' obstacle-avoidance zone (3 m beyond the surface, see `IndependentAgentNode._avoid_obstacles`) plus the position
error sigma.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

AVOID_ZONE_M = 3.0          # agents push away from an obstacle within radius + 3 m
STAGGER_TOLERANCE_M = 2.0   # obstacles further apart than this along the path are not one gate


@dataclass(frozen=True)
class Gap:
    centre: Tuple[float, float]          # midpoint between the two obstacle centres
    free_width_m: float                  # surface-to-surface distance
    along_m: float                       # distance from the reference point to the gap along the path
    obstacles: Tuple[Tuple[float, float, float], Tuple[float, float, float]]


def unit(heading_rad: float) -> np.ndarray:
    return np.array([math.cos(heading_rad), math.sin(heading_rad)])          # North, East


def heading_to(start: Sequence[float], end: Sequence[float]) -> float:
    d = np.asarray(end, float)[:2] - np.asarray(start, float)[:2]
    if float(np.linalg.norm(d)) < 1e-6:
        raise ValueError("start and end coincide")
    return math.atan2(d[1], d[0])


def formation_width(slots_ne: np.ndarray, heading_rad: float) -> float:
    """Extent of the formation across the direction of travel."""
    across = np.array([-math.sin(heading_rad), math.cos(heading_rad)])
    projections = np.asarray(slots_ne, float)[:, :2] @ across
    return float(projections.max() - projections.min())


def find_gap(obstacles: Iterable[Sequence[float]], origin_ne: Sequence[float], heading_rad: float,
             lookahead_m: float = 40.0, max_pair_m: float = 30.0) -> Optional[Gap]:
    """Nearest gap ahead whose connecting line crosses the straight path from ``origin`` along ``heading``."""
    origin = np.asarray(origin_ne, float)[:2]
    forward, across = unit(heading_rad), np.array([-math.sin(heading_rad), math.cos(heading_rad)])
    items = [tuple(float(v) for v in o) for o in obstacles]
    best = None
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = np.array(items[i][:2]), np.array(items[j][:2])
            centre_distance = float(np.linalg.norm(a - b))
            if centre_distance > max_pair_m:
                continue
            side_a, side_b = float((a - origin) @ across), float((b - origin) @ across)
            if side_a * side_b >= 0:                       # both on the same side: not a gap the path goes through
                continue
            along_a, along_b = float((a - origin) @ forward), float((b - origin) @ forward)
            # Staggered obstacles are not a side-by-side gate (Codex review #6); local avoidance handles them.
            if abs(along_a - along_b) > max(items[i][2], items[j][2]) + STAGGER_TOLERANCE_M:
                continue
            centre = (a + b) / 2
            along = float((centre - origin) @ forward)
            if not 0.0 < along <= lookahead_m:
                continue
            gap = Gap((float(centre[0]), float(centre[1])), centre_distance - items[i][2] - items[j][2], along,
                      (items[i], items[j]))
            if best is None or gap.along_m < best.along_m:
                best = gap
    return best


def through_route(origin_ne: Sequence[float], gap: Gap, target_ne: Sequence[float]) -> list:
    """Waypoints that take the swarm through the gate's midpoint (the corridor the clearance check assumes), then on
    to the target. Codex review #6: flying the original straight line could pass close to one tower even when the
    centre-to-centre width looked passable."""
    return [tuple(float(v) for v in np.asarray(origin_ne, float)[:2]), tuple(float(v) for v in gap.centre),
            tuple(float(v) for v in np.asarray(target_ne, float)[:2])]


def needs_squeeze(slots_ne: np.ndarray, heading_rad: float, gap: Gap, sigma_m: float = 0.0) -> bool:
    return formation_width(slots_ne, heading_rad) + 2.0 * (AVOID_ZONE_M + sigma_m) > gap.free_width_m


def passable_single_file(gap: Gap, sigma_m: float = 0.0) -> bool:
    return 2.0 * (AVOID_ZONE_M + sigma_m) <= gap.free_width_m


def all_past(positions_ne: np.ndarray, gap: Gap, heading_rad: float, margin_m: float = 4.0) -> bool:
    """Every drone is beyond the gap line (plus a margin) along the direction of travel."""
    forward = unit(heading_rad)
    centre = np.asarray(gap.centre)
    return bool(np.all((np.asarray(positions_ne, float)[:, :2] - centre) @ forward > margin_m))


def stop_point(gap: Gap, origin_ne: Sequence[float], heading_rad: float, formation_length_m: float,
               standoff_m: float = 6.0) -> np.ndarray:
    """Where the formation centre should wait: far enough before the gap that the column fits behind it."""
    back = gap.along_m - standoff_m - formation_length_m / 2.0
    return np.asarray(origin_ne, float)[:2] + unit(heading_rad) * max(0.0, back)


def formation_length(slots_ne: np.ndarray, heading_rad: float) -> float:
    projections = np.asarray(slots_ne, float)[:, :2] @ unit(heading_rad)
    return float(projections.max() - projections.min())
