"""Conservative straight-line transition planning for live swarm formations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sitl.formation_library import MIN_SPACING, assign_slots, slots_for


# Formation templates are spaced at the project's hard minimum. Spread their
# slots before a live morph to leave room for SITL/controller tracking error.
TARGET_MIN_SPACING_M = MIN_SPACING + 1.5
TRACKING_CLEARANCE_M = MIN_SPACING + 0.35
TRANSITION_PEAK_SPEED_M_S = 0.65


@dataclass(frozen=True)
class FormationTransitionPlan:
    slots_by_drone: np.ndarray
    minimum_path_separation_m: float
    duration_s: float


def _minimum_pair_path_distance(start_a, end_a, start_b, end_b) -> float:
    """Exact closest distance for two agents moving linearly over alpha in [0, 1]."""
    relative_start = np.asarray(start_a, dtype=float) - np.asarray(start_b, dtype=float)
    relative_delta = ((np.asarray(end_a, dtype=float) - np.asarray(start_a, dtype=float)) -
                      (np.asarray(end_b, dtype=float) - np.asarray(start_b, dtype=float)))
    denom = float(np.dot(relative_delta, relative_delta))
    alpha = 0.0 if denom <= 1e-12 else float(np.clip(
        -np.dot(relative_start, relative_delta) / denom, 0.0, 1.0,
    ))
    return float(np.linalg.norm(relative_start + alpha * relative_delta))


def plan_formation_transition(current_positions_ne, formation: str, centroid_ne=None,
                              minimum_separation_m: float = TRACKING_CLEARANCE_M,
                              obstacles=(), max_transition_s: float = 45.0,
                              peak_speed_limit_m_s: float = TRANSITION_PEAK_SPEED_M_S
                              ) -> FormationTransitionPlan:
    """Assign slots by shortest travel and reject paths that violate separation.

    Positions are current shared-frame North/East metres. The resulting offsets
    are relative to ``centroid_ne`` and indexed by drone ID. A common translation
    of the goal does not change pairwise path clearance, so the planned paths are
    checked from the measured positions to the translated assigned slots.
    """
    current = np.asarray(current_positions_ne, dtype=float)
    if current.ndim != 2 or current.shape[1] != 2 or len(current) < 2:
        raise ValueError("a live transition needs at least two North/East positions")
    if not np.isfinite(current).all():
        raise ValueError("current positions must be finite")
    if not np.isfinite(minimum_separation_m) or minimum_separation_m <= 0:
        raise ValueError("minimum separation must be positive and finite")
    centroid = current.mean(axis=0) if centroid_ne is None else np.asarray(centroid_ne, dtype=float)
    if centroid.shape != (2,) or not np.isfinite(centroid).all():
        raise ValueError("formation centroid must contain finite North/East values")
    slots = slots_for(formation, len(current))[:, :2]
    template_minimum = min(
        float(np.linalg.norm(slots[i] - slots[j]))
        for i in range(len(slots)) for j in range(i + 1, len(slots))
    )
    if template_minimum < TARGET_MIN_SPACING_M:
        slots = slots * (TARGET_MIN_SPACING_M / template_minimum)
    relative_current = current - centroid
    current_3d = np.column_stack((relative_current, np.zeros(len(current))))
    best = None
    # A formation has no fixed compass orientation. Search rotations and choose
    # the assignment with the widest simultaneous straight-path clearance.
    for angle in np.linspace(0.0, 2.0 * np.pi, 72, endpoint=False):
        rotation = np.asarray([[np.cos(angle), -np.sin(angle)],
                              [np.sin(angle), np.cos(angle)]])
        rotated = slots @ rotation.T
        assignment = assign_slots(
            current_3d, np.column_stack((rotated, np.zeros(len(rotated)))),
        )
        assigned_candidate = rotated[np.asarray(assignment, dtype=int)]
        destinations_candidate = centroid + assigned_candidate
        closest_candidate = float("inf")
        max_travel_candidate = 0.0
        for i in range(len(current)):
            max_travel_candidate = max(
                max_travel_candidate,
                float(np.linalg.norm(destinations_candidate[i] - current[i])),
            )
            for j in range(i + 1, len(current)):
                closest_candidate = min(
                    closest_candidate,
                    _minimum_pair_path_distance(
                        current[i], destinations_candidate[i],
                        current[j], destinations_candidate[j],
                    ),
                )
        score = (closest_candidate, -max_travel_candidate)
        if best is None or score > best[0]:
            best = (score, assigned_candidate, destinations_candidate,
                    closest_candidate, max_travel_candidate)
    _, assigned, destinations, closest, max_travel = best
    if closest + 1e-9 < minimum_separation_m:
        raise ValueError(
            f"planned paths come within {closest:.2f} m; require at least "
            f"{minimum_separation_m:.2f} m"
        )
    # Quintic smoothstep peaks at 1.875 times average path speed. Keep a
    # conservative controller-speed bound and reject implausibly long transitions.
    duration = max(12.0, 1.875 * max_travel / peak_speed_limit_m_s)
    if duration > max_transition_s:
        raise ValueError(
            f"transition needs {duration:.1f} s at the speed limit; maximum is "
            f"{max_transition_s:.1f} s"
        )
    for obstacle in obstacles:
        if len(obstacle) != 3:
            raise ValueError("obstacles must be north/east/radius triples")
        center = np.asarray(obstacle[:2], dtype=float)
        radius = float(obstacle[2]) + 1.5
        if not np.isfinite(center).all() or not np.isfinite(radius) or radius <= 1.5:
            raise ValueError("obstacles must be finite with positive radius")
        for start, end in zip(current, destinations):
            segment = end - start
            length2 = float(segment @ segment)
            alpha = 0.0 if length2 <= 1e-12 else float(np.clip(
                np.dot(center - start, segment) / length2, 0.0, 1.0,
            ))
            clearance = float(np.linalg.norm(start + alpha * segment - center))
            if clearance < radius:
                raise ValueError(
                    f"planned path passes within {clearance:.2f} m of a keep-out zone; "
                    f"require {radius:.2f} m"
                )
    return FormationTransitionPlan(
        slots_by_drone=np.column_stack((assigned, np.zeros(len(assigned)))),
        minimum_path_separation_m=closest,
        duration_s=duration,
    )
