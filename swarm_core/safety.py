"""Local, velocity-aware separation intervention shared by the control modes."""

from typing import Iterable

import numpy as np


def separation_guard_acceleration(
    position: np.ndarray,
    velocity: np.ndarray,
    neighbors: Iterable[dict],
    *,
    minimum_separation_m: float = 2.5,
    reaction_time_s: float = 1.0,
    max_accel_m_s2: float = 2.5,
) -> np.ndarray:
    """Return an outward acceleration when a local peer is closing too quickly.

    The guard uses only the vehicle's own estimate and the neighbor states supplied by
    the active transport. It brakes before the nominal separation boundary using a
    reaction-time plus stopping-distance buffer. It is a sampled local intervention,
    not a formal collision-avoidance guarantee under stale or missing peer data.
    """
    pos = np.asarray(position, dtype=float)
    vel = np.asarray(velocity, dtype=float)
    if (pos.ndim != 1 or vel.shape != pos.shape or not np.isfinite(pos).all()
            or not np.isfinite(vel).all()):
        raise ValueError("position and velocity must be finite vectors with the same shape")
    if (not np.isfinite(minimum_separation_m) or minimum_separation_m <= 0
            or not np.isfinite(reaction_time_s) or reaction_time_s < 0
            or not np.isfinite(max_accel_m_s2) or max_accel_m_s2 <= 0):
        raise ValueError("separation and acceleration limits must be finite and positive")

    result = np.zeros_like(pos)
    for neighbor in neighbors:
        try:
            other_pos = np.asarray(neighbor["position"], dtype=float)
            other_vel = np.asarray(neighbor["velocity"], dtype=float)
        except (KeyError, TypeError, ValueError):
            continue
        if (other_pos.shape != pos.shape or other_vel.shape != pos.shape
                or not np.isfinite(other_pos).all() or not np.isfinite(other_vel).all()):
            continue
        relative_position = pos - other_pos
        distance = float(np.linalg.norm(relative_position))
        if distance <= 1e-6:
            # Deterministic axis avoids undefined direction for coincident estimates.
            axis = np.zeros_like(pos)
            axis[0] = 1.0
            result += max_accel_m_s2 * axis
            continue

        direction = relative_position / distance
        relative_velocity = vel - other_vel
        closing_speed = max(0.0, -float(np.dot(relative_velocity, direction)))
        stopping_buffer = (
            closing_speed * reaction_time_s
            + closing_speed * closing_speed / (2.0 * max_accel_m_s2)
        )
        available_gap = distance - minimum_separation_m
        if available_gap >= stopping_buffer:
            continue

        penetration = max(0.0, -available_gap)
        urgency = max(0.0, stopping_buffer - available_gap)
        magnitude = min(
            max_accel_m_s2,
            4.0 * urgency + 3.0 * closing_speed + 8.0 * penetration,
        )
        result += magnitude * direction

    norm = float(np.linalg.norm(result))
    if norm > max_accel_m_s2:
        result *= max_accel_m_s2 / norm
    return result
