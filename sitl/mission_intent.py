"""Versioned, shared high-level mission intent for local swarm agents."""

from dataclasses import asdict, dataclass
import math
from typing import Any, Dict

import numpy as np

from sitl.formation_library import MIN_SPACING


VERSION = 1
FRAME = "shared_local_north_east_metres"
MAX_TTL_S = 60.0
MAX_FUTURE_SKEW_S = 0.25


@dataclass(frozen=True)
class MissionIntent:
    """A common goal; each aircraft combines it with its own local slot."""

    sequence: int
    issued_at: float
    expires_at: float
    goal_ne: tuple[float, float]
    formation: str = "configured"
    target_alt_m: float = 5.0
    version: int = VERSION
    frame: str = FRAME
    formation_slots: tuple[tuple[float, float, float], ...] | None = None
    formation_start_slots: tuple[tuple[float, float, float], ...] | None = None
    formation_transition_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result["goal_ne"] = list(self.goal_ne)
        if self.formation_slots is not None:
            result["formation_slots"] = [list(slot) for slot in self.formation_slots]
        if self.formation_start_slots is not None:
            result["formation_start_slots"] = [list(slot) for slot in self.formation_start_slots]
        return result

    @classmethod
    def from_dict(cls, value: Dict[str, Any], now: float) -> "MissionIntent":
        if not isinstance(value, dict):
            raise ValueError("mission intent must be an object")
        if value.get("version") != VERSION or value.get("frame") != FRAME:
            raise ValueError("unsupported mission version or coordinate frame")
        sequence = value.get("sequence")
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1:
            raise ValueError("invalid mission sequence")
        issued_at = float(value.get("issued_at"))
        expires_at = float(value.get("expires_at"))
        goal = value.get("goal_ne")
        if not isinstance(goal, (list, tuple)) or len(goal) != 2:
            raise ValueError("mission goal must contain north/east metres")
        goal_ne = tuple(float(component) for component in goal)
        if not all(math.isfinite(component) for component in (*goal_ne, issued_at, expires_at, now)):
            raise ValueError("mission intent contains non-finite values")
        if issued_at > now + MAX_FUTURE_SKEW_S:
            raise ValueError("mission intent is from the future")
        ttl = expires_at - issued_at
        if ttl <= 0 or ttl > MAX_TTL_S:
            raise ValueError("mission intent expiry is invalid")
        if expires_at <= now:
            raise ValueError("mission intent has expired")
        formation = value.get("formation", "configured")
        if not isinstance(formation, str) or not formation or len(formation) > 64:
            raise ValueError("invalid formation identifier")
        target_alt_m = float(value.get("target_alt_m", 5.0))
        if not math.isfinite(target_alt_m) or not 0.5 <= target_alt_m <= 25.0:
            raise ValueError("mission altitude must be between 0.5 and 25 metres")
        raw_slots = value.get("formation_slots")
        formation_slots = None
        raw_start_slots = value.get("formation_start_slots")
        formation_start_slots = None
        transition_s = float(value.get("formation_transition_s", 0.0))
        if not math.isfinite(transition_s) or not 0.0 <= transition_s <= 60.0:
            raise ValueError("formation transition duration must be between 0 and 60 seconds")
        if raw_slots is not None:
            if not isinstance(raw_slots, (list, tuple)) or not 1 <= len(raw_slots) <= 10:
                raise ValueError("formation plan needs between 1 and 10 slots")
            if transition_s != 0.0 and not 0.1 <= transition_s <= 60.0:
                raise ValueError("formation transition duration must be zero or between 0.1 and 60 seconds")
            try:
                slot_array = np.asarray(raw_slots, dtype=float)
            except (TypeError, ValueError) as exc:
                raise ValueError("formation slots must be numeric North/East/Down triples") from exc
            if (slot_array.shape != (len(raw_slots), 3) or
                    not np.isfinite(slot_array).all()):
                raise ValueError("formation slots must be finite North/East/Down triples")
            for i in range(len(slot_array)):
                for j in range(i + 1, len(slot_array)):
                    if np.linalg.norm(slot_array[i, :2] - slot_array[j, :2]) < MIN_SPACING - 1e-9:
                        raise ValueError("formation slots violate the minimum horizontal spacing")
            formation_slots = tuple(tuple(float(value) for value in row) for row in slot_array)
            if raw_start_slots is not None:
                try:
                    start_array = np.asarray(raw_start_slots, dtype=float)
                except (TypeError, ValueError) as exc:
                    raise ValueError("formation start slots must be numeric triples") from exc
                if (start_array.shape != slot_array.shape or
                        not np.isfinite(start_array).all()):
                    raise ValueError("formation start slots must be finite triples matching target slots")
                for i in range(len(start_array)):
                    for j in range(i + 1, len(start_array)):
                        if np.linalg.norm(start_array[i, :2] - start_array[j, :2]) < MIN_SPACING - 1e-9:
                            raise ValueError("formation start slots violate the minimum horizontal spacing")
                formation_start_slots = tuple(
                    tuple(float(value) for value in row) for row in start_array
                )
            if transition_s > 0.0 and formation_start_slots is None:
                raise ValueError("a live formation transition requires measured start slots")
        elif raw_start_slots is not None:
            raise ValueError("formation start slots require formation target slots")
        elif transition_s != 0.0:
            raise ValueError("formation transition duration requires formation slots")
        return cls(sequence, issued_at, expires_at, goal_ne, formation, target_alt_m,
                   VERSION, FRAME, formation_slots, formation_start_slots, transition_s)


@dataclass
class MissionLease:
    """Validated intent with a local monotonic expiry after wall-clock validation."""

    intent: MissionIntent | None = None
    deadline_monotonic: float | None = None

    def accept(self, value: Dict[str, Any], wall_now: float,
               monotonic_now: float) -> MissionIntent:
        intent = MissionIntent.from_dict(value, wall_now)
        remaining = intent.expires_at - float(wall_now)
        if not math.isfinite(monotonic_now) or remaining <= 0:
            raise ValueError("mission lease has no remaining validity")
        self.intent = intent
        self.deadline_monotonic = float(monotonic_now) + remaining
        return intent

    def active(self, monotonic_now: float) -> MissionIntent | None:
        if self.intent is None or self.deadline_monotonic is None:
            return None
        if not math.isfinite(monotonic_now) or float(monotonic_now) >= self.deadline_monotonic:
            self.clear()
            return None
        return self.intent

    def clear(self) -> None:
        self.intent = None
        self.deadline_monotonic = None
