"""Versioned, shared high-level mission intent for local swarm agents."""

from dataclasses import asdict, dataclass
import math
from typing import Any, Dict


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

    def to_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result["goal_ne"] = list(self.goal_ne)
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
        return cls(sequence, issued_at, expires_at, goal_ne, formation, target_alt_m)


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
