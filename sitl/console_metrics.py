"""Run metrics fed from agent status (separation, takeovers, filter, obstacles, estimate health).

Part of `IndependentSwarmConsole` (independent_swarm_console.py); split out for size, behaviour unchanged."""
from __future__ import annotations
import time


class MetricsMixin:
    def _note_metric(self, kind: str, **data) -> None:
        if self.metrics is not None:
            self.metrics.note_event(kind, time.time(), **data)

    def _update_metrics(self) -> None:
        if self.metrics is None:
            return
        if self._fence_seen is None:
            self._fence_seen = {}
        if self._acting_seen is None:
            self._acting_seen = {}
        if self._safety_seen is None:
            self._safety_seen = {}
        if self._obstacles_seen is None:
            self._obstacles_seen = {}
        if self._estimate_seen is None:
            self._estimate_seen = {}
        with self.lock:
            states = {index: state.copy() for index, state in self.latest.items()}
        self.metrics.update(time.time(), states)
        for index, state in states.items():
            fence = state.get("geofence") or {}
            current = fence.get("state")
            if current == "breach" and self._fence_seen.get(index) != "breach":
                self._note_metric("geofence_breach", drone=index + 1)
            self._fence_seen[index] = current
            acting = bool((state.get("coordinator") or {}).get("acting"))
            if acting and not self._acting_seen.get(index):
                term = (state.get("coordinator") or {}).get("term")
                self._note_metric("coordinator_takeover", drone=index + 1, term=term)
                self.event("coordinator_takeover", drone=index + 1, term=term)
                print(f">> Operator silent: D{index + 1} is now the ACTING COORDINATOR (term {term}).", flush=True)
            elif not acting and self._acting_seen.get(index):
                self.event("coordinator_stepdown", drone=index + 1)
                print(f">> D{index + 1} is no longer acting coordinator.", flush=True)
            self._acting_seen[index] = acting
            filt = bool((state.get("safety_filter") or {}).get("active"))
            if filt and not self._safety_seen.get(index):
                self._note_metric("safety_filter_activation", drone=index + 1)
            self._safety_seen[index] = filt
            health = (state.get("estimate_health") or {}).get("state", "ok")
            if health != "ok" and self._estimate_seen.get(index, "ok") == "ok":
                self._note_metric("estimate_degraded", drone=index + 1, state=health)
                print(f">> D{index + 1}: position estimate {health.upper()} (glitch); peers keep extra distance.", flush=True)
            self._estimate_seen[index] = health
            sensing = state.get("obstacle_sensing") or {}
            known = int(sensing.get("known", 0) or 0)
            if known > self._obstacles_seen.get(index, 0):
                self._note_metric("obstacle_known", drone=index + 1, known=known,
                                  own_detections=sensing.get("detected"), from_peers=sensing.get("learned"))
                self._obstacles_seen[index] = known

    def _any_airborne(self) -> bool:
        with self.lock:
            return any(state.get("airborne") is True for state in self.latest.values())
