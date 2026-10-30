"""Replay a recorded run in the live dashboard (read-only).

    python3 -m sitl.replay runs/console/<run>/events.jsonl --port 8766 --speed 4

The console already records every agent status (about 1 Hz per drone), mission intent and protocol
event in ``events.jsonl``. This module turns that file back into the session object the dashboard
expects, so the same map, mode buttons, formation view and run-metrics panel can be scrubbed through
afterwards. Nothing is sent to any vehicle; commands are refused.
"""
from __future__ import annotations

import argparse
import bisect
import json
import threading
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional

import numpy as np

from sitl.run_metrics import RunMetrics

_MARKER_LABELS = {
    "operator_command": lambda e: f"command: {e.get('action', '?')}",
    "mode_prepare": lambda e: f"mode switch {e.get('from_mode', '?')} -> {e.get('to_mode', '?')} requested",
    "mode_switch_started": lambda e: "mode switch started",
    "mode_switch_complete": lambda e: f"mode is now {e.get('mode', '?')}",
    "mode_switch_aborted": lambda e: f"mode switch aborted ({e.get('reason', '?')})",
    "mode_switch_rolled_back": lambda e: "mode switch rolled back",
    "transition_prepare": lambda e: "formation transition requested",
    "transition_started": lambda e: "formation transition started",
    "transition_complete": lambda e: "formation transition complete",
    "transition_aborted": lambda e: f"formation transition aborted ({e.get('reason', '?')})",
    "mission_stream": lambda e: "coordinator stream " + ("ON" if e.get("enabled") else "OFF"),
    "agent_rescue_landed": lambda e: f"rescue landing D{e.get('drone_id', -1) + 1}",
    "session_end": lambda e: "session end",
}


def _epoch(stamp: str) -> float:
    return datetime.fromisoformat(stamp).timestamp()


class Recording:
    def __init__(self, path: Path):
        self.path = Path(path)
        if self.path.is_dir():
            candidates = sorted(self.path.glob("*/events.jsonl")) or [self.path / "events.jsonl"]
            self.path = candidates[-1]
        statuses: Dict[int, List[Dict[str, Any]]] = {}
        self.intents: List[Dict[str, Any]] = []
        self.markers: List[Dict[str, Any]] = []
        times: List[float] = []
        with self.path.open(encoding="utf-8") as handle:
            for line in handle:
                try:
                    event = json.loads(line)
                    t = _epoch(event["utc"])
                except (ValueError, KeyError, TypeError):
                    continue
                times.append(t)
                kind = event.get("kind")
                if kind == "agent_status" and isinstance(event.get("drone_id"), int):
                    event["_t"] = t
                    statuses.setdefault(event["drone_id"], []).append(event)
                elif kind == "mission_intent":
                    event["_t"] = t
                    self.intents.append(event)
                elif kind in _MARKER_LABELS:
                    self.markers.append({"_t": t, "kind": kind, "label": _MARKER_LABELS[kind](event)})
        if not statuses:
            raise ValueError(f"{self.path} contains no agent_status events")
        self.statuses = {i: sorted(v, key=lambda e: e["_t"]) for i, v in statuses.items()}
        self._status_times = {i: [e["_t"] for e in v] for i, v in self.statuses.items()}
        self._intent_times = [e["_t"] for e in self.intents]
        self.count = max(statuses) + 1
        first = min(v[0]["_t"] for v in self.statuses.values())
        self.t0 = min(first, min(times)) if times else first
        self.t1 = max(max(v[-1]["_t"] for v in self.statuses.values()), max(times))
        for marker in self.markers:
            marker["t"] = round(marker["_t"] - self.t0, 2)

    def status_at(self, drone: int, t: float, max_age_s: float = 4.0) -> Optional[Dict[str, Any]]:
        times = self._status_times.get(drone)
        if not times:
            return None
        i = bisect.bisect_right(times, t) - 1
        if i < 0 or t - times[i] > max_age_s:
            return None
        return self.statuses[drone][i]

    def intent_at(self, t: float) -> Optional[Dict[str, Any]]:
        i = bisect.bisect_right(self._intent_times, t) - 1
        return self.intents[i] if i >= 0 else None


class ReplaySession:
    """Quacks like an IndependentSwarmConsole for ``dashboard_state`` (read-only)."""

    def __init__(self, recording: Recording, speed: float = 4.0, playing: bool = True):
        self.recording = recording
        self.count = recording.count
        self.lock = threading.RLock()
        self.latest: Dict[int, Dict[str, Any]] = {}
        self.nodes: List[Any] = []
        self.args = SimpleNamespace(obstacle=[], dashboard_port=0, adaptive_swarm=True)
        self.control_mode, self.formation = "hybrid", "v"
        self.goal_ne = np.zeros(2)
        self.target_alt_m = 5.0
        self.intent_enabled = True
        self.external_agents = False
        self.metrics: Optional[RunMetrics] = None
        self.cursor = 0.0                      # seconds from the start of the recording
        self.speed, self.playing = float(speed), bool(playing)
        self._wall = time.time()
        self._metrics_cursor = -1.0
        self._rebuild(0.0)

    # ---- time control
    def _advance(self) -> None:
        now = time.time()
        if self.playing:
            self.cursor = min(self.duration, self.cursor + (now - self._wall) * self.speed)
            if self.cursor >= self.duration:
                self.playing = False
        self._wall = now
        self._rebuild(self.cursor)

    @property
    def duration(self) -> float:
        return self.recording.t1 - self.recording.t0

    def replay_post(self, body: Dict[str, Any]) -> Dict[str, Any]:
        action = body.get("action")
        if action == "play":
            if self.cursor >= self.duration:
                self.cursor = 0.0
            self.playing = True
        elif action == "pause":
            self.playing = False
        elif action == "seek":
            self.cursor = min(self.duration, max(0.0, float(body.get("t", 0.0))))
        elif action == "speed":
            self.speed = min(32.0, max(0.25, float(body.get("speed", 1.0))))
        else:
            raise ValueError("unknown replay action")
        self._wall = time.time()
        self._rebuild(self.cursor)
        return self.replay_info()

    def replay_info(self) -> Dict[str, Any]:
        return {"t": round(self.cursor, 2), "duration": round(self.duration, 2), "playing": self.playing,
                "speed": self.speed, "source": str(self.recording.path),
                "markers": [{"t": m["t"], "label": m["label"], "kind": m["kind"]} for m in self.recording.markers]}

    # ---- state reconstruction
    def _rebuild(self, cursor: float) -> None:
        rec, t = self.recording, self.recording.t0 + cursor
        wall = time.time()
        latest: Dict[int, Dict[str, Any]] = {}
        for drone in range(self.count):
            status = rec.status_at(drone, t)
            if status is None:
                continue
            copy = {k: v for k, v in status.items() if k != "_t"}
            # The dashboard judges freshness against the wall clock: keep the recorded age.
            copy["operator_received_at"] = wall - max(0.0, t - status["_t"])
            latest[drone] = copy
        with self.lock:
            self.latest = latest
        first = latest.get(0) or next(iter(latest.values()), None)
        if first:
            self.control_mode = (first.get("mode_switch") or {}).get("mode") or first.get("control_mode") or self.control_mode
            self.formation = first.get("formation") or self.formation
        intent = rec.intent_at(t)
        if intent:
            self.goal_ne = np.asarray(intent["goal_ne"], dtype=float)
            self.target_alt_m = float(intent.get("target_alt_m", 5.0))
        self._update_metrics(cursor)

    def _update_metrics(self, cursor: float) -> None:
        if self.metrics is None or cursor < self._metrics_cursor:
            self.metrics, self._metrics_cursor = RunMetrics(self.count, sample_period_s=0.5), -1.0
        rec = self.recording
        step = 0.5
        t = max(self._metrics_cursor + step, 0.0)
        while t <= cursor:
            absolute = rec.t0 + t
            states = {}
            for drone in range(self.count):
                status = rec.status_at(drone, absolute)
                if status is not None:
                    states[drone] = {k: v for k, v in status.items() if k != "_t"}
            self.metrics.update(absolute, states)
            for marker in rec.markers:
                if t - step < marker["t"] <= t and marker["kind"].startswith(("mode_switch_", "transition_")):
                    self.metrics.note_event(marker["kind"], absolute)
            self._metrics_cursor = t
            t += step

    # ---- read-only guards for the dashboard's command routes
    def handle(self, line: str) -> bool:
        print(f">> Replay is read-only; ignoring {line!r}")
        return True

    def _set_formation(self, style: str) -> Dict[str, Any]:
        return {"accepted": False, "message": "Replay is read-only."}

    def set_control_mode(self, mode: str) -> Dict[str, Any]:
        return {"accepted": False, "message": "Replay is read-only."}


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Replay a recorded swarm run in the dashboard")
    parser.add_argument("recording", type=Path, help="events.jsonl or a run folder containing it")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--speed", type=float, default=4.0)
    parser.add_argument("--paused", action="store_true", help="start paused at the beginning")
    args = parser.parse_args(argv)
    from sitl.operator_dashboard import start_dashboard
    session = ReplaySession(Recording(args.recording), speed=args.speed, playing=not args.paused)
    print(f">> Replaying {session.recording.path} ({session.duration:.0f} s, {session.count} drones, "
          f"{len(session.recording.markers)} markers) at http://127.0.0.1:{args.port}  (Ctrl+C to stop)")
    start_dashboard(session, args.port)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
