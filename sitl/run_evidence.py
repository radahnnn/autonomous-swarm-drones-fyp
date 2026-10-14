"""Write one auditable JSONL telemetry stream and summary per console run."""

import json
import math
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


CRITERIA = {
    "connected_drones": "All requested vehicles connect and become EKF ready",
    "landing": "All requested vehicles report disarmed after LAND",
    "minimum_separation_m": 2.5,
    "p95_settled_slot_error_m": 1.0,
}


def _utc():
    return datetime.now(timezone.utc).isoformat()


class RunEvidence:
    def __init__(self, repo_root: Path, drones: int, mode: str, heartbeat_limit_s: float, output_root=None):
        root = Path(output_root) if output_root else repo_root / "runs" / "console"
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.directory = root / run_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stream = (self.directory / "events.jsonl").open("w", encoding="utf-8")
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.drones = drones
        self.heartbeat_limit_s = heartbeat_limit_s
        self.criteria = {**CRITERIA, "maximum_heartbeat_age_s": heartbeat_limit_s}
        self.connected = 0
        self.ekf_ready = False
        self.landing_confirmed = False
        self.samples = 0
        self.full_fleet_samples = 0
        self.watchdog_timeouts = 0
        self.errors = []
        self.min_separation = None
        self.max_heartbeat_age = 0.0
        self.max_airborne = 0
        self.closed = False
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo_root, capture_output=True, text=True)
        status = subprocess.run(["git", "status", "--porcelain"], cwd=repo_root, capture_output=True, text=True)
        self.event("run_start", mode=mode, drones=drones, git_head=head.stdout.strip() if head.returncode == 0 else None,
                   working_tree_dirty=bool(status.stdout.strip()) if status.returncode == 0 else None,
                   criteria=self.criteria)

    def event(self, kind, **fields):
        with self.lock:
            if self.closed:
                return
            row = {"utc": _utc(), "elapsed_s": round(time.monotonic() - self.started, 3), "kind": kind, **fields}
            self.stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
            self.stream.flush()

    def sample(self, positions, targets, active, fresh, heartbeat_age, formation, leader, settled):
        errors = [math.dist(positions[i], targets[i]) if settled and active[i] and fresh[i] and targets[i] is not None else None
                  for i in range(self.drones)]
        pairs = [math.dist(positions[i][:2], positions[j][:2]) for i in range(self.drones) for j in range(i + 1, self.drones)
                 if active[i] and active[j] and fresh[i] and fresh[j]]
        separation = min(pairs) if pairs else None
        ages = [heartbeat_age[i] for i in range(self.drones) if active[i] and heartbeat_age[i] is not None]
        self.event("sample", positions_ned=positions, targets_ned=targets, active=active, telemetry_fresh=fresh,
                   heartbeat_age_s=heartbeat_age, formation=formation, leader=leader, slot_error_m=errors,
                   minimum_separation_m=separation, settled=settled)
        with self.lock:
            self.samples += 1
            if all(active) and all(fresh) and all(age is not None for age in heartbeat_age):
                self.full_fleet_samples += 1
            self.errors.extend(error for error in errors if error is not None)
            self.max_airborne = max(self.max_airborne, sum(active))
            if separation is not None:
                self.min_separation = separation if self.min_separation is None else min(self.min_separation, separation)
            if ages:
                self.max_heartbeat_age = max(self.max_heartbeat_age, *ages)

    def close(self, outcome):
        with self.lock:
            if self.closed:
                return
            p95 = None
            if self.errors:
                ordered = sorted(self.errors)
                p95 = ordered[math.ceil(0.95 * len(ordered)) - 1]
            checks = {
                "connected_and_ekf_ready": self.connected == self.drones and self.ekf_ready,
                "all_airborne_observed": self.max_airborne == self.drones,
                "full_fleet_telemetry_observed": self.full_fleet_samples >= 50,
                "landing_confirmed": self.landing_confirmed,
                "separation_at_least_2_5m": self.min_separation is not None and self.min_separation >= 2.5,
                "p95_slot_error_at_most_1m": p95 is not None and p95 <= 1.0,
                "heartbeat_age_within_limit": self.max_airborne > 0 and self.max_heartbeat_age <= self.heartbeat_limit_s,
                "no_unexpected_watchdog_timeout": self.watchdog_timeouts == 0,
            }
            summary = {"outcome": outcome, "finished_utc": _utc(), "drones": self.drones,
                       "samples": self.samples, "full_fleet_samples": self.full_fleet_samples,
                       "watchdog_timeouts": self.watchdog_timeouts, "settled_slot_error_observations": len(self.errors),
                       "p95_settled_slot_error_m": p95, "minimum_separation_m": self.min_separation,
                       "maximum_heartbeat_age_s": self.max_heartbeat_age, "maximum_airborne": self.max_airborne,
                       "criteria": self.criteria, "checks": checks, "pass": outcome == "landed" and all(checks.values())}
            self.stream.write(json.dumps({"utc": _utc(), "elapsed_s": round(time.monotonic() - self.started, 3),
                                          "kind": "run_end", "summary": summary}, separators=(",", ":")) + "\n")
            self.stream.close()
            (self.directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
            self.closed = True
