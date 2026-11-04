"""Count ArduPilot EKF warnings in a run's dataflash logs: `python3 -m tools.ekf_events <run_dir> [--list]`.

Reads evidence/*/sitl-instances/drone-NN/logs/*.BIN (ArduPilot MSG records) and counts the messages that mark an
estimator upset: emergency yaw resets, IMU lane switches, and other "EKF3 ..." warnings. Used to judge the EKF tuning
(user-approved item 7a, 2026-11-04): fewer resets per run with the same injected disturbances.
"""
from __future__ import annotations

import re
import sys
from collections import Counter
from pathlib import Path

from pymavlink import mavutil

import os

# Older dataflash logs were moved off the system disk (2026-11-04) to this archive, mirroring the repo's runs/ paths.
ARCHIVE = Path(os.environ.get("FYP_BIN_ARCHIVE", "/media/drone/SSDSAM/FYP_Archive/Runs_BIN_2026-11-04"))

NORMAL = re.compile(r"yaw alignment complete|origin set|is using GPS|initialised|started relative|tilt alignment", re.I)
CATEGORIES = (("yaw_reset", re.compile(r"yaw reset", re.I)), ("lane_switch", re.compile(r"lane switch", re.I)),
              ("ekf_other", re.compile(r"\bEKF", re.I)))


def drone_events(log_path: Path):
    """[(time_s, category, text)] for one dataflash log."""
    log = mavutil.mavlink_connection(str(log_path), dialect="ardupilotmega")
    events = []
    while True:
        msg = log.recv_match(type="MSG")
        if msg is None:
            break
        text = str(getattr(msg, "Message", ""))
        if NORMAL.search(text):
            continue                                   # start-up messages, not upsets
        for name, pattern in CATEGORIES:
            if pattern.search(text):
                events.append((getattr(msg, "TimeUS", 0) / 1e6, name, text))
                break
    return events


def run_events(run_dir: Path):
    """{drone_number: [(time_s, category, text)]} over all of a run's dataflash logs."""
    result = {}
    run_dir = Path(run_dir)
    if not any(run_dir.glob("evidence/*/sitl-instances/drone-*/logs/*.BIN")):
        try:
            archived = ARCHIVE / run_dir.resolve().relative_to(Path.cwd().resolve())
        except ValueError:
            archived = None
        if archived is not None and archived.exists():
            run_dir = archived
    for drone_dir in sorted(run_dir.glob("evidence/*/sitl-instances/drone-*")):
        number = int(drone_dir.name.split("-")[1])
        events = []
        for log_path in sorted(drone_dir.glob("logs/*.BIN")):
            events += drone_events(log_path)
        result[number] = events
    return result


def summary(run_dir: Path) -> Counter:
    counts = Counter()
    for events in run_events(run_dir).values():
        counts.update(category for _, category, _ in events)
    return counts


if __name__ == "__main__":
    listing = "--list" in sys.argv
    for arg in (a for a in sys.argv[1:] if not a.startswith("--")):
        per_drone = run_events(Path(arg))
        total = Counter(c for events in per_drone.values() for _, c, _ in events)
        logs = sum(len(e) >= 0 for e in per_drone.values())
        print(f"{arg}: {dict(total) or 'no EKF warnings'} ({logs} drones)")
        if listing:
            for number, events in per_drone.items():
                for t, category, text in events:
                    print(f"  D{number} t={t:7.1f}s {category:11s} {text}")
