"""Offline 3-D simulator-truth audit from DataFlash SIM logs: `python3 -m tools.bin_truth_audit <run_dir>...`.

Aligns airborne `true_position_ne` samples in `evidence/*/events.jsonl` with ArduPilot's DataFlash `SIM`
records (`Lat`, `Lng`, `Alt`) to recover true simulator altitude (`Alt - DATUM_ALT`) for older runs logged
before `SIM_STATE` (message ID 108) was requested live. Read-only; never modifies files under `runs/`.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import numpy as np
from pymavlink import mavutil

from sitl.common_frame import CommonCoordinateFrame
from sitl.fleet import DATUM_ALT, DATUM_LAT, DATUM_LON
from sitl.separation_geometry import horizontal_equivalent_gap

ARCHIVE = Path(os.environ.get("FYP_BIN_ARCHIVE", "/media/drone/SSDSAM/FYP_Archive/Runs_BIN_2026-11-04"))


def load_sim_trajectory(bin_paths: Iterable[Path], frame: Optional[CommonCoordinateFrame] = None) -> np.ndarray:
    """Return an (M, 3) array of [north_m, east_m, alt_rel_datum_m] from DataFlash `SIM` messages."""
    frame = frame or CommonCoordinateFrame(DATUM_LAT, DATUM_LON, DATUM_ALT)
    rows: List[List[float]] = []
    for path in sorted(bin_paths):
        log = mavutil.mavlink_connection(str(path), dialect="ardupilotmega")
        while True:
            msg = log.recv_match(type="SIM")
            if msg is None:
                break
            ned = frame.gps_to_global_ned(float(msg.Lat), float(msg.Lng), float(msg.Alt) - frame.alt0)
            rows.append([float(ned[0]), float(ned[1]), float(msg.Alt) - frame.alt0])
    return np.asarray(rows, dtype=float) if rows else np.zeros((0, 3), dtype=float)


def _find_sim_root(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
    if any(run_dir.glob("evidence/*/sitl-instances/drone-*/logs/*.BIN")):
        return run_dir
    try:
        archived = ARCHIVE / run_dir.resolve().relative_to(Path.cwd().resolve())
    except ValueError:
        archived = None
    if archived is not None and archived.exists():
        return archived
    return run_dir


def audit_trajectories(events: Iterable[Dict[str, Any]], sim_by_drone: Dict[int, np.ndarray], *,
                       gate_m: float = 2.5, vertical_radius_m: float = 2.0,
                       max_dt_s: float = 1.5) -> Dict[str, Any]:
    """Compute true 2-D horizontal, 3-D Euclidean, and ellipsoid minima from status events + SIM arrays."""
    latest: Dict[int, Dict[str, Any]] = {}
    min_h = min_3d = min_ell = math.inf
    max_match_err = 0.0
    worst_pair_h: Optional[Dict[str, Any]] = None
    pair_samples = 0

    for event in events:
        if event.get("kind") != "agent_status":
            continue
        drone = int(event["drone_id"])
        rec = dict(event)
        true_ne = rec.get("true_position_ne")
        if rec.get("airborne") and true_ne is not None:
            true_alt = rec.get("true_altitude_m")
            if true_alt is not None and math.isfinite(float(true_alt)):
                rec["_true_alt"] = float(true_alt)
            elif drone in sim_by_drone and len(sim_by_drone[drone]) > 0:
                arr = sim_by_drone[drone]
                dists = np.hypot(arr[:, 0] - float(true_ne[0]), arr[:, 1] - float(true_ne[1]))
                idx = int(np.argmin(dists))
                max_match_err = max(max_match_err, float(dists[idx]))
                rec["_true_alt"] = float(arr[idx, 2])
        latest[drone] = rec
        now = float(rec.get("timestamp", 0.0))
        airborne = {
            i: s for i, s in latest.items()
            if (s.get("airborne") and s.get("true_position_ne") is not None
                and "_true_alt" in s and abs(now - float(s.get("timestamp", 0.0))) <= max_dt_s)
        }
        ids = sorted(airborne)
        if len(ids) < 2:
            continue
        pair_samples += 1
        for idx_a, a in enumerate(ids):
            pa = airborne[a]["true_position_ne"]
            za = airborne[a]["_true_alt"]
            for b in ids[idx_a + 1:]:
                pb = airborne[b]["true_position_ne"]
                zb = airborne[b]["_true_alt"]
                h = math.hypot(float(pa[0]) - float(pb[0]), float(pa[1]) - float(pb[1]))
                v = abs(float(za) - float(zb))
                d3 = math.hypot(h, v)
                ell = horizontal_equivalent_gap(h, v, gate_m, vertical_radius_m)
                if h < min_h:
                    min_h = h
                    worst_pair_h = {
                        "pair": [a + 1, b + 1],
                        "horizontal_m": round(h, 3),
                        "vertical_m": round(v, 3),
                        "gap_3d_m": round(d3, 3),
                        "ellipsoid_m": round(ell, 3),
                    }
                min_3d = min(min_3d, d3)
                min_ell = min(min_ell, ell)

    return {
        "pair_samples": pair_samples,
        "true_min_horizontal_m": None if not math.isfinite(min_h) else round(min_h, 3),
        "true_min_3d_m": None if not math.isfinite(min_3d) else round(min_3d, 3),
        "true_min_ellipsoid_m": None if not math.isfinite(min_ell) else round(min_ell, 3),
        "max_match_error_m": round(max_match_err, 4),
        "at_min_horizontal": worst_pair_h,
    }


def _iter_events(events_path: Path) -> Iterable[Dict[str, Any]]:
    with events_path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def audit_run(run_dir: Path, *, gate_m: float = 2.5, vertical_radius_m: float = 2.0) -> Dict[str, Any]:
    run_dir = Path(run_dir)
    events_files = sorted(run_dir.glob("evidence/*/events.jsonl"))
    if not events_files:
        return {"run": run_dir.name, "error": "no events.jsonl"}
    sim_root = _find_sim_root(run_dir)
    frame = CommonCoordinateFrame(DATUM_LAT, DATUM_LON, DATUM_ALT)
    sim_by_drone: Dict[int, np.ndarray] = {}
    for drone_dir in sorted(sim_root.glob("evidence/*/sitl-instances/drone-*")):
        try:
            drone_id = int(drone_dir.name.split("-")[1]) - 1
        except (IndexError, ValueError):
            continue
        bins = sorted(drone_dir.glob("logs/*.BIN"))
        if bins:
            sim_by_drone[drone_id] = load_sim_trajectory(bins, frame=frame)
    result = audit_trajectories(
        _iter_events(events_files[0]), sim_by_drone,
        gate_m=gate_m, vertical_radius_m=vertical_radius_m,
    )
    result["run"] = run_dir.name
    return result


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        r = audit_run(Path(arg))
        if "error" in r:
            print(f"{r['run']}: {r['error']}")
            continue
        info = r["at_min_horizontal"] or {}
        print(
            f"{r['run']:18s} | h={r['true_min_horizontal_m']} m | 3d={r['true_min_3d_m']} m | "
            f"ellipsoid={r['true_min_ellipsoid_m']} m | match_err<={r['max_match_error_m']} m | "
            f"worst_h={info}"
        )
