"""Wind and GPS-noise stress sweep (SIMULATION evidence, pure Python simulator).

For each control mode and each disturbance level, run N seeded 10-drone trials (V formation, translating, then
morphing to a line) and report the distribution of the minimum true inter-drone distance against the 2.5 m gate.
Disturbances: Gauss-Markov GPS error (common + independent) and a Gauss-Markov wind gust (mean + swarm-wide +
per-drone). The wind is an assumed acceleration model (see swarm_core/wind.py), not a fitted one.

    python3 -m experiments.stress_wind_gps --seeds 30 --out experiments/results/stress_wind_gps.json
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from simulator.engine import SwarmSimulation
from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from swarm_core.wind import WindModel

GATE_M = 2.5
LEVELS = {  # name: (gps sigma m, mean wind m/s, gust m/s)  -- wind speeds are converted with the drag coefficient
    "calm": (0.04, 0.0, 0.0),
    "gps_1.5m": (1.5, 0.0, 0.0),
    "wind_3": (0.04, 3.0, 1.0),
    "gps_1.5m+wind_3": (1.5, 3.0, 1.0),
    "gps_2.5m+wind_5": (2.5, 5.0, 2.0),
}


def run_trial(mode: str, level: str, seed: int, n: int = 10, duration_s: float = 24.0, overrides=None) -> dict:
    """``overrides`` maps (controller attribute, parameter) to a value, e.g. ('decentral_ctrl', 'k_sep'): 6.0."""
    sigma, mean_wind, gust = LEVELS[level]
    rng = np.random.default_rng(seed)
    positions = []
    while len(positions) < n:
        candidate = rng.uniform(-8.0, 8.0, size=2)
        if all(np.linalg.norm(candidate - p) >= 3.0 for p in positions):
            positions.append(candidate)
    drones = [Drone(drone_id=i, initial_position=positions[i], measurement_noise_std=0.0) for i in range(n)]
    probe = SwarmSimulation(drones, control_mode=mode, seed=seed, gps_noise_std=sigma, separation_minimum_m=GATE_M)
    drag = float(probe.profile.get("drag_coeff"))
    wind = None
    if mean_wind or gust:
        wind = WindModel.from_speed((mean_wind, 0.0), gust, drag, seed=seed + 1)
    sim = SwarmSimulation(drones, control_mode=mode, seed=seed, gps_noise_std=sigma, separation_minimum_m=GATE_M,
                          wind=wind)
    for (controller, name), value in (overrides or {}).items():
        setattr(getattr(sim, controller), name, float(value))
    sim.set_formation(FormationType.V_SHAPE, centroid=np.zeros(2))
    velocity = np.array([0.5, 0.2])
    centroid = np.zeros(2)
    for step in range(int(duration_s / sim.dt)):
        t = step * sim.dt
        centroid = centroid + velocity * sim.dt
        if t >= 12.0 and sim.current_formation == FormationType.V_SHAPE:
            sim.set_formation(FormationType.LINE, centroid=centroid.copy())
        else:
            sim.centroid_target, sim.centroid_velocity = centroid.copy(), velocity.copy()
        sim.step()
    summary = sim.metrics.get_summary()
    # The first seconds are the spread-out start converging to the formation; judge the settled part only.
    settled = [s.min_inter_drone_dist for s in sim.metrics.history if s.time >= 6.0]
    return {"min_gap_m": float(min(settled)), "steady_error_m": float(summary.get("steady_state_error_m", 0.0)),
            "collision": bool(summary.get("any_collision", 0.0))}


def percentile(values, q):
    return float(np.percentile(values, q))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--drones", type=int, default=10)
    parser.add_argument("--modes", default="centralized,decentralized,hybrid")
    parser.add_argument("--levels", default=",".join(LEVELS))
    parser.add_argument("--out", type=Path, default=Path("experiments/results/stress_wind_gps.json"))
    args = parser.parse_args()
    results = {}
    started = time.time()
    for mode in args.modes.split(","):
        for level in args.levels.split(","):
            trials = [run_trial(mode, level, 1000 + i, args.drones) for i in range(args.seeds)]
            gaps = [t["min_gap_m"] for t in trials]
            results[f"{mode}/{level}"] = {
                "min_gap_p05": percentile(gaps, 5), "min_gap_median": percentile(gaps, 50), "min_gap_worst": min(gaps),
                "gate_pass_rate": sum(g >= GATE_M for g in gaps) / len(gaps),
                "collisions": sum(t["collision"] for t in trials),
                "steady_error_median": percentile([t["steady_error_m"] for t in trials], 50), "trials": len(trials)}
            r = results[f"{mode}/{level}"]
            print(f"{mode:<13}{level:<18} median {r['min_gap_median']:.2f} m  p05 {r['min_gap_p05']:.2f}  "
                  f"worst {r['min_gap_worst']:.2f}  pass {r['gate_pass_rate']:.0%}  collisions {r['collisions']}",
                  flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"drones": args.drones, "gate_m": GATE_M, "results": results,
                                    "elapsed_s": time.time() - started}, indent=2))


if __name__ == "__main__":
    main()
