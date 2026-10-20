#!/usr/bin/env python3
"""Paired five-drone centralized/decentralized control comparison.

The same initial state, ArduPilot-SITL-fitted dynamics profile, mission, wireless
impairments and random seed are used for both modes in each trial.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.stats import t

from simulator.engine import SwarmSimulation
from swarm_core.drone import Drone
from swarm_core.formations import FormationType


N_DRONES = 5
DT = 0.05
DURATION_S = 14.0
MOVE_START_S = 2.0
MOVE_END_S = 8.0
STEADY_START_S = 11.0
MOVE_SPEED_M_S = 0.6
PACKET_LOSS = 0.10
LATENCY_S = 0.05
PROFILE = "sitl_default_quad"


def _initial_positions(seed):
    rng = np.random.default_rng(seed)
    # Jitter around the same initial V geometry while retaining safe clearance.
    offsets = np.array([[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0],
                        [-6.0, -6.0], [-6.0, 6.0]])
    return offsets + rng.normal(0.0, 0.12, size=offsets.shape)


def run_trial(mode, seed):
    positions = _initial_positions(seed)
    drones = [Drone(i, positions[i], profile=PROFILE) for i in range(N_DRONES)]
    sim = SwarmSimulation(
        drones, control_mode=mode, comm_range=12.0,
        packet_loss_rate=PACKET_LOSS, latency_mean=LATENCY_S,
        dt=DT, gps_noise_std=0.08, gps_common_mode_fraction=0.60,
        seed=seed, profile=PROFILE,
    )
    sim.set_formation(FormationType.V_SHAPE, centroid=np.zeros(2))
    steps = int(DURATION_S / DT)
    for step in range(steps):
        now = step * DT
        if MOVE_START_S <= now < MOVE_END_S:
            sim.centroid_velocity = np.array([MOVE_SPEED_M_S, 0.0])
            sim.centroid_target += sim.centroid_velocity * DT
        else:
            sim.centroid_velocity = np.zeros(2)
        sim.step()

    steady = [row for row in sim.metrics.history if row.time >= STEADY_START_S]
    all_errors = np.asarray([row.formation_error for row in sim.metrics.history])
    steady_errors = np.asarray([row.formation_error for row in steady])
    return {
        "mode": mode,
        "seed": seed,
        "steady_mean_error_m": float(np.mean(steady_errors)),
        "steady_p95_error_m": float(np.percentile(steady_errors, 95)),
        "mission_mean_error_m": float(np.mean(all_errors)),
        "final_error_m": float(all_errors[-1]),
        "minimum_separation_m": float(min(row.min_inter_drone_dist for row in sim.metrics.history)),
        "any_collision": bool(any(row.collision_occurred for row in sim.metrics.history)),
        "steady_samples": len(steady),
    }


def _mean_ci(values):
    arr = np.asarray(values, dtype=float)
    mean = float(np.mean(arr))
    if len(arr) < 2:
        return {"mean": mean, "ci95_low": None, "ci95_high": None}
    half = float(t.ppf(0.975, len(arr) - 1) * np.std(arr, ddof=1) / np.sqrt(len(arr)))
    return {"mean": mean, "ci95_low": mean - half, "ci95_high": mean + half}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20261020)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.trials < 2:
        parser.error("--trials must be at least 2 to estimate a confidence interval")

    trials = []
    for index in range(args.trials):
        seed = args.seed + index
        pair = {mode: run_trial(mode, seed) for mode in ("centralized", "decentralized")}
        trials.append(pair)
        print(f"trial {index + 1}/{args.trials} seed={seed}: "
              f"central={pair['centralized']['steady_mean_error_m']:.3f}m, "
              f"decentral={pair['decentralized']['steady_mean_error_m']:.3f}m")

    metrics = ("steady_mean_error_m", "steady_p95_error_m", "mission_mean_error_m",
               "final_error_m", "minimum_separation_m")
    aggregate = {}
    for mode in ("centralized", "decentralized"):
        rows = [pair[mode] for pair in trials]
        aggregate[mode] = {metric: _mean_ci([row[metric] for row in rows]) for metric in metrics}
        aggregate[mode]["collision_trials"] = sum(row["any_collision"] for row in rows)
    deltas = {
        metric: _mean_ci([pair["decentralized"][metric] - pair["centralized"][metric]
                          for pair in trials])
        for metric in metrics
    }
    report = {
        "title": "Paired 5-drone architecture comparison",
        "trials": args.trials,
        "seeds": [args.seed + i for i in range(args.trials)],
        "conditions": {
            "same_initial_state_per_seed": True,
            "dynamics_profile": PROFILE,
            "mission": "hold 2s, move north 0.6m/s for 6s, hold 6s",
            "packet_loss_probability": PACKET_LOSS,
            "mean_peer_and_coordinator_latency_s": LATENCY_S,
            "gps_noise_std_m": 0.08,
            "dt_s": DT,
            "steady_window_s": [STEADY_START_S, DURATION_S],
        },
        "metrics": aggregate,
        "paired_delta_decentralized_minus_centralized": deltas,
        "trial_results": trials,
        "limitations": [
            "Software point-mass simulation only; not SITL or physical flight.",
            "10 repeated seeds are preliminary and do not characterize all radio conditions.",
            "Collision checks use simulated states at discrete time steps.",
        ],
    }
    encoded = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
        print(f"Saved paired results: {args.output}")
    else:
        print(encoded)


if __name__ == "__main__":
    main()
