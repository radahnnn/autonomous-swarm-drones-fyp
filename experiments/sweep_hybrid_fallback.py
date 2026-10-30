"""Hybrid fallback-strategy and degrade-timeout sweep (numerical engine; not SITL or hardware).

Scenarios:
  burst_outage   - 10% loss, Gilbert-Elliott bursts, two 3 s coordinator outages (as compare_three_modes)
  permanent_loss - coordinator link lost at t = 5 s and never restored while the target keeps moving
Reports separation against the 2.5 m gate, steady error, mode switching, and (permanent_loss) how
far the swarm keeps following the mission.

  PYTHONPATH=. python3 experiments/sweep_hybrid_fallback.py --drones 10 --seeds 30 --output out.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

from simulator.engine import SwarmSimulation
from swarm_core.drone import Drone
from swarm_core.formations import FormationGenerator, FormationType

GATE_M = 2.5
STRATEGIES = ("consensus", "hold_target", "dead_reckon", "hover")
TIMEOUTS_S = (0.5, 0.8, 1.2, 2.0)


def run(mode, seed, strategy, timeout, scenario, count):
    rng = np.random.default_rng(seed)
    offsets = FormationGenerator.get_formation_offsets(FormationType.V_SHAPE, count, spacing=3.0)
    positions = offsets + rng.normal(0, .12, offsets.shape)
    drones = [Drone(i, positions[i], profile="assumed_baseline") for i in range(count)]
    sim = SwarmSimulation(drones, control_mode=mode, profile="assumed_baseline", dt=.05,
                          comm_range=12, packet_loss_rate=.1, latency_mean=.05, gps_noise_std=.08,
                          gps_common_mode_fraction=.6, seed=seed,
                          use_gilbert_elliott=scenario == "burst_outage",
                          fallback_strategy=strategy, separation_minimum_m=GATE_M)
    sim.set_formation(FormationType.V_SHAPE, centroid=np.zeros(2), spacing=3)
    if timeout is not None:
        sim.hybrid_ctrl.degrade_timeout = timeout
    for step in range(600):
        now = step * .05
        if scenario == "burst_outage":
            sim.set_coordinator_link(not (5 <= now < 8 or 15 <= now < 18))
        else:
            sim.set_coordinator_link(now < 5)
        sim.centroid_velocity = np.array([.4, 0]) if 2 <= now < 20 else np.zeros(2)
        sim.centroid_target += sim.centroid_velocity * .05
        sim.step()
    history = sim.metrics.history
    errors = [r.formation_error for r in history if r.time >= 25]
    centroid = np.mean([d.position for d in drones], axis=0)
    return dict(
        seed=seed, min_separation_m=float(min(r.min_inter_drone_dist for r in history)),
        steady_p95_error_m=float(np.percentile(errors, 95)),
        switches=int(sum(sim.hybrid_ctrl.switch_counts.values())) if mode == "hybrid" else 0,
        mission_lag_m=float(np.linalg.norm(centroid - sim.centroid_target[:2])),
    )


def summarise(rows):
    sep = [r["min_separation_m"] for r in rows]
    return dict(
        trials=len(rows), mean_min_separation_m=float(np.mean(sep)), worst_min_separation_m=float(min(sep)),
        below_gate=int(sum(1 for s in sep if s < GATE_M)),
        worst_steady_p95_error_m=float(max(r["steady_p95_error_m"] for r in rows)),
        mean_switches=float(np.mean([r["switches"] for r in rows])),
        mean_mission_lag_m=float(np.mean([r["mission_lag_m"] for r in rows])),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--drones", type=int, default=10)
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--first-seed", type=int, default=20261020)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    seeds = range(args.first_seed, args.first_seed + args.seeds)
    report = {"scope": "numerical engine, assumed_baseline; not SITL/hardware", "gate_m": GATE_M,
              "drones": args.drones, "first_seed": args.first_seed, "cells": {}}
    for scenario in ("burst_outage", "permanent_loss"):
        base = [run("centralized", s, "consensus", None, scenario, args.drones) for s in seeds]
        report["cells"][f"{scenario}|centralized"] = summarise(base)
        for strategy in STRATEGIES:
            for timeout in TIMEOUTS_S:
                rows = [run("hybrid", s, strategy, timeout, scenario, args.drones) for s in seeds]
                report["cells"][f"{scenario}|hybrid|{strategy}|{timeout}"] = summarise(rows)
        print(scenario, "done", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
