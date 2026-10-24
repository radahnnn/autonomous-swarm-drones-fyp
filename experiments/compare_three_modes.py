"""Paired software-only comparison; assumed dynamics, never a hardware/SITL claim."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import numpy as np
from scipy.stats import t
from simulator.engine import SwarmSimulation
from swarm_core.drone import Drone
from swarm_core.formations import FormationGenerator, FormationType

MODES = ('centralized', 'decentralized', 'hybrid')

def ci(values):
    a = np.asarray(values, dtype=float)
    half = float(t.ppf(.975, len(a)-1) * a.std(ddof=1) / np.sqrt(len(a)))
    return dict(mean=float(a.mean()), low=float(a.mean()-half), high=float(a.mean()+half))


def trial(mode, seed, scenario, count, separation_guard_m=2.5):
    rng = np.random.default_rng(seed)
    offsets = FormationGenerator.get_formation_offsets(FormationType.V_SHAPE, count, spacing=3.0)
    positions = offsets + rng.normal(0, .12, offsets.shape)
    drones = [Drone(i, positions[i], profile='assumed_baseline') for i in range(count)]
    sim = SwarmSimulation(drones, control_mode=mode, profile='assumed_baseline', dt=.05,
                         comm_range=12, packet_loss_rate=.1, latency_mean=.05,
                         gps_noise_std=.08, gps_common_mode_fraction=.6, seed=seed,
                         use_gilbert_elliott=scenario == 'burst_repeated_outage',
                         fallback_strategy='hold_target',
                         separation_minimum_m=separation_guard_m)
    sim.set_formation(FormationType.V_SHAPE, centroid=np.zeros(2), spacing=3)
    for step in range(600):
        now = step * .05
        outage = scenario != 'nominal_impaired' and (5 <= now < 8 or 15 <= now < 18)
        sim.set_coordinator_link(not outage)
        sim.centroid_velocity = np.array([.4, 0]) if 2 <= now < 20 else np.zeros(2)
        sim.centroid_target += sim.centroid_velocity * .05
        sim.step()
    rows = sim.metrics.history
    errors = [r.formation_error for r in rows if r.time >= 25]
    sep = min(r.min_inter_drone_dist for r in rows)
    return dict(seed=seed, drones=count, mode=mode, scenario=scenario,
                steady_mean_error_m=float(np.mean(errors)),
                steady_p95_error_m=float(np.percentile(errors,95)),
                minimum_separation_m=float(sep),
                pass_criteria=bool(sep >= 2.5 and np.percentile(errors,95) <= 1),
                switches=sum(sim.hybrid_ctrl.switch_counts.values()) if mode == 'hybrid' else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials',type=int,default=10)
    parser.add_argument('--drones',type=int,default=5,help='swarm size, 1..10')
    parser.add_argument('--separation-guard-m',type=float,default=2.5,
                        help='local separation guard activation distance in metres')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.trials < 2: parser.error('need at least two trials')
    if not 1 <= args.drones <= 10: parser.error('--drones must be between 1 and 10')
    if not np.isfinite(args.separation_guard_m) or args.separation_guard_m <= 0:
        parser.error('--separation-guard-m must be finite and positive')
    results=[]; summary={}
    for scenario in ('nominal_impaired','repeated_outage','burst_repeated_outage'):
        for seed in range(20261020,20261020+args.trials):
            for mode in MODES:
                results.append(trial(mode,seed,scenario,args.drones,args.separation_guard_m))
        summary[scenario]={}
        for mode in MODES:
            rows=[r for r in results if r['scenario']==scenario and r['mode']==mode]
            summary[scenario][mode]={k:ci([r[k] for r in rows]) for k in
                 ('steady_mean_error_m','steady_p95_error_m','minimum_separation_m')}
            summary[scenario][mode]['passes']=sum(r['pass_criteria'] for r in rows)
        print(scenario, {m:summary[scenario][m]['passes'] for m in MODES},flush=True)
    paired = {}
    for scenario in summary:
        paired[scenario] = {}
        for mode in ('decentralized', 'hybrid'):
            rows = [r for r in results if r['scenario']==scenario and r['mode']==mode]
            baseline = {r['seed']:r for r in results if r['scenario']==scenario and r['mode']=='centralized'}
            paired[scenario][mode + '_minus_centralized'] = {
                metric: ci([r[metric]-baseline[r['seed']][metric] for r in rows])
                for metric in ('steady_mean_error_m','minimum_separation_m')}
    revision = subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True).stdout.strip()
    status = subprocess.run(['git','status','--porcelain'],capture_output=True,text=True).stdout
    tracked_diff = subprocess.run(['git','diff','HEAD'],capture_output=True).stdout
    report=dict(paired_differences=paired, scope='Pure Python engine; not console/SITL or hardware validation',
                dynamics='assumed_baseline; avoids invalid old fitted profile',
                reproducibility=dict(seed_start=20261020,trials_per_cell=args.trials,
                                     python=sys.version,platform=platform.platform(),
                                     revision=revision,working_tree_dirty=bool(status.strip()),
                                     tracked_diff_sha256=hashlib.sha256(tracked_diff).hexdigest()),
                conditions=dict(drones=args.drones,spacing_m=3,dt_s=.05,duration_s=30,
                                packet_loss=.1,latency_mean_s=.05,gps_noise_std_m=.08,
                                separation_guard_m=args.separation_guard_m,
                                mission='hold 2s, north .4m/s until 20s, hold to 30s',
                                outages_s=[[5,8],[15,18]],
                                trials_per_cell=args.trials),
                criteria=dict(minimum_separation_m=2.5,steady_p95_error_m=1),
                safety_guard_minimum_m=args.separation_guard_m,
                caveats=['Identical seeds/configuration, not guaranteed identical delivered packets: distances affect transport.',
                         'All modes use centrally supplied goals; decentralized refers to local control.',
                         'Engine controllers differ from SITL adapter; no reliability claim beyond these trials.'],
                summary=summary,results=results)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')

if __name__ == '__main__': main()
