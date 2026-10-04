"""
Reproduce every Monte Carlo experiment (stdout tables, PNG figures, raw per-seed CSVs).

    python experiments/run_all.py                       # default (fitted SITL) dynamics
    SWARM_DYNAMICS_PROFILE=assumed python experiments/run_all.py
    SWARM_NUM_SEEDS=5 python experiments/run_all.py     # quick smoke run

SITL-based validation scripts (validate_*_sitl.py) need ArduPilot and are not included.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = [
    "run_demo.py",
    "test_network_sweep.py",
    "test_burst_outage_sweep.py",
    "test_gps_noise_sweep.py",
    "test_feedforward_ablation.py",
]

if __name__ == "__main__":
    root = os.path.dirname(HERE)
    for s in SCRIPTS:
        print(f"\n##### {s} #####", flush=True)
        subprocess.run([sys.executable, os.path.join(HERE, s)], cwd=root, check=True)
