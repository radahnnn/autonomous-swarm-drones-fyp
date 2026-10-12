#!/usr/bin/env python3
"""
Collect position-step responses from ArduPilot SITL at several step sizes (and repeats) for structure identification.

    python experiments/collect_steps.py [out_dir] [--sizes 2,5,8] [--repeats 2]

Writes <out_dir>/step_<size>m_r<k>/sitl_step_response.csv. Uses the same test as validate_step_response_sitl.py.
Honours SWARM_PARAMS_EXTRA like that script. Needs ArduPilot SITL (ARDUCOPTER_BIN).
"""
import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from validate_step_response_sitl import run_sitl_step_test  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", nargs="?", default="runs/steps")
    ap.add_argument("--sizes", default="2,5,8")
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--directions", default="N", help="comma list from N (north), E (east), NE (diagonal)")
    args = ap.parse_args()
    dirs = {"N": (1.0, 0.0), "E": (0.0, 1.0), "NE": (1.0, 1.0)}
    for dname in args.directions.split(","):
      for size in [float(s) for s in args.sizes.split(",")]:
        for rep in range(1, args.repeats + 1):
            tag = "" if dname == "N" else f"_{dname}"
            d = Path(args.out_dir) / f"step_{size:g}m{tag}_r{rep}"
            d.mkdir(parents=True, exist_ok=True)
            print(f"\n##### step {size:g} m, repeat {rep} #####", flush=True)
            df = run_sitl_step_test(sim_time_limit=10.0, step_distance=size, direction=dirs[dname])
            if len(df) == 0:
                print("no data recorded", flush=True)
                continue
            df.to_csv(str(d / "sitl_step_response.csv"), index=False)
            print(f"saved {d / 'sitl_step_response.csv'} ({len(df)} frames)", flush=True)


if __name__ == "__main__":
    main()
