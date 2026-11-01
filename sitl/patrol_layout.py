"""The Gazebo survey-patrol world in one place (north, east metres in the shared frame).

The world generator, the launcher and the console flags are all derived from this module so they cannot drift apart.
Survey points are the six hexagon sectors of the pair formations (`pairs` uses 0, 2, 4; `pairs_b` uses 1, 3, 5).
Poles are real obstacles in Gazebo and in the drones' simulated range sensor; trees are scenery only.

    python3 -m sitl.patrol_layout env            # shell exports for sitl/launch_gazebo_world.sh
    python3 -m sitl.patrol_layout console-args   # extra arguments for sitl.swarm_console / sitl.fleet_ctl
"""
from __future__ import annotations

import argparse

from sitl.formation_library import pair_sector_centres

PAIRS = 3
PATROL_SIDE_M = 10.0
SENSE_RANGE_M = 4.0
# On the patrol's first edge (north +5 m, from east +5 to -5): the leading drone has to go round it.
POLE_ON_ROUTE = (5.0, -1.5, 0.6)
# Off the route, but inside the sensing range of the pairs that survey sector 3: detected, not in the way.
POLE_DECOY = (-13.0, -3.0, 0.5)
POLES = (POLE_ON_ROUTE, POLE_DECOY)
TREES = ((16.0, -14.0), (18.0, 12.0), (-16.0, 14.0), (-19.0, -6.0), (10.0, 20.0), (-3.0, -20.0), (22.0, 2.0), (-10.0, -18.0))


def survey_points() -> list[tuple[float, float]]:
    return [(float(n), float(e)) for n, e in pair_sector_centres(PAIRS)]


def shell_env() -> str:
    poles = " ".join(f"{n:g},{e:g},{r:g}" for n, e, r in POLES)
    markers = " ".join(f"{n:.2f},{e:.2f}" for n, e in survey_points())
    trees = " ".join(f"{n:g},{e:g}" for n, e in TREES)
    return f'export CAMERA=1 OBSTACLES="{poles}" MARKERS="{markers}" TREES="{trees}"'


def console_args() -> str:
    sensed = " ".join(f"--sensed-obstacle={n:g},{e:g},{r:g}" for n, e, r in POLES)
    return f"{sensed} --sense-range {SENSE_RANGE_M:g}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("what", choices=("env", "console-args"))
    args = parser.parse_args()
    print(shell_env() if args.what == "env" else console_args())


if __name__ == "__main__":
    main()
