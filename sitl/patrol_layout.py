"""The Gazebo survey-patrol world in one place (north, east metres in the shared frame).

The world generator, the launcher and the console flags are all derived from this module so they cannot drift apart.
Survey points are the six hexagon sectors of the pair formations (`pairs` uses 0, 2, 4; `pairs_b` uses 1, 3, 5).
Poles are real obstacles in Gazebo and in the drones' simulated range sensor; trees are scenery only.

    python3 -m sitl.patrol_layout env            # shell exports for sitl/launch_gazebo_world.sh
    python3 -m sitl.patrol_layout console-args   # extra arguments for sitl.swarm_console / sitl.fleet_ctl
"""
from __future__ import annotations

import argparse

from sitl.formation_library import scaled_pair_sector_centres

PAIRS = 3
PATROL_SIDE_M = 10.0
SENSE_RANGE_M = 4.0
# On the patrol's first edge (north +5 m, from east +5 to -5): the leading drone has to go round it.
POLE_ON_ROUTE = (5.0, -1.5, 0.6)
DECOY_RADIUS_M = 0.5


def pole_decoy() -> tuple[float, float, float]:
    """Off the route, 4 m beyond (further south of) one partner slot of the pair that surveys sector 3:
    inside the 4 m sensing range of that drone (detected) but outside its 3.5 m avoidance zone (not in the way).
    Computed from the actual (scaled) slots so it stays right whatever the formation scale."""
    from sitl.formation_library import slots_for
    slots = slots_for("pairs_b", 6)[:, :2]                       # pair 1 (slots 2, 3) holds sector 3, due south
    partner = min(slots[2:4], key=lambda p: p[1])
    return (float(partner[0] - 4.0), float(partner[1]), DECOY_RADIUS_M)


def poles() -> tuple:
    return (POLE_ON_ROUTE, pole_decoy())
TREES = ((16.0, -14.0), (18.0, 12.0), (-16.0, 14.0), (-19.0, -6.0), (10.0, 20.0), (-3.0, -20.0), (22.0, 2.0), (-10.0, -18.0))


def survey_points() -> list[tuple[float, float]]:
    return [(float(n), float(e)) for n, e in scaled_pair_sector_centres(PAIRS)]


def shell_env() -> str:
    pole_text = " ".join(f"{n:g},{e:g},{r:g}" for n, e, r in poles())
    markers = " ".join(f"{n:.2f},{e:.2f}" for n, e in survey_points())
    trees = " ".join(f"{n:g},{e:g}" for n, e in TREES)
    return f'export CAMERA=1 OBSTACLES="{pole_text}" MARKERS="{markers}" TREES="{trees}"'


def console_args() -> str:
    sensed = " ".join(f"--sensed-obstacle={n:g},{e:g},{r:g}" for n, e, r in poles())
    return f"{sensed} --sense-range {SENSE_RANGE_M:g}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("what", choices=("env", "console-args"))
    args = parser.parse_args()
    print(shell_env() if args.what == "env" else console_args())


if __name__ == "__main__":
    main()
