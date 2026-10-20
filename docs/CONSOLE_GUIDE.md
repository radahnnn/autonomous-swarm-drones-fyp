# Swarm flight console guide

One console replaces the former `interactive_flight_console.py` (3 drones), `interactive_flight_console_5drones.py`
and `self_healing_swarm_console.py`. Code: `sitl/swarm_console.py`; formation slots: `sitl/formation_library.py`;
ports, homes and labels: `sitl/fleet.py`; arming/takeoff helpers: `sitl/flight_prep.py`.

```bash
python3 -m sitl.swarm_console --drones 5 --gazebo        # 3D (needs cinewhoop_5drones.sdf running)
python3 -m sitl.swarm_console --drones 3 --standalone    # headless 2D SITL, fast arming
python3 -m sitl.swarm_console --drones 5 --auto-launch   # take off immediately
sitl/launch_console.sh --drones 5 --gazebo               # same, in its own terminal window
```
Options: `-n/--drones 1..10` (default 5), `--gazebo` / `--standalone` (auto-detected if omitted), `--restart`, `--auto-launch`,
`--ekf-timeout SECONDS` (default 300; 900 in Gazebo, which can run at only 5-30% real time), `--evidence-dir PATH`,
and `--control-mode centralized|hybrid` (default centralized). Hybrid is experimental; see [live hybrid guidance](HYBRID_LIVE_GUIDANCE.md).
Every run writes a dated `runs/console/` evidence folder; see [criteria and file format](RUN_EVIDENCE.md).

## Commands
| Command | Action |
|---|---|
| `launch` | arm (EKF-gated), take off and form up every grounded drone together |
| `w s a d u j [metres]` | move North / South / West / East / up / down (e.g. `w 10`); `c` returns to centre |
| `v l k x o e tri` | morph to V, line, column, diamond, ring, echelon, triangle (Hungarian slot assignment) |
| `p` | square patrol demo |
| `f <id>` / `r <id>` / `lift <id>` | fail a drone (leader re-election + reconfiguration) / revive and rejoin / launch one drone |
| `status` | role and position table |
| `q` | synchronized landing |

## Conventions (shared with launchers and Gazebo worlds)
- Drone i = ArduPilot instance `-I<i>`, SYSID i+1. Existing MAVProxy-forwarded instances may be reached on UDP 14552 + 10*i; instances launched directly by this console expose TCP 5760 + 10*i. Gazebo FDM uses port 9002 + 10*i.
- Gazebo: all instances share one home (the world origin); the world files spawn drones on their V slots
  (`cinewhoop_3drones.sdf`, `cinewhoop_5drones.sdf`). Other swarm sizes work standalone; Gazebo needs a matching world.
- Standalone: each drone's home is its own V slot, so the swarm starts on the ground in formation.
- Slots are offsets [N, E, D] from the centroid; every style keeps drones at least 2.5 m apart for 1-10 drones
  (tested). Hand-tuned tables for up to 5 drones are preserved; larger swarms use generators.

## Old command mapping
Old 3/5-drone consoles took off automatically and used `t` = triangle, `x` = diamond (5 drones). Now: use `launch`
(or `--auto-launch`), `tri` = triangle, `x` = diamond, `o` = ring (was circle/pentagon). `t <id>` still means lift.

## Verification status
Tested live in standalone SITL with five drones (takeoff, formation changes, movement, square patrol, injected failure and
re-election, revive and rejoin, confirmed landing). The consolidated demo suite passed 161 tests on 12 October 2026.
On 14 October 2026, the five-drone Gazebo world rendered on the NVIDIA GPU and all five SITL links connected, but EKF
readiness did not converge sufficiently for this console to take off. Five-drone standalone runs produced evidence and
landed, but both centralized failure/rejoin and experimental hybrid runs failed at least one logged acceptance criterion.
The earlier 12 October demo run remains the observed functional 2D route; use the default centralized mode for a demo.
