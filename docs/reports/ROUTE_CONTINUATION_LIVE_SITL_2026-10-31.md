# Mission continuation without a coordinator — live 10-drone SITL (2026-10-31)

**Evidence level: SITL** — 10 ArduPilot SITL instances, standalone 2D, one host, loopback UDP, `--control-mode hybrid --agent-runtime independent --adaptive-swarm --transition-protocol`. Commits under test: `e473ce4` (run A) and `ad79cd5` (run B). Raw logs (not in Git, checksummed): `runs/independent/route-continuation-live-20261031/`. Run reports: `docs/evidence/2026-10-30/route_continuation_run{A,B}_run_report.json`.

## What was built
- `sitl/route.py`: a **route** = North/East waypoints + start time + speed on the shared operator clock, with an optional repeating loop. The goal is a continuous pure function of the clock, `position(t)`; every drone evaluates it itself, so the swarm stays in step without any message.
- The route rides on the mission intent (validated; old intents still parse). Agents use the route position as the goal, with a formation transition taking priority. Decentralized and hybrid agents keep flying it after the coordinator is lost (autonomy window 120 s with at least one live peer); centralized still holds and lands after 20 s.
- Console: `patrol square <side_m> [speed]` (approach from the current goal, then loop the square), `route stop` (holds where the route is, no jump), `route status`. The route is checked against the geofence, blocks moves and formation changes while active, and is drawn on the map.

## Live scenario (both runs)
launch → patrol square 12 m → coordinator stream off 30 s (hybrid) → on → switch to decentralized → off 30 s → on → switch to centralized → off 14 s → on → route stop → land.

| Run | Route speed | Hybrid, coordinator off 30 s | Decentralized, coordinator off 30 s | Centralized, coordinator off 14 s | Run report |
|---|---|---|---|---|---|
| A | 0.6 m/s | kept patrolling (10.9 m net) | kept patrolling (10.9 m net) | held (0.3 m net; 10 "holding position" logs) | **FAIL**: min gap 1.7 m, 2 violation episodes, max tracking error 3.07 m |
| B | 0.3 m/s | kept patrolling (9.4 m net) | kept patrolling (7.6 m net) | held (0.5 m net; 10 "holding position" logs) | **PASS**: min gap 3.80 m, 0 violations, max tracking error 0.73 m (p95 0.54 m) |

## Finding: run A exposed a flaw in the feature (fixed in `ad79cd5`)
The agents slew their position setpoint at `LocalDroneAgent.POSITION_TARGET_SPEED_M_S` = 0.45 m/s. A route faster than that leaves the setpoint behind the goal by (route speed − 0.45) m for every second flown, without bound. At 0.6 m/s the lag reached about 15 m after two minutes; the swarm then showed 2–3 m tracking errors in five drones around a patrol corner (t+155…165 s) and sampled separations down to 1.5 m. The setpoints themselves stayed smooth (0.46 m/s steps, no jumps), which is how the cause was isolated. Routes and `patrol` now refuse speeds above 0.4 m/s (default 0.3). Run B at 0.3 m/s, otherwise identical, passed.

## Limits
- One run per configuration; separation sampled at ~1 Hz.
- The coordinator was simulated lost by pausing its intent stream (`coord off`), not by killing its process. The 120 s autonomy cap and the lease renewal were exercised only within the 30 s windows.
- The centralized 20 s fail-stop **landing** was not exercised (the stream returned after 14 s).
- Routes are only squares so far; the planner does not route around the geofence or obstacles beyond the pre-check.
- Whether the hybrid/decentralized formation keeps its shape during a long autonomous route with real packet loss was not tested.
