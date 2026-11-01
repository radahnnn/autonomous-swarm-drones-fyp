# Independent runtime in Gazebo, 6 drones — first runs (2026-11-01)

**Evidence level: GAZEBO** (Gazebo Sim headless server, 6 ArduPilot SITL instances in JSON mode, ArduPilot plugin,
one host, `--control-mode hybrid --agent-runtime independent --adaptive-swarm --transition-protocol --time-scale 0.16`).
Scenario: `scenarios/gazebo_6.txt`, driven by `python3 -m sitl.fleet_ctl ... --gazebo`; world launcher
`sitl/launch_gazebo_world.sh 6`. Raw runs (not in Git, checksummed): `runs/independent/gazebo-6drone-20261101*`;
run reports: `docs/evidence/2026-10-30/gazebo_6drone_run{1,2}_run_report.json`.
**Status: partial. This is a first bring-up, not a passing demo.** The Gazebo GUI was not used (server only).

## Measurements and what worked
- Real-time factor with 6 SITL attached: **about 0.16** (0.084 for the server alone before SITL attached). A 25-minute
  wall-clock run is therefore about 4 minutes of simulated flight; EKF convergence alone takes roughly 5 minutes.
- The independent runtime (agents, peer links, console, protocols) ran in Gazebo for the first time: all 6 reached
  EKF-ready, armed, took off, stayed alive for ~25 min (`health`: no agent died), telemetry age about 0.2 s.
- **In-flight mode switching worked in Gazebo in both runs**: hybrid -> decentralized -> centralized, each acknowledged
  and completed (`mode_switch_complete` x2).
- Slow-simulator support added (`--time-scale`): setpoint slew speed, transition duration and arrival timeouts scale
  with the real-time factor (UNIT-tested; plan duration bound raised from 60 s to 600 s after run 1).

## What did not work
| Run | Run report | Formation change |
|---|---|---|
| 1 | PASS, min gap 2.562 m, 0 violations (barrier filter activated 297 times) | **rejected before starting**: scaled duration (about 75 s) exceeded the protocol's 60 s plan bound. Fixed in `c04cfe9`. |
| 2 | **FAIL**, min gap 1.598 m, 3 violation episodes (1.5 s total) | **cancelled before start** by an agent's separation check (`agent_abort:0:separation:2`); no drone moved. The second request (V -> V) was a no-op. |

So **no formation transition completed in Gazebo**. In run 2 the pairs D5-D3 (1.60 m) and D1-D3 (2.21 m) dipped about
50 s after the first airborne report, soon after the cancelled transition; the cause was not isolated (candidates:
Gazebo/ArduPilot position settling after takeoff at 0.16 real-time, the V-formation spacing being only just above the
2.8 m pre-start check, or behaviour after the abort). Runs 1 and 2 differ only by the code fix and chance, so the
spread (2.56 m vs 1.60 m) also shows the run-to-run variance is large.
- `land`: the console exited about 4 minutes after the command without printing landing confirmation; landing
  completion was not verified.
- The scenario's "transition complete" expectations timed out in both runs, as above.

## Not done
Route/patrol in Gazebo (route speeds are not time-scaled), coordinator-loss and agent-kill tests, GUI/recording,
10 drones. Next: isolate the run-2 dip (log positions around the abort; try a wider V for Gazebo), repeat the transition,
verify landing.
