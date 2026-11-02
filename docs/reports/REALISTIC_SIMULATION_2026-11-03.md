# Realistic simulation: disturbances, attribution matrix and fixes (2026-11-02/03)

**Evidence level: SITL** (ArduPilot standalone 2D, 10 drones, one host) unless stated. All disturbance values are
**assumptions** (the team has not yet measured GPS error, battery, vibration or link on its quads). Raw runs (not in Git):
`runs/independent/realistic-2d10-*`, `runs/independent/attribution-20261103/`.

## What `--realistic` adds
`python3 -m sitl.swarm_console ... --realistic` (independent runtime): SITL profile `realistic` (GPS horizontal wander
1.0 m, vertical 0.5 m, lag 120 ms; wind 3 m/s from the west with 1 m/s gusts; accelerometer/gyro noise and motor
vibration; battery sag + land-on-low failsafe at constant pack voltage), a lossy, delayed, bursty peer link (5 % loss,
40 +/- 15 ms, Gilbert-Elliott bursts), `--position-sigma 1.0` and `--formation-scale 1.4`. Every SITL value was read
back from inside ArduPilot. Split profiles `realistic-gps / -wind / -vibration / -battery` exist for attribution.

## Attribution matrix (10 drones, same ~12 min mission: hover, decentralized patrol, centralized, LINE and back)
| Disturbance | Run report | Min gap | Mean tracking |
|---|---|---|---|
| none (battery model only) | PASS | 5.18 m | 0.16 m |
| GPS error | PASS | 3.70 m | 0.31 m |
| wind + gusts | PASS | 5.19 m | 0.16 m |
| vibration + sensor noise | PASS | 5.18 m | 0.17 m |
| lossy radio | PASS | 5.19 m | 0.17 m |
| all together | FAIL (ground contacts after ~9 min; see 3) | 2.52 m | 0.30 m |

**GPS error is the disturbance that matters** (min gap -1.5 m, tracking error x2); wind, vibration and the radio alone
changed almost nothing in this model. One run per cell.

## Problems found and fixed (each with tests)
1. Gazebo would not arm at a 2.5 ms physics step (400 Hz gyro < ArduCopter's 720 Hz requirement) -> step capped at
   1.38 ms, default 1.25 ms (6 drones armed, RTF 0.19).
2. Formation changes aborted under GPS noise (protocol thresholds tuned on clean positions) -> `--position-sigma`:
   wider position tolerances, planned clearance one sigma above the 2.8 m abort line (never worse than the current
   closest pair), clearance-maximising assignment for world-anchored layouts; mode switch allowed in post-abort HOLD.
3. Neighbours 4.24 m apart tripped safety checks on noise alone -> global formation scale (`--formation-scale`, 1.4
   under `--realistic`), applied at `slots_for` so console, agents, start layout and Gazebo spawn agree.
4. Battery model: with 0.03 ohm sag the simulated motors saturated and every drone sank after ~35 s; later, as the
   simulated 3S pack drained below ~11.2 V (~9 min) SITL's generic quad could not hover even with MOT_BAT_VOLT
   compensation. The team's 5" 6S quads hover at far lower throttle, so drain is now off in simulation (constant pack
   voltage, sag + failsafe kept). **Endurance must be measured on the hardware; plan real missions under ~5 min.**
   Four earlier "PASS" realistic runs were measured on drones bouncing off the ground and are withdrawn; the run
   report now FAILs on any ground contact while armed outside LAND.
5. LINE -> V aborted in every matrix run (even calm): the plan was built while the 50 m line's tail drone was still
   drifting -> the console now waits (<= 30 s) for the swarm to settle before planning an airborne formation change.

## Final realistic run (all fixes, all disturbances, 10 drones)
Take-off of all 10, decentralized patrol, LINE, COLUMN and V transitions **completed** (3 of 4; one V attempt refused
before start), no ground contact, all agents alive, landing. Run report **FAIL**: one near-miss episode cluster, min
gap 1.01 m at t ~ 183 s (5th percentile 4.44 m).

## Open problem (top of the plan): position-estimate glitches
In that near-miss, D2's *reported* position jumped ~10 m per second (e.g. N -9.2 E -2.5 -> N -6.7 E -16.1 within
4 s) while its target stayed put. That is not physical motion: D2's position estimate glitched under GPS error plus
vibration (the drone may also have drifted while chasing the bad estimate; the logs cannot tell). Needed: an
**estimation-health guard** - an agent that sees implausible jumps in its own estimate marks itself degraded and
holds/lands, peers widen the safety distance to it, and the run report separates estimate glitches from true gaps.
Not yet run with all fixes: the 6-drone Gazebo survey patrol under `--realistic`; the tower-gap manoeuvre (not built).
