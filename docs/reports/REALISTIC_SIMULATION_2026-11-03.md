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

## Update (2026-11-03, day): estimate guard, retries, Gazebo patrol, tower gap

- **Estimation-health guard** (`sitl/estimate_health.py`): a drone whose own position estimate implies > 4 m/s
  (judged over >= 0.5 s) freezes its target, flags itself in peer messages (peers keep 2 m extra in the barrier filter)
  and lands if glitches persist 15 s. First version judged over 0.05 s and raised 20 false alarms per run (jitter);
  real jumps in the estimate were 0-2 per run (counted in the attribution runs).
- **Retry of pre-start cancellations** for transient reasons (radio burst, drone still settling), max 2; safety
  reasons never retried.
- Realistic 10-drone 2D run with both: min gap **2.48 m** (was 0.92 m), both mode switches completed, 2 of 4
  transitions completed (the other two were pre-start cancellations, before the retry existed). Still FAIL by one
  0.5 s sample 2 cm under the gate.
- **Gazebo, 6 drones, full survey patrol under `--realistic`: PASS** (min gap 4.81 m, 0 violations): centralized V
  patrol with pole detection (D1 sensed, 5 learned), decentralized pairs A and B, hybrid with coordinator cut and
  takeover, regroup to V, landing; 9 capture rounds x 6/6 real camera images
  (`docs/evidence/2026-11-01/gazebo_patrol_realistic_pairs_b_cameras.png`); 2 transient cancellations rescued by the
  retry. One run. Report json: `docs/evidence/2026-10-30/gazebo_patrol_realistic_20261103_run_report.json`.
- **Tower gap (user's idea), 10 drones 2D under `--realistic`, towers 11.2 m apart across the path:** the swarm sensed
  the towers, judged the V (needs > 10 m free) would not fit, stopped short, formed a heading-aligned column, flew
  through, confirmed every drone past - all live. Re-forming the V failed (radio-burst cancellation, then a safety
  refusal), and the run FAILs: consecutive column members came within 2.36 m (in the gap) and 0.85 m (12 m past it,
  outside the towers' push zone). Diagnosis: in a moving single file under ~1 m GPS error the drones' along-track
  lags differ and the ~5.5 m spacing breathes below the gate. **Next:** wider column spacing for the gap (~8 m) and a
  slower pass, then rerun; Gazebo version with two real towers after that.
