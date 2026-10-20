# Hybrid hardening and comparison — 20 October 2026

## Scope and checkpoints

Experimental branch: `work/decentralized-sitl-20261020`. The stable demo branch is unchanged.
Checkpoint `c9262e9` preserves the preceding implementation and three original standalone runs,
including the failed rejoin run. Copies and SHA-256 hashes are in `docs/evidence/2026-10-20/`.
Original run directories are retained. A run recorded against a dirty checkout does not mean its
code exactly equals the reported HEAD; the initial three runs preceded the checkpoint.

## Implemented changes

- Mission movement and formation changes stop while a drone is faulted or recovering; a failed
  slot cannot be translated or reassigned by those commands. Patrol stops on a rejected movement.
- Hybrid coordinator loss freezes the entire mission, including altitude and relative slots.
  Local/hybrid in-flight formation morphs remain disabled pending safe path planning.
- Rejoin screens the straight horizontal path to the reserved slot against fresh fleet positions,
  using 2.5 m clearance. Only one recovery can start at a time. This screen assumes other drones
  hold their positions; it is not a guarantee against tracking error or unknown movement.
- Recovery arming has cancellation checks and no forced-arm fallback. LAND prevents late rejoin
  and waits for the recovery worker to release its connection before issuing landing commands.
- Stale position data beyond 2 s requests LAND even if heartbeats continue. Stale/nonfinite peer
  source data is rejected rather than refreshed by a relay. Telemetry faults do not auto-relaunch.
- Optional heartbeat recovery retains the requirement for three seconds of fresh, disarmed
  heartbeats. An armed report resets that timer. Operator faults require manual rejoin.
- `status` shows agent mode/blend, fresh peers, telemetry age, recovery state, and minimum
  horizontal separation across all vehicles with fresh positions, including inactive vehicles.
- `--agent-runtime process` runs one controller per OS process. Private versioned JSON messages
  carry only local inputs. Wrong IDs, replay, stale/nonfinite data and invalid frames are rejected.
  A dead/unresponsive controller requests LAND for its simulated vehicle. Restart is required.

The console still supplies topology/mission, emulates peer transport and owns all MAVLink links.
The peer protocol is documented in [AGENT_PROTOCOL.md](../AGENT_PROTOCOL.md). This is a useful
process boundary toward distributed deployment, not a completed onboard architecture.

## Paired comparison: 90 pure-software trials

Three modes × three conditions × ten shared seeds (20261020–20261029). Each trial has five
vehicles, a 3 m V spacing, the same initial perturbation, 10% independent packet loss, 50 ms mean
latency, 0.08 m measurement noise and a 30 s mission: hold 2 s, move north at 0.4 m/s until 20 s,
then hold. Outage conditions cut coordinator transmission at 5–8 s and 15–18 s. Burst cases add
Gilbert–Elliott loss. Each mode uses the same configuration; distance-dependent loss can still
produce different delivered packets. All modes retain centrally provided mission goals.

Dynamics are the explicitly **assumed baseline**, not the old timing-artifact fitted profile.
These trials exercise the Python simulation engine, not the console adapter, Gazebo or hardware.
Do not combine them with the standalone SITL evidence as one reliability estimate.

Acceptance: minimum separation ≥2.5 m throughout, and final steady-window p95 formation error
≤1 m. Each cell below has ten trials. Confidence intervals are Student-t 95% intervals for the
mean across seeds; raw trials and paired differences are in
[three-mode-comparison.json](../evidence/2026-10-20/three-mode-comparison.json).

| Condition | Centralized passes | Decentralized passes | Hybrid passes |
|---|---:|---:|---:|
| Nominal impaired channel | 10/10 | 10/10 | 10/10 |
| Two coordinator outages | 10/10 | 10/10 | 10/10 |
| Burst loss + two outages | 8/10 | 10/10 | 9/10 |

For the burst/outage case:

| Mode | Mean steady error (95% CI), m | Mean trial minimum spacing (95% CI), m |
|---|---|---|
| Centralized | 0.120 (0.090–0.149) | 2.634 (2.532–2.737) |
| Decentralized | 0.175 (0.154–0.195) | 2.759 (2.695–2.822) |
| Hybrid | 0.120 (0.091–0.148) | 2.662 (2.590–2.735) |

The failed centralized seeds 20261022/20261023 reached 2.429/2.432 m. The failed hybrid seed
20261023 reached 2.454 m. These are violations of the project spacing criterion, not necessarily
physical collisions. They are preserved. The final steady window alone does not describe all
outage tracking transients. Ten seeds do not prove high reliability; this comparison provides no
basis for claiming hybrid always wins. Next research priority is diagnosing transition spacing
in those failing seeds before tuning on a separate development set and testing held-out seeds.

## Verification and limits

Regression coverage includes source staleness, replay, finite data, worker/local output equivalence,
worker death, whole-mission outage hold, blocked movement during repeated faults, corridor rejection,
stale-position LAND, recovery timer reset and cancellation. Live evidence is listed below when
completed. Optional automatic heartbeat recovery is unit-tested, not yet demonstrated end-to-end
with a real telemetry network interruption. No new physical-flight or Gazebo claim is made.

Reproduce the software study from the repository root:

```bash
python3 -m experiments.compare_three_modes --trials 10 --output /tmp/three-mode-comparison.json
```

Start the experimental standalone five-drone console:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone --control-mode hybrid --agent-runtime process
```

After EKF readiness: `launch`, `status`, `coord off`, `status`, `coord on`. Wait for all agents to
recover, then try `w 2`, `f 3`, and later `r 3` after landing. `q` lands and closes the simulation.

## Five-process standalone live run

Evidence: [20261020T183103.738499Z](../evidence/2026-10-20/20261020T183103.738499Z/summary.json).
Five separate hybrid controllers, 10% peer loss, 50 ms mean latency, seed 42.
Scenario: takeoff, coordinator off/on twice, blocked movement during outage, north 2 m,
Drone 3 LAND, blocked movement during recovery, Drone 3 rejoin, final LAND/disarm for all five.
Recorded criteria pass: **True**. Minimum active-pair separation 3.075 m;
p95 settled slot error 0.668 m; watchdog timeouts 0.
Offline scan across *all fresh vehicle positions*, including inactive/landing vehicles, gives
minimum horizontal separation 3.075 m. This still excludes intervals without fresh positions.

Source provenance: this run began after process isolation, mission locks, and corridor checks were
added on top of c9262e9. Recovery cancellation/no-force-arm, stale-position watchdog, extra protocol
validation, and the center/launch lock fixes were added while it was running and were **not loaded
by that process**. Those later changes have regression coverage; a final-source run is recorded separately.
The full suite passed 184 tests before the additional center/launch regression; focused tests cover that
last fix. The stored pytest output includes environment-only Matplotlib warnings.
