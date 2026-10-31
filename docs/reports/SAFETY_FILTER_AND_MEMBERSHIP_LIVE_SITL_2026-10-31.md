# Control-barrier filter and minimum-motion slot reassignment — live 10-drone SITL (2026-10-31)

**Evidence level: SITL** — 10 ArduPilot SITL instances, standalone 2D, one host. Same scenario as the election run
(`docs/reports/COORDINATOR_ELECTION_LIVE_SITL_2026-10-31.md`): hybrid start, patrol square 12 m at 0.3 m/s, switch to
centralized, operator stream off (D1 acting coordinator), **D1's agent killed** (SIGTERM, by PID), operator returns.
Commits under test: `fa8c2d0` (barrier filter), `6a94db1` (slot reassignment). Raw run (not in Git, checksummed):
`runs/independent/membership-live-20261031/`; run report: `docs/evidence/2026-10-30/membership_reassign_cbf_run_report.json`.
The run was driven by the new scenario runner (`python3 -m sitl.fleet_ctl scenarios/election_10.txt`).

## Problem (defect 9)
When a drone's agent dies the survivors are redistributed over a smaller formation template. The old rule gave the
survivors the template slots **by rank**, which shifts every one of them one slot along; in a V or diamond this
sends many across the centre line at once. Live result before any fix: survivors' minimum gap 3.90 m -> **0.10 m**.

## Changes
1. **Barrier filter** (`sitl/cbf_filter.py`, `sitl/decentralized_guidance.py`): per-neighbour half-plane constraint on
   velocity (`h = |p|^2 - D^2`, D = 2.8 m, alpha = 3), solved exactly in 2-D, applied to the commanded setpoint.
   Live result with the filter alone: survivors' minimum gap 0.10 m -> **1.92 m** (still FAIL, 6 violation episodes,
   infeasible bursts when many drones converged on one point).
2. **Minimum-motion slot reassignment** (`sitl/adaptive_membership.py`): survivors keep, as far as possible, their
   original slots; the smaller template is matched by Hungarian assignment from their *original slot positions*.
   Pure function of (active set, formation, count): every agent derives the same map without exchanging positions.
   Planned straight-line analysis (UNIT): V worst-case clearance 1.18 m -> 3.00 m, diamond 0.00 m -> 2.83 m, mean move
   9.6 m -> 2.0 m for a V of 10.

## Live result (both changes, same scenario)
| Quantity | No filter, rank slots | Filter, rank slots | **Filter + minimum-motion slots** |
|---|---|---|---|
| Survivors' minimum gap | 0.10 m (1 episode) | 1.92 m (6 episodes) | **3.62 m (0 episodes)** |
| Run report | FAIL | FAIL | **PASS** (5th-percentile gap 3.995 m, 253 pair-samples) |
| Tracking error mean / p95 | - | - | 0.231 m / 0.442 m |

The 3.62 m minimum occurred at t = 94 s, after the kill (t ~ 84 s), i.e. across the membership change. The drone that was
killed is excluded from the gap automatically (its status goes stale).

## Limits (read before quoting)
- **One run per configuration, not a paired or repeated comparison**; the three columns are three separate live runs of
  the same script on different commits. The improvement is large, but variance between live runs has not been measured.
- The membership change tested is one dead agent (D1, the apex of the V); other members, two losses, and rejoin are
  covered by unit tests only.
- Separation is sampled at ~1 Hz and horizontal only; it is not a continuous-time guarantee.
- The barrier filter assumes single-integrator motion and constant neighbour velocity (see `sitl/cbf_filter.py`); the
  real vehicle lags the setpoint, which is why the filter alone was not enough.
- After `land` the operator stops its mission stream, and in this run D2 started a new acting-coordinator term while the
  swarm was descending (cosmetic, but unexplained beyond that); landing completion was **not** verified by this run
  (the scenario's landed-expectation did not match the console output and the console exited).
- Tooling findings: a relative `--evidence-dir` broke SITL/agent start (agents saw no heartbeat; fixed by resolving
  the path); an earlier start failure right after another run was not diagnosed.
