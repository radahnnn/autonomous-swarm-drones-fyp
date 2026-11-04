# Overnight work 2026-11-03/04: Codex fixes verified, root cause of the realistic near-misses, repeat campaigns

**Evidence level: SITL (2D, 10 drones) and GAZEBO (6 drones)**, with true (simulator) positions logged via SIMSTATE.
Per-campaign summaries: `docs/evidence/2026-11-04/*-SUMMARY.md`; raw runs (not in Git): `runs/independent/campaign-20261104*`,
`runs/independent/gazebo-recheck-20261104`.

## 1. Root cause of the recurring near-miss under `--realistic` (found in ArduPilot's own dataflash logs)
- The near-miss was **real** (true positions confirmed it) and **not** caused by swarm commands: the drones' targets stayed
  on their slots while individual autopilots flew 7-18 m away from them (flyaways).
- The logs show **EKF3 emergency yaw resets / IMU-lane switches** in exactly that window. Count of these messages per 10-drone
  run: calm 0, wind only 0, GPS only 3, GPS + wind (+ radio) 17-24.
- Cause: SITL's GPS noise model (`SIM_GPS1_HNSE`) wanders the GPS *position* while the GPS *velocity* stays exact; together
  with wind accelerations this drives the estimator's yaw into resets. Real receivers' errors are consistent, so this is
  partly a simulator-model artefact - but EKF yaw resets and flyaways also occur on real drones.
- Removing vibration/sensor noise did not remove it (true min 0.20 m without vibration), which ruled vibration out.

## 2. Decisions made overnight (user asked for reasonable decisions, recorded with reasons)
| Decision | Why |
|---|---|
| Realistic GPS wander 1.0 -> 0.5 m | at 1.0 m the EKF reset storm dominated every run; at 0.5 m resets fell to 0-9 per run. Not reduced further, to avoid tuning the test until it passes. Real GPS error must be measured. |
| Onboard flyaway detection: far (> 3 m) from a **steady** target for 1 s -> degraded (peers keep +2 m); > 10 m -> land | the swarm cannot prevent an autopilot flyaway, but it can isolate it. 4 m / 2 s reacted too late; "steady target" added after it false-flagged drones lagging a morph and froze them mid-transition. |
| Gap manoeuvre re-plans an aborted (HOLD) formation up to twice, then stops | it used to wait forever after a mid-move abort. |
| Evidence coverage fix, landing drones not "missing" | report bugs found in the live check. |
| Not done (needs the user): formation changes when some drones have landed | today every formation change requires all drones airborne; after the flyaway guard lands a drone the other 8-9 cannot reshape. Needs partial-fleet planning (design decision). |

## 3. Results with the final code (campaign d) and earlier campaigns
| Scenario (10 drones, 2D) | Campaign a (before flyaway tuning) | b (3 m/1 s) | c (same code as b) | **d (final)** |
|---|---|---|---|---|
| Calm mission | 3/3 PASS, true min 5.16 m | - | - | - |
| Realistic mission | 2/3, true worst 0.37 m | 2/3, 1.51 m | 1/3, 2.10 m | **2/3, true worst 1.71 m, all steps completed 3/3** |
| Tower gap, realistic | 1/3, 1.24 m | 3/3, 2.70 m | 1/3, 1.27 m | **2/3, true worst 2.04 m, all steps completed 3/3** |

Gazebo (6 drones, realistic, final code except the steady-target refinement): survey patrol **PASS** (true min 4.97 m) and
tower gap **PASS** (true min 4.25 m), all steps completed; one run each.

Honest reading: with no disturbances the system is consistently safe (3/3, ~5.2 m). Under the full assumed disturbance set,
roughly one run in three still has a short dip below the 2.5 m gate (true worst 1.7-2.0 m in the final campaign), driven by
autopilot estimation events. Variance between campaigns of the same code (b vs c) is large, so 3 runs per cell are not enough
to rank small code changes; the pass rates above are indicative, not precise. Codex's 16 review fixes all behaved correctly in
flight (mode switches, transitions, retries, LAND, true-position reporting).
