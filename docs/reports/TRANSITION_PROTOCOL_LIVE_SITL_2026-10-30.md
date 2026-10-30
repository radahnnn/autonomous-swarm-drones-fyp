# Synchronized formation transitions — first live SITL run (2026-10-30)

**Evidence level: SITL** — 5 real ArduPilot SITL instances, standalone (2D), one host, loopback UDP, `--control-mode decentralized --agent-runtime independent --adaptive-swarm --transition-protocol`. No network impairment, no VM split, no Gazebo, no hardware. Commit under test: `0c02b7c` (earlier runs: `8e6e4a6` and before). Compact summary: `docs/evidence/2026-10-30/transition_protocol_live_summary.json`. Raw event logs and console output: `runs/independent/transition-protocol-live-20261030/` (local, not in Git; `SHA256SUMS.txt` alongside).

Scenario driven through the console: launch (5 drones to 5.0 m) → `formation line` → `formation column` with agent 3's process killed (SIGTERM) 4 s into the morph → `land` → quit.

## What happened, in order (all three runs kept)

| Run | Commit | Result | Root cause found |
|---|---|---|---|
| 1 | `8e6e4a6` is the fix for this; run was on `52added` | All 5 agents acked within 0.2 s and the coordinator committed, then an agent aborted 0.2 s later with `peer_stale:3`; the next plan failed `position_mismatch` | (a) supervision used only the packets received in the current 0.1 s tick, so most peers looked missing; (b) an abort before the common start froze agents in HOLD with shifted slots instead of discarding the plan |
| 2 | `8e6e4a6` | Aborted 2 s after prepare with `peer_off_plan:3`; the whole swarm had drifted ~3 m west before any morph started | The coordinator moved the mission goal to the swarm centroid at *prepare* time while agents still used the old slot template, translating every target |
| 3 | `0c02b7c` | **LINE completed; COLUMN with a killed agent ended in a clean HOLD** (details below) | — |

Neither of the two bugs could be found by the in-memory tests (they fed complete peer data and ideal tracking); both were reproduced as regression tests and mutation-checked.

## Run 3 results

**LINE transition (all five healthy):**
- prepare → all acked/committed in 0.21 s; common start 4.02 s after prepare; complete 19.05 s after prepare (12 s morph + arrival dwell).
- Minimum pair separation 3.62 m (gate 2.5 m) over 91 status samples — **status is published at 1 Hz, so this is a sampled value, not continuous**.
- Final positions: line along East at exactly 4.0 m spacing (−8, −4, 0, 4, 8), all five agents reported `formation = line`.

**COLUMN transition with agent 3 killed 4 s into the morph:**
- Surviving agents detected the dead peer in about 1 s (`peer_stale:2`, the 1.0 s threshold) and the coordinator moved to HOLDING 8.9 s after prepare (≈ 4.8 s after the common start).
- All four survivors entered HOLD at the same progress (0.406–0.408), stayed airborne at 5.0 m, and **no LAND was issued by the protocol**. Minimum separation over the transition and hold: 3.85 m (sampled, 1 Hz; includes the killed drone's last position).
- After 20 s of HOLD the console told the operator to re-plan or LAND (`needs_operator`). `land` was acknowledged by the four survivors, who landed and disarmed; the killed agent's vehicle was rescue-landed by the console at shutdown. No leftover processes or ports.

## Limits (read before quoting any of this)

- One successful run of each scenario; no repetition, no confidence intervals, no seeds.
- 1 Hz status sampling for separation; the agents' own 10 Hz data was not logged for this analysis.
- Same host, loopback UDP, no loss/latency/partition injected; coordinator never failed; only one failure type (agent process kill) was tried.
- Standalone 2D SITL: vehicles are ArduPilot SITL, not Gazebo or hardware.
- Opt-in flag only; the old intent-driven morph is still the default and was not re-tested here.
- Thresholds (`lead_s`, `peer_stale_s`, `local_hold_separation_m`, …) are the spec's initial values, not calibrated.
- The agent's abort reaches peers through the coordinator relay (no direct peer broadcast yet); here the peers also detected the dead agent independently.

## Next
Repeat with 10 drones, repeat the kill scenario several times, add loss/latency (`--peer-loss`, `--peer-latency-ms`), a dropped `go`, a coordinator kill, and the host/VM split; then decide whether to make the protocol the default.
