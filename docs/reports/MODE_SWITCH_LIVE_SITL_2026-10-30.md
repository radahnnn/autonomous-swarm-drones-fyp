# In-flight control-mode switching — first live 10-drone SITL run (2026-10-30)

**Evidence level: SITL** — 10 real ArduPilot SITL instances, standalone (2D), one host, loopback UDP, started with `--control-mode hybrid --agent-runtime independent --adaptive-swarm --transition-protocol --dashboard-port 8765`. No Gazebo, no VM split, no hardware, no network impairment. Commit under test: `eb38cf0`. Compact summary: `docs/evidence/2026-10-30/mode_switch_live_summary.json`; raw logs (not in Git, checksummed): `runs/independent/mode-switch-live-20261030/`. The browser dashboard was watched live during the run.

## What was built
- `sitl/mode_switch.py`: pure state machines (`AgentModeSwitch`, `CoordinatorModeSwitch`). Prepare → every drone acknowledges → common start time → smooth (cubic) blend of the outgoing and incoming controllers' commands over a 2 s ramp → verify phase in which any drone can still be rolled back → `done`. Aborts and `done` are retransmitted until every agent confirms (a lossy-channel simulation found a split-mode hole before this was added: a lost abort let some agents self-commit; the coordinator's verify deadline is now shorter than the agents' self-commit timeout).
- Agents compute all three modes (`LocalDroneAgent`): decentralized (consensus + flocking), hybrid (centralized tracking with automatic fallback), centralized (hybrid that never degrades).
- **Fault policy per mode** (this is what visibly differs between the modes, because the SITL position target is anchored to the assigned slot in all of them):
  | Coordinator silent | Centralized | Decentralized | Hybrid |
  |---|---|---|---|
  | Behaviour | hold position; land after 20 s (fail-stop) | keep flying the last mission on peers (up to 120 s) | fall back to local control, recover when it returns (up to 120 s) |
- Console `mode <centralized|decentralized|hybrid>` and dashboard buttons (enabled in flight). Formation changes and mode switches exclude each other.

## Live scenario and results (10 drones)
Driven by script: launch → `mode decentralized` → `formation ring` (protocol) → `mode centralized` → coordinator stream off 12 s → on → `mode hybrid` → off 29 s → on → `mode decentralized` → off 29 s → on → `mode hybrid` → land.

| Item | Result |
|---|---|
| In-flight switches | 5 of 5 completed; commit 0.10–0.17 s after the command, common start at +3.0 s, complete **7.3–8.2 s** after the command; 0 aborts, 0 rollbacks |
| Mode change on all drones | 50 of 50 `control mode a -> b` lines (5 switches × 10 drones) |
| Min pair separation in the 14 s windows around the switches | 3.54–4.22 m (1 Hz samples; gate 2.5 m) |
| Ring formation change (10 drones) | prepare → complete **34.2 s** (28 s morph); planned path clearance 3.12 m; **sampled minimum separation 2.65 m — the tightest moment of the whole flight (0.15 m above the gate)** |
| Min separation over the whole flight | 2.65 m (1 Hz samples) |
| Coordinator off, centralized (12 s) | all 10 drones logged "coordinator lost; holding position"; none landed; normal guidance resumed when the stream returned |
| Coordinator off, hybrid (29 s) and decentralized (29 s) | all 10 stayed airborne on the last mission (`mission` sequence frozen, no landing) |
| Land command | 10 of 10 landed and disarmed |
| Tracebacks / errors in console log | 0 |
| Resources (10 SITL + 10 agents + console, 61 samples) | mean 94 % of one core, peak 591 % of 600 % (takeoff), peak RSS 1.03 GB, load1 peak 2.4 |
| Teardown | no processes or ports left |

## Limits (read before quoting)
- One run; no repetition, no confidence intervals. 1 Hz status sampling for separation.
- Same host, loopback UDP, no loss/latency injected, coordinator never killed (only its intent stream paused), no agent killed during a switch, no rollback exercised live (rollback is covered by unit, lossy-channel and in-memory integration tests only).
- The centralized fail-stop **landing** (after 20 s) was not exercised live; only the hold phase (12 s) was.
- In SITL the three modes fly almost identically while everything is healthy (the target is anchored to the slot); the difference is in the fault policy above. Do not claim better accuracy for one mode from this run.
- Numbers describe SITL, not hardware.

## Next
Repeat with injected failures: kill an agent during a switch, drop a `go`, loss/latency, centralized > 20 s coordinator loss (landing), then the host/VM split; investigate the 0.5 m gap between planned and sampled clearance in the 10-drone ring morph (longer morph, larger clearance threshold, or a larger ring) before the demo.
