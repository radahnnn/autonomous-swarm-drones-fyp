# Geofence, altitude layers, run metrics and replay — live 10-drone SITL (2026-10-31)

**Evidence level: SITL** — 10 ArduPilot SITL instances, standalone 2D, one host, loopback UDP; `--control-mode hybrid --agent-runtime independent --adaptive-swarm --transition-protocol --geofence-radius 24 --altitude-layers 3 --layer-spacing 1.5`. Commit under test: `efc9e15` (run 1) and `031ea48` (run 2). Raw logs (not in Git, checksummed): `runs/independent/geofence-live-20261031/`. Report of run 2: `docs/evidence/2026-10-30/geofence_layers_live_run_report.json`.

## What was added
| Feature | What it does |
|---|---|
| **Run metrics** (`sitl/run_metrics.py`) | Live minimum gap against the 2.5 m gate (sampled from ~1 Hz status), margin, low-margin and violation episodes, per-drone mode / link age / tracking error, event counters, 3-D gap; `run_report.json` + `RUN_REPORT.md` written at shutdown. Dashboard panel with margin bar and sparkline. |
| **Replay** (`python3 -m sitl.replay`) | Replays a recorded `events.jsonl` in the same dashboard (read-only): play/pause, speed, scrub slider, previous/next event (mode switches, formation transitions, coordinator stream, launch/land); metrics recomputed up to the cursor. Verified on the recording of the earlier 10-drone run. |
| **Geofence** (`sitl/geofence.py`) | Circle or box (+ ceiling). Prevention: every setpoint is clamped inside the margin-shrunk fence; the console refuses moves, altitude changes and formations that would not fit (message names the drone and position). Detection: inside / warn / breach; a breach lasting 8 s lands the drone. The fence is drawn on the map. |
| **Altitude layers** | Drone ID mod K layers, spacing configurable (default 1.5 m); constant per drone so nothing changes altitude mid-morph. Planner and guard stay horizontal (conservative); layers add vertical separation on top. |

## Live results
- Moves refused as designed: `w 40` ("D1 would be at N 40.0 E 0.0, outside the geofence") and `d 20` ("D3 would be at N 3.0 E 23.0 ..."); `w 6` accepted.
- Altitude layers observed: 0.5 / 2.0 / 3.5 m (layers 0 / 1.5 / 3.0 m on top of the commanded altitude); status carries `altitude_layer_m`.
- **Run 1 found a real gap (script error exposed it):** `u 40` then `j 40` took the whole swarm to the 0.5 m floor; the formation protocol then correctly aborted (`not_airborne:0`) and held. Fix (`031ea48`): operator altitude commands are limited to 2 m or higher while flying.
- **Run 2 (with the floor):** `j 40` limited ("Altitude command limited"); ring change completed inside the fence; in-flight switch hybrid → decentralized completed; landing clean. Run report **PASS**: min horizontal gap 3.108 m (at t = 102 s, during the ring morph), min 3-D gap 3.452 m, 5th percentile 3.376 m, 0 violation and 0 low-margin episodes, tracking error mean 0.166 m / p95 0.491 m, fence state `inside` throughout, counters `transition_complete 1`, `mode_switch_complete 1`.
- Run 1's report also recorded the aborted transition (`transition_aborted 1`, `needs_operator 1`), which shows the counters work on failures.

## Limits
- One run per configuration; separation sampled at ~1 Hz.
- A real **breach was not exercised live** (the clamp prevents it by construction); breach detection, the counter and the 8 s landing are covered by unit tests only.
- The altitude layers were **not** shown to improve the horizontal minimum gap (3.11 m here vs 2.65 m in an earlier run without layers — different runs, no controlled comparison). They add vertical separation only; adjacent formation slots are not guaranteed to be on different layers (layer = ID mod K).
- The fence is software only: it does not replace the flight controller's fence or RC override on real hardware.
- The ArduPilot altitude reference is the SITL relative altitude; no wind or GPS error here.
