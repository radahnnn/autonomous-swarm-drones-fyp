# Project state — start here

**Last verified:** 2026-10-29, branch `wip/decentralized-sitl-handoff-20261029`, 318 unit/integration tests passing (`python3 -m pytest -q tests`).
**Rule for this file:** it is the single entry point. If it disagrees with an older document, this file wins; if it is wrong, fix it in the same commit that changes the facts.

Any assistant (Claude, Codex, or other) or teammate should read, in order:
1. this file
2. `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md` (live work log — append, never rewrite)
3. `docs/SOFTWARE_ACCEPTANCE_MATRIX.md` (what counts as done) and `docs/AGENT_PROTOCOL.md` (message formats)

## 0. If you are an AI agent (Codex, Claude, or other): do this first

1. Read this file fully, then the collaboration log's rules and its **latest entry** (the "next step" there is your starting task).
2. Verify before trusting: `git rev-parse --abbrev-ref HEAD && git rev-parse HEAD && git status --short`, `df -h /`, and `python3 -m pytest -q tests`. If branch, HEAD or status differ from section 7's "Last known state", **stop and report the difference** before editing.
3. Check no other assistant is currently writing to this checkout. Only one writer at a time.
4. Work from the plan in section 8, one focused commit per fix. Describe results with the evidence labels (UNIT … PHYSICAL); never present mocked tests as SITL evidence.
5. **Before you finish, update the docs** (see "How to keep this file current" below) and append a session entry to the collaboration log. Leave the tree clean where practical.
6. Reviewer role (Codex while Claude writes): read the diff and the latest log entry, run the tests, and add review findings to the log as a new entry; do not edit code unless the user hands you the writer role.

### How to keep this file current

Update this file **in the same commit** whenever a change alters any of: branch/HEAD or test count (section 7), a defect's status (section 5), an evidence level or objective status (section 4), file/folder locations (section 2), or the plan (section 8). Rules:
- Change facts, don't append commentary. Delete a defect from section 5 when fixed and note it in the changelog at the bottom with the commit hash.
- Keep "Last verified" at the top accurate (date, branch, test count). Only write a status you have actually checked; mark anything unverified as such.
- Raise an objective's status only with the evidence label that supports it, and cite the report or log path.
- Don't copy long content here; link to the report. Keep this file under ~250 lines.
- Session-by-session detail belongs in `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`, not here.

## 1. The project in one paragraph

Final Year Project BEE-60 (Dept. of Electrical Engineering, supervisor Dr. Abdul Ghafoor, deadline 31 Dec 2026): software for a swarm of multirotors flying line/V/ring/grid-style formations under **centralized**, **decentralized** and **hybrid** control, validated in a numerical simulator, ArduPilot SITL, Gazebo, and across a host/VM network boundary. Target scale 5–10 vehicles. Physical flight is a stretch goal and has not happened.

**Accurate wording for the current formation work:** distributed *execution* (each drone's agent flies its own assigned trajectory) with *centralized* transition planning. Do not claim decentralized formation negotiation yet.

## 2. Where things live

| What | Location | Rule |
|---|---|---|
| **Active development checkout (this repo)** | `/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/working-copies/decentralized-sitl-20261020`, branch `wip/decentralized-sitl-handoff-20261029` | Only place code is edited. One writer at a time. |
| Frozen demo baseline | `.../working-copies/demo-master` (`master`) | Read-only. Defense fallback. |
| Research lines (read-only archives) | `.../working-copies/hw-matched-identification`, `identification-phase2-20261020`, `experimental-hybrid-20261020` | Do not edit. Identification results are preliminary. |
| History shelf (checkpoints, backups, dependencies, references) | `/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/` (not a Git repo) | Archive only. Index: `START_HERE.md` there + `docs/history/ARCHIVE_INDEX.md` here. |
| Heavy evidence (ArduPilot `.BIN` logs, raw run output) | `/media/drone/SSDSAM/FYP_Archive/` (NTFS external SSD) | Checksummed; see archive index. |
| SITL VM (Ubuntu 24.04, QEMU/KVM) | `/media/drone/SSDSAM/drone-swarm-vm/` | Single copy; not yet backed up. |
| Matek H743 parameter exports and diagnostics | `hardware/` (repo; Oct 23 export and summary copied in 2026-10-29) | Do not change hardware files unless assigned. |
| GitHub | `https://github.com/radahnnn/autonomous-swarm-drones-fyp` | Do not push/merge/PR without the user's instruction. |

Git branches (remote): `master` (demo), `work/decentralized-sitl-20261020` (parent of this branch, `cb7fdcc`), `wip/decentralized-sitl-handoff-20261029` (this branch, WIP checkpoint `d1e7444`), `work/corrected-phase2-20261020`, `hw-matched-identification`, `archive/*`, `checkpoint/2d-working-savepoint`. This branch is 24 commits ahead of / 4 docs-only commits behind `origin/master`; README conflicts are expected on merge.

## 3. Code map (this checkout)

- `swarm_core/` — numerical foundation: config profiles, drone model, graph/Laplacian, network emulator, formations + Hungarian assignment, centralized/decentralized/hybrid controllers, safety.
- `simulator/` — fast numerical engine and plots.
- `sitl/` — ArduPilot integration. Key files: `swarm_console.py` (central pilot console), `independent_swarm_console.py` + `independent_agent_node.py` (one process per drone), `mission_intent.py` (coordinator→agent mission message), `peer_udp.py` (drone↔drone UDP), `decentralized_guidance.py`, `operator_dashboard.py` (browser UI), `flight_prep.py`, `fleet.py`, `formation_library.py`.
- `experiments/` — validation and sweep scripts (SITL, burst outage, VM split, operator loss, restart, partition).
- `tests/` — 318 tests. Mostly UNIT/INTEGRATION; they do not prove live SITL behaviour.
- `docs/reports/` — dated validation reports; `docs/evidence/` and `docs/reports/evidence/` — compact committed evidence. Raw runs are not committed.
- `runs/` — compact tracked evidence + ignored raw output. Five very large raw runs were moved to the SSD and are symlinked here.

Commands: see `docs/CONSOLE_GUIDE.md` (e.g. `python3 -m sitl.swarm_console --drones 5 --standalone --control-mode hybrid --agent-runtime process`). Setup: `pip install -e ".[dev]"` and `pip install -e ".[sitl]"` (pymavlink is a separate extra).

## 4. Progress against the proposal (evidence-labelled)

Labels: UNIT, INTEGRATION, SITL, QGC, GAZEBO, VM-NETWORK, PHYSICAL (see collaboration log). Approx. 70–75% of the software objectives are done; the rest is hard and mostly safety/protocol work.

| Objective | Status | Highest evidence | Gap |
|---|---|---|---|
| Mathematical model / simulator | Done | UNIT/INTEGRATION | Fitted SITL parameters are preliminary; old τ=0.992 s fit was a timestamp artifact; S4 cascade model not integrated into Phase 2. |
| Centralized control | Working | SITL, GAZEBO (5 drones, 20 Oct), QGC (15 Oct) | Repeated matched live runs. |
| Decentralized control | Working in SITL | SITL (same host), VM-NETWORK (5 and 10 agents, one clean mission each) | Repeated split-host fault runs; radio. |
| Hybrid switching | Working in SITL | SITL (same host) | Repeated live and separate-host evidence. Hybrid violated the 2.5 m guard in 1 of 10 burst-outage seeds. |
| Formations + reconfiguration | Partial | UNIT; SITL for earlier formations | V→line morph fails the 2.5 m gate (Phase 2). Transition protocol not built. |
| Scaling 5→10 | Partial | 30-seed point-mass pass (2 mm worst margin); one 10-agent VM mission | Thin margin; repeated live/fault runs. |
| Physical prototype | Bench-checked only | PHYSICAL (USB bench: FC, sensors, GPS fix; props off) | Never flown. ESP32/CAN link is a proposal; CAN enabled in params but no command path confirmed. |

## 5. Known defects and risks (current)

Tracked as the backlog in `docs/history/agent-handoffs/CLAUDE_ONE_WEEK_PROMPT_2026-10-29.md`. Confirmed on 2026-10-29 (the stale/out-of-order `MissionIntent` defect was fixed in the same session; see changelog):

1. **Runaway log on dropped SITL link** — run `external-sitl-20261022T193321` wrote ~54 M lines/agent (4.6 GB) of `EOF on TCP socket`; no backoff or log limit in the retry path.
2. **CI cannot install pymavlink** — `.github/workflows/ci.yml` installs `.[dev]` only, while tests import `sitl`. Python 3.10–3.12 matrix is unverified.
3. No prepare/ack/common-start/abort for synchronized transitions; supervision is coordinator-only (see spec §1); command gating during transitions is client-side only (`_operator_command` and dashboard endpoints are not gated server-side); link-loss/coordinator-loss/membership semantics undefined.
4. Dashboard: `session.count` vs `SwarmConsole.n` mismatch; commands lack structured results.
5. Mode supervisor has no child-readiness check or rollback; process cleanup is broad rather than ownership-based.
6. Several formation/slot-assignment implementations exist (`swarm_core/formations.py`, `sitl/formation_library.py`, others).
7. Repo hygiene: some raw `.log` evidence files under `docs/reports/evidence/VM_SPLIT_*` are tracked in Git, against the "keep raw logs out of Git" rule.
8. Data at risk: the VM image has a single copy on an external NTFS drive; the VM's fallback console password is in its setup note.

## 6. Hardware facts

Verified over USB on 2026-10-06 (`hardware/README.md`, bench, propellers off, disarmed): Matek H743-SLIM V3 running **ArduCopter 4.6.2**; gyro, accel, compass, baro healthy; GPS healthy with a 3D fix (23 satellites); X-frame quad, DShot600. Expected pre-arm notices: RC not found, 3D accel calibration needed. The Oct 23 parameter export (`hardware/params/matek_h743_quad_20261023.param`) enables the CAN port (`CAN_P1_DRIVER=1`, DroneCAN node 10) — this is *configuration only*; no CAN message path to a companion has been demonstrated.

On 2026-10-29 the real ESP32 board enumerated over USB as a CP2102 UART bridge (`/dev/ttyUSB0`) (**PHYSICAL, passive enumeration only**). No Linux CAN interface was exposed, the Matek was not attached by USB, and no serial port or CAN traffic was opened/read. The active test stopped because propellers were installed and the flight battery was connected. The Oct 23 export further records DroneCAN at 250 kbit/s and `CAN_D1_UC_SER_EN=0`; node 10 is the autopilot's DroneCAN node ID, not evidence of an ESP32 node or MAVLink command channel. Detailed evidence and the propellers-off plan: `hardware/reports/PHYSICAL_ESP32_CAN_BENCH_PLAN_2026-10-29.md`.

Not verified / open:
- No physical flight has been attempted; RC link and accel calibration not done. One quad is assembled; the department has 10 drones, the team has 2.
- ESP32 role (MAVLink relay vs onboard agent) and the link between the ESP32 and the flight controller (UART vs CAN) are undecided. An MCP2551 alone is only a CAN transceiver.
- Current recommendation: prove a connectorized UART/MAVLink command path first if an approved no-solder harness is available; validate CAN separately as an experimental DroneCAN link. This is a design recommendation, not a verified physical path.
- Free ports per the port-mapping PDF: UART4/6/7/8, I2C1/2, one CAN (from notes, not re-checked on the board).
- Department fleet count remains team-reported (10 total, team has 2); no per-aircraft inventory has been completed. Inventory and bench-check each aircraft before extrapolating results.
- Hardware files are configuration records, not approval to fly. Do not modify them unless assigned.
- A propellers-off bench test of the command path is planned (target: week of 2026-10-30), run in a separate Codex hardware chat.
- Details: `hardware/README.md`, `docs/HARDWARE_SOFTWARE_MAPPING.md`, `docs/PHYSICAL_NETWORK_VALIDATION_PLAN.md`; the 30 Sep / 3 Oct notes in `docs/history/agent-handoffs/` are *superseded* where they say firmware/GPS are unknown.

## 7. Working agreements

**Last known state:** branch `wip/decentralized-sitl-handoff-20261029`; the checkpoint `d1e7444` is tagged `wip-checkpoint-20261029`. Commits since then are docs plus the stale-intent fix (`git log --oneline wip-checkpoint-20261029..HEAD`). Root disk ~35 GB free. Pushed: docs commits up to `70b120d`; the fix commit is local until the user says to push.

- Claude is primary writer for the week of 2026-10-29; Codex reviews. One writer at a time. Log every session in the collaboration log.
- Focused commits; clean tree at handoff; no push/merge/PR/history rewrite without instruction.
- Never `git reset --hard`, `git clean`, force-push, auto-rebase, broad `pkill -9 -f`. Don't delete runs, logs, evidence, backups, branches or hardware files.
- Check disk (`df -h /` — about 35 GB free after the 2026-10-29 move) before full tests, SITL, Gazebo, VM runs. Confirm process/port ownership first.
- Never present mocked/unit results as live SITL evidence.

## 8. Plan

1. ~~Stale/out-of-order MissionIntent fix + regression tests~~ — done 2026-10-29 (UNIT).
2. Transition protocol: spec drafted in `docs/TRANSITION_PROTOCOL_SPEC.md` (2026-10-29, awaiting Codex review and the user's go-ahead); then implement it (defects 3).
3. Fix the runaway retry log (defect 1), then dashboard and mode-supervisor issues (defects 4–5).
4. CI dependency fix (defect 2) and formation-implementation consolidation (defect 6).
5. Final days of the week: validate 5- and 10-drone centralized/decentralized/hybrid with checksummed evidence.
6. Plan a propellers-off hardware milestone; code freeze target 17 Dec, deliverables 31 Dec.

## Changelog (newest first; one line per change to this file)

- 2026-10-29 — Claude — Added `docs/TRANSITION_PROTOCOL_SPEC.md` (draft); sharpened defect 3 with the verified gating gap.
- 2026-10-29 — Codex — Recorded PHYSICAL passive ESP32 USB enumeration, clarified what the Oct 23 DroneCAN settings do not prove, selected UART/MAVLink as the first command-path recommendation, and linked the propellers-off CAN/fleet bench plan.
- 2026-10-29 — Claude — Fixed stale/out-of-order `MissionIntent` handling in `sitl/independent_agent_node.py` (sequence check moved before any state change); added `tests/test_mission_intent_ordering.py` (UNIT). Removed the defect from section 5.
- 2026-10-29 — Claude — Corrected hardware facts (firmware ArduCopter 4.6.2 and GPS fix were bench-verified Oct 6; CAN enabled in Oct 23 params); copied the Matek Oct 23 export and summary into `hardware/`.
- 2026-10-29 — Claude — Created. Consolidated scattered project state; archived the Antigravity checkout and moved heavy logs/runs to the SSD (see `docs/history/ARCHIVE_INDEX.md`).
