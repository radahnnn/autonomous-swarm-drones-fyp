# Project state — start here

**Last verified:** 2026-10-29, branch `wip/decentralized-sitl-handoff-20261029`, 312 unit/integration tests passing (`python3 -m pytest -q tests`).
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
| Matek H743 parameter exports and diagnostics | `hardware/` (repo) and `/home/drone/Documents/Codex/2026-10-12/here-is-a-summary-you-can/` (not in repo) | Do not change hardware files unless assigned. |
| GitHub | `https://github.com/radahnnn/autonomous-swarm-drones-fyp` | Do not push/merge/PR without the user's instruction. |

Git branches (remote): `master` (demo), `work/decentralized-sitl-20261020` (parent of this branch, `cb7fdcc`), `wip/decentralized-sitl-handoff-20261029` (this branch, WIP checkpoint `d1e7444`), `work/corrected-phase2-20261020`, `hw-matched-identification`, `archive/*`, `checkpoint/2d-working-savepoint`. This branch is 24 commits ahead of / 4 docs-only commits behind `origin/master`; README conflicts are expected on merge.

## 3. Code map (this checkout)

- `swarm_core/` — numerical foundation: config profiles, drone model, graph/Laplacian, network emulator, formations + Hungarian assignment, centralized/decentralized/hybrid controllers, safety.
- `simulator/` — fast numerical engine and plots.
- `sitl/` — ArduPilot integration. Key files: `swarm_console.py` (central pilot console), `independent_swarm_console.py` + `independent_agent_node.py` (one process per drone), `mission_intent.py` (coordinator→agent mission message), `peer_udp.py` (drone↔drone UDP), `decentralized_guidance.py`, `operator_dashboard.py` (browser UI), `flight_prep.py`, `fleet.py`, `formation_library.py`.
- `experiments/` — validation and sweep scripts (SITL, burst outage, VM split, operator loss, restart, partition).
- `tests/` — 312 tests. Mostly UNIT/INTEGRATION; they do not prove live SITL behaviour.
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
| Physical prototype | **Not started** | none | One quad assembled, never flown. ESP32/CAN link is a proposal; no command path confirmed. MCP2551 alone is only a CAN transceiver. |

## 5. Known defects and risks (current)

Tracked as the backlog in `docs/history/agent-handoffs/CLAUDE_ONE_WEEK_PROMPT_2026-10-29.md`. Confirmed on 2026-10-29:

1. **Stale/out-of-order MissionIntent mutates state before validation** — `sitl/independent_agent_node.py` lines ~405–441: formation, slots and transition fields are changed before `intent.sequence <= self.last_mission_sequence` is checked. **Not yet fixed.**
2. **Runaway log on dropped SITL link** — run `external-sitl-20261022T193321` wrote ~54 M lines/agent (4.6 GB) of `EOF on TCP socket`; no backoff or log limit in the retry path.
3. **CI cannot install pymavlink** — `.github/workflows/ci.yml` installs `.[dev]` only, while tests import `sitl`. Python 3.10–3.12 matrix is unverified.
4. No prepare/ack/common-start/abort for synchronized transitions; no live transition supervision; commands not blocked during transitions; link-loss/coordinator-loss/membership semantics undefined.
5. Dashboard: `session.count` vs `SwarmConsole.n` mismatch; commands lack structured results.
6. Mode supervisor has no child-readiness check or rollback; process cleanup is broad rather than ownership-based.
7. Several formation/slot-assignment implementations exist (`swarm_core/formations.py`, `sitl/formation_library.py`, others).
8. Repo hygiene: some raw `.log` evidence files under `docs/reports/evidence/VM_SPLIT_*` are tracked in Git, against the "keep raw logs out of Git" rule.
9. Data at risk: the VM image has a single copy on an external NTFS drive; the VM's fallback console password is in its setup note.

## 6. Hardware facts (from Sep–Oct notes; unverified unless stated)

- Flight controller: Matek H743-SLIM V3. Firmware currently on the boards: **unknown** (Betaflight vs ArduPilot not confirmed). GPS/compass fitted: **unknown**. The department has 10 drones; the team has 2 (one assembled, unflown).
- Free ports per the port-mapping PDF: UART4/6/7/8, I2C1/2, one CAN. ESP32 role (MAVLink relay vs onboard logic) undecided.
- The repo's parameter export `hardware/params/matek_h743_quad_20261006.param` and later exports are *configuration records*, not approval to fly.
- A propellers-off bench test of the command path is planned (target: week of 2026-10-30).
- Details and history: `docs/history/agent-handoffs/FYP_AGENT_HANDOFF_2026-10-03_free-claude.md`, `docs/HARDWARE_SOFTWARE_MAPPING.md`, `docs/PHYSICAL_NETWORK_VALIDATION_PLAN.md`.

## 7. Working agreements

**Last known state:** branch `wip/decentralized-sitl-handoff-20261029`; code unchanged since checkpoint `d1e7444` (log commit `2efa526`, prompt commit `c97c563`); this cleanup is the next commit. Tests: 312 passed. Root disk: ~35 GB free. Remote `origin` = the GitHub repo above; nothing from 2026-10-29 has been pushed beyond `c97c563`.

- Claude is primary writer for the week of 2026-10-29; Codex reviews. One writer at a time. Log every session in the collaboration log.
- Focused commits; clean tree at handoff; no push/merge/PR/history rewrite without instruction.
- Never `git reset --hard`, `git clean`, force-push, auto-rebase, broad `pkill -9 -f`. Don't delete runs, logs, evidence, backups, branches or hardware files.
- Check disk (`df -h /` — about 35 GB free after the 2026-10-29 move) before full tests, SITL, Gazebo, VM runs. Confirm process/port ownership first.
- Never present mocked/unit results as live SITL evidence.

## 8. Plan

1. Fix stale-intent ordering + regression tests (defect 1).
2. Write the transition-protocol spec (prepare/ack/common-start/abort) for Codex review, then implement with supervision and failure semantics (defects 4–6, 2).
3. CI fix (defect 3) and formation-implementation consolidation (defect 7).
4. Final days of the week: validate 5- and 10-drone centralized/decentralized/hybrid with checksummed evidence.
5. Plan a propellers-off hardware milestone; code freeze target 17 Dec, deliverables 31 Dec.

## Changelog (newest first; one line per change to this file)

- 2026-10-29 — Claude — Created. Consolidated scattered project state; archived the Antigravity checkout and moved heavy logs/runs to the SSD (see `docs/history/ARCHIVE_INDEX.md`).
