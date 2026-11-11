# Antigravity handoff — swarm-drone FYP — 2026-11-11

## Start here

Open the existing local checkout:

`/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/working-copies/decentralized-sitl-20261020`

Branch: `wip/decentralized-sitl-handoff-20261029`. Repository: `radahnnn/autonomous-swarm-drones-fyp`. `master` is the frozen demo fallback. The reviewed source tip before this documentation handoff was `ec575ae`; verify the current local and GitHub tips before editing.

Read in order: `AGENTS.md`; all of `docs/PROJECT_STATE.md` (especially sections 0, 4, 5, 7, 8); the latest entries of `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`; `docs/SOFTWARE_ACCEPTANCE_MATRIX.md`; `docs/AGENT_PROTOCOL.md`; then the reports linked by the state file. The collaboration log contains the 2026-11-11 Codex review. Do not treat this short handoff as a substitute for those files.

One writer at a time. Check branch, status, remotes, disk space and active processes before work. Preserve all runs, logs, backups, hardware records and other checkouts. No reset --hard, clean, force-push or broad process kills. The user authorized the 2026-11-11 WIP branch push, not a `master` merge or a physical flight.

## What works and what remains unproven

- The active `sitl/` independent runtime supports centralized, decentralized and hybrid modes, 10-drone standalone SITL, 6-drone Gazebo, operator dashboard, formation transitions, and individual agents. Formation keeping and goal propagation have leader-follower peer consensus; formation changes and tower-gap decisions are still planned by the console.
- The 2026-11-11 review ran 697 UNIT/INTEGRATION tests successfully with loopback sockets enabled; `pyflakes` passed. These tests do not establish live safety. The Codex sandbox initially blocked local sockets and caused 22 environment failures; the normal-socket rerun passed.
- Local overnight evidence: `runs/independent/night-20261105/SUMMARY.md`. Calm 10-drone SITL: 10/10 strict PASS. Realistic two-layer SITL: 6 PASS, 2 FAIL, 2 INCOMPLETE of 10. Early-switch timing: 3/3 PASS against baseline 0/3 (exploratory). Gazebo patrol, Gazebo gap and grid-check: one strict PASS each. Video and detailed run data remain in the local `runs/independent/night-20261105/` directory and are not in Git.
- In layered runs, `SIMSTATE` supplies true horizontal position but no true altitude. The 0.89 m worst true horizontal gap cannot prove either 3-D collision or 3-D clearance. Estimated ellipsoid separation uses the autopilot's altitude estimate. Do not claim verified 3-D safety from these runs.
- Physical state: one assembled drone, no flight, no proven PC/ESP32/Matek command path. The Oct 29 hardware report describes passive ESP32 USB enumeration and a proposed props-off bench plan. MCP2551 alone is a transceiver. Physical flight is required for the final grade but remains a separate supervised hardware workstream.
- Main review risks: broad `pkill -9 -f arducopter` in `sitl/sitl_launcher.py`; two-layer assignment does not separate every close pair in the triangle formation; future mixed 2-D/3-D truth inputs can fail in `sitl/run_metrics.py`. See the 2026-11-11 collaboration-log entry for file lines and actions.

## File map and next work

- `sitl/swarm_console.py` is the current CLI entry point; `sitl/independent_swarm_console.py` manages operator commands; `sitl/independent_agent_node.py` is each vehicle's agent; `sitl/sitl_launcher.py` starts SITL. Retired local/process runtimes are preserved in `archive/legacy_runtimes/`.
- `swarm_core/` and `simulator/` are the numerical model, `experiments/` contains studies, `scenarios/` contains scripted live missions, `tests/` is the unit/integration suite, and `docs/PARAMETERS.md` inventories many constants.
- Follow the approved items in `docs/PROJECT_STATE.md` section 8. First resolve evidence validity for layered 3-D separation; repeat matched realistic scenarios; then address open safety and protocol defects. Keep claims at their measured evidence level. Ask the user before adding new plan items, per the section 8 rule.
- The latest detailed code/evidence review is appended to `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`. After each session, update that log and `docs/PROJECT_STATE.md` with verified facts.

## Local versus GitHub

GitHub holds tracked source and documentation on the WIP branch after the 2026-11-11 push. Local raw runs, `.BIN` evidence, videos, caches and backups are deliberately outside Git and must not be staged wholesale. The large `.BIN` archive is at `/media/drone/SSDSAM/FYP_Archive/Runs_BIN_2026-11-04/`. A local `runs/independent/BIN_LOGS_MOVED.txt` note points to it. The separate demo and research checkouts under `working-copies/` are preserved.
