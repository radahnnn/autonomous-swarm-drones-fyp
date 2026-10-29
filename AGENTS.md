# Instructions for AI agents (Codex and others)

Read `docs/PROJECT_STATE.md` first — it is the single source of truth for what this project is, where everything lives, current status, known defects, safety rules and the plan. Section 0 tells you how to start and how to keep it updated.

Then read the rules and the latest entry in `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`.

Essentials: one writer at a time; work only in this checkout on branch `wip/decentralized-sitl-handoff-20261029`; never push/merge/PR/rewrite history without the user's instruction; no `git reset --hard`, `git clean`, force-push, or broad `pkill`; do not delete runs, logs, evidence, backups, branches or hardware files; label evidence honestly (UNIT/INTEGRATION/SITL/QGC/GAZEBO/VM-NETWORK/PHYSICAL); update `docs/PROJECT_STATE.md` and the collaboration log before you finish.
