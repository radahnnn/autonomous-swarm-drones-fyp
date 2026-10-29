# Archive index — where old and heavy material lives (as of 2026-10-29)

Nothing here is needed to run the project. Active state is in `docs/PROJECT_STATE.md`.

## On the Ubuntu system disk

| Path | Contents | Status |
|---|---|---|
| `/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/` | Organization shelf: `backups/` (hash-listed tarballs), `checkpoints/` (phase1, 3-drones, demo-readiness), `dependencies/` (ArduPilot, Gazebo plugin; ~8 GB), `github/` (Oct-12 mirror), `local-logs/`, `references/` (proposal, reports, defense, hardware photos) | Archive. Its `START_HERE.md` (14 Oct) is outdated — use `docs/PROJECT_STATE.md`. |
| `.../working-copies/demo-master` | `master` demo baseline | Read-only |
| `.../working-copies/hw-matched-identification`, `identification-phase2-20261020`, `experimental-hybrid-20261020` | Research / archive branches | Read-only |
| `/home/drone/Documents/Codex/2026-10-12/here-is-a-summary-you-can/` | Matek H743 parameter exports, parameter summary docx, defense prep note | **Keep** — not elsewhere in the repo |
| `/home/drone/Documents/Codex/2026-10-23`, `2026-10-27` | Unrelated coursework outputs | Not this project |
| `/home/drone/.gemini/antigravity/scratch/` | Two old checkpoint repos (`*_phase1_checkpoint`, `*_3drones_checkpoint`), three early tarballs, and unrelated coursework | Duplicates of the shelf; kept for now |
| `/home/drone/Documents/ChatGPT/drone` | Empty git repo (Codex first setup) | Nothing unique |
| `/home/drone/ardupilot` and the shelf's `dependencies/ardupilot` | ArduPilot SITL builds; recent logs only | Machine-installed |

## On the external SSD `/media/drone/SSDSAM` (NTFS)

| Path | Contents | Verification |
|---|---|---|
| `FYP_Archive/ArduPilot_Logs_2026-10-29/` | 90 ArduPilot `.BIN` logs dated before 2026-10-25 (72 unique, 14 GB), `MANIFEST.tsv` maps original paths | `sha256sum -c OBJECT_CHECKSUMS.sha256` passed 2026-10-29 |
| `FYP_Archive/Antigravity_Original_2026-10-29/` | Git bundle of the original Antigravity checkout (all refs), its two interim-report drafts, ignored SITL logs and `mav.*` | Logs/reports compared identical; bundle verified |
| `FYP_Archive/Repo_Runs_Logs_2026-10-29/` | Five very large raw runs from `runs/independent/` (symlinked back into this repo) and all 501 `logs/*.BIN`; `*.sha256` manifests | Checksums matched before originals were removed |
| `drone-swarm-vm/` | Ubuntu 24.04 QEMU/KVM VM (qcow2, scripts, setup note); used for the host/VM tests | **Single copy; no backup** |

Restore: `cp -a` the object back, or `git clone <bundle>`. If the SSD is not mounted, the five `runs/independent/...` symlinks dangle; nothing runs from them.

## Notes preserved in this repo

`docs/history/agent-handoffs/` — early Claude/Codex handoffs, plus the free-Claude `FYP_LOG` (30 Sep) and `FYP_AGENT_HANDOFF` (3 Oct). These hold early hardware questions and decisions; some content is superseded.
