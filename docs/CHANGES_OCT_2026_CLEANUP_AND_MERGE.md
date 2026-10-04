# What changed on 4 October 2026: cleanup, Phase 2 analysis and branch merge

Read this if you cloned the repo earlier and wonder why files moved or branches disappeared.
Final state: `master` @ `50978ac` is the only branch. Nothing was deleted from history; all earlier work is reachable from `master`.

## 1. Summary

Three lines of work were combined into `master`:

1. **Cleanup (Phase 1)**: repo hygiene, packaging, docs layout.
2. **Phase 2 analysis**: wiring the fitted dynamics into the code and re-running experiments with 20 seeds.
3. **`phase-1-foundations-packaging`** (FYP Swarm Team): config profiles, simulation/SITL parity, scenarios, sensor realism, extra tests. This branch had been developed in parallel and was based on the old `master`.

Where (1)/(2) and (3) changed the same code, **the team branch's version was kept**. My `SwarmConfig`, `fitted_sitl`/`assumed` profile selection by environment variable, and 20-seed CI statistics layer are therefore **not** on `master`; they remain in git history at commit `5e0408f`.

The three feature branches were deleted from GitHub after the merge (`cleanup/phase1-hygiene`, `phase2-config-validation`, `phase-1-foundations-packaging`). Their commits are all contained in `master`. Tags are unchanged (`v0-baseline` marks the state before any of this).

## 2. Cleanup changes (kept on `master`)

| Change | Details |
|---|---|
| `.pyc` files | The 13 tracked `__pycache__/*.pyc` files were removed from git; `.gitignore` extended (egg-info, build, venvs, coverage, `*.tar.gz`) |
| Packaging | `pyproject.toml` with `pip install -e ".[dev]"` / `".[sitl,dev]"`; pytest collects only `tests/` |
| Hardcoded paths | Removed `/home/drone/...` and the `.gemini` "brain" artifact copies from experiments; SITL binary from `ARDUCOPTER_BIN` / `ARDUPILOT_HOME`; launch scripts find `swarm_params.parm` next to themselves; `launch_gazebo.sh` uses `ARDUPILOT_GAZEBO_DIR` |
| Script renames | `sitl/test_telemetry.py` -> `sitl/diag_telemetry.py`, `sitl/test_takeoff_diag.py` -> `sitl/diag_takeoff.py` (pytest must never collect hardware diagnostics) |
| Line endings | `.gitattributes` forces LF for `*.sh` and `*.py` |
| Docs layout | Root-level reports moved: `TASK_B/C/E_*.md` -> `docs/reports/`, `FYP_RUNNING_LOG.md` -> `docs/logs/`, `ACADEMIC_REVIEW_RESPONSE.md`, `CODEX_REVIEW_BRIEF.md`, `SUB_SUPERVISOR_MEETING_BRIEF.md` -> `docs/`. Absolute `file:///home/drone/...` links were rewritten as relative links. Empty `FYP_SETUP_LOG.md` deleted |
| Test fix | `tests/test_sitl_adapter.py`: the mocked heartbeat now carries `type`/`autopilot` so `mode_string_v10` yields GUIDED; previously the adapter's GUIDED-mode check blocked setpoints and the test failed |

## 3. Phase 2 analysis (kept as documentation)

`docs/reports/PHASE2_CONFIG_VALIDATION_REPORT.md` compares the original dynamics (tau 0.18 s, cd 0.20/s) with the SITL-fitted ones (tau 0.992 s, cd 0.637/s) over 20 seeds. Raw per-seed CSVs: `docs/reports/phase2_outputs/raw/`; stdout of both runs and the pre-change baseline are in `docs/reports/phase2_outputs/`.

Key findings:
- The controller ranking is unchanged: hybrid ~ centralized under good links, both far better than decentralized; the naive hybrid chatters (92-361 mode switches).
- Absolute errors grow with fitted dynamics (hybrid at 0% loss: 0.002 m -> 0.080 m; decentralized: 0.47 m -> 1.9 m).
- The "1.5 m minimum clearance at 1.5 m GPS noise" claim is not supported (mean minimum separation 1.475 +/- 0.128 m with fitted dynamics). No collisions in any run.
- The hybrid does not beat plain hold-last-command for 1-2 s outages; it wins only at 3 s.
- The analytic steady-state lag formula matches the measurement (1.347 m predicted, 1.343 m measured).

## 4. What came from the team branch

Config profiles with provenance (`assumed_baseline`, `sitl_default_quad`), simulation/SITL parity pipeline and report (`docs/SIMULATION_SITL_PARITY.md`), SITL scenarios, velocity setpoints, flight-safety guardrails, sensor/estimation realism, the headline outage experiment, and the extra tests (`test_config_and_safety.py`, `test_simulation_sitl_parity.py`). See its commit messages in `git log`.

## 5. Things to know / open items

- **Default profile**: `master` defaults to `assumed_baseline` (tau 0.18 s). Thesis numbers should use the fitted profile; the Phase 2 report shows the assumed default flatters accuracy.
- Controller gains (kp 1.8, kd 2.2) were not retuned for the slower fitted dynamics.
- The 20-seed / 95% CI statistics and raw-CSV logging are not in the current experiment scripts (see `5e0408f` for a working version).
- Commit messages of some commits carry a `Co-Authored-By: Claude` trailer, so GitHub lists "claude" under Contributors. This is credit text only and gives no repository access.
- If you have an old clone: `git fetch --prune origin && git checkout master && git reset --hard origin/master` (this discards local uncommitted work, so copy anything you need first).
