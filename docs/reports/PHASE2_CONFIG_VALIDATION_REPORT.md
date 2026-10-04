# Phase 2 Report: Fitted Dynamics Wired In, Safety Constants Unified, 20-Seed Statistics

Branch: `phase2-config-validation` (baseline: `master` @ `93addaa`, tag `v0-baseline` = pre-cleanup).

> **Provenance note.** These results were produced with the standalone Phase 2 implementation (branch `phase2-config-validation`, commit `5e0408f`: `SwarmConfig`, `fitted_sitl` / `assumed` profiles, 20 seeds with 95% CIs). That branch was later merged with the team's `phase-1-foundations-packaging` branch, whose profile system (`assumed_baseline`, `sitl_default_quad`) replaced it; the physical parameters are identical (tau = 0.992 s, cd = 0.637 /s for the fitted profile). The experiment scripts on `master` use the team's version, which does not include the 20-seed CI/raw-CSV layer (`exp_stats.py`, `run_all.py`, still available in commit `5e0408f`). Raw per-seed CSVs are in `phase2_outputs/raw/`. Note the merged default profile is `assumed_baseline`, not the fitted one.

## 1. What changed

| Item | Before | After |
|---|---|---|
| Vehicle dynamics | `Drone` hardcoded tau=0.18 s, cd=0.20 /s; `config.py` (fitted 0.992 / 0.637) imported nowhere | `SwarmConfig` in `swarm_core/config.py` is the single source. `Drone`, controllers, hybrid supervisor, metrics, engine and SITL adapter read it. Default profile = `fitted_sitl` |
| Profiles | none | `fitted_sitl` (tau=0.992, cd=0.637) and `assumed` (0.18, 0.20). Select with `SWARM_DYNAMICS_PROFILE` |
| APF safety radius | 1.0 m (centralized), 1.2 m (decentralized / hybrid), 2.5 m (unused in config) | one `apf_radius = 1.2 m` |
| Collision threshold | 0.70 literal in several places | derived: `2 * drone_radius` |
| Formation spacing | 2.5 (engine) vs 3.5 (SITL adapter) literals | `nominal_spacing = 2.5`, `sitl_formation_spacing = 3.5` (explicit, named) |
| Feedforward drag | engine did not pass drag to controllers (they used a literal 0.20) | engine passes the vehicle's own `drag_coeff` |
| Seeds | 6 per condition, +/- = std-dev | 20 per condition (`SWARM_NUM_SEEDS`), +/- = 95% CI (Student t) |
| Raw data | PNGs only | per-seed CSVs `experiments/results/*_raw.csv` |
| Ablation derivation | `cd = 0.20` hardcoded | taken from config, so the analytic prediction follows the model |
| Tests | 15 | 25 (`tests/test_config.py`: config is what the code actually uses) |

## 2. Results: assumed vs fitted dynamics (20 seeds, mean +/- 95% CI)

Raw stdout of both runs is in `phase2_outputs/`; pre-change numbers (6 seeds) in `phase2_outputs/baseline_pre_phase2/`.

### 2.1 Packet-loss sweep, final formation error (m), moving target + V->Line morph

| Controller | Loss | assumed | fitted_sitl |
|---|---|---|---|
| Centralized (hold last) | 0% | 0.043 +/- 0.000 | 0.083 +/- 0.003 |
| Centralized | 50% | 0.042 +/- 0.000 | 0.111 +/- 0.004 |
| Decentralized | 0% | 0.473 +/- 0.010 | 1.898 +/- 0.062 |
| Decentralized | 50% | 0.293 +/- 0.010 | 1.127 +/- 0.042 |
| Hybrid (hysteresis) | 0% | 0.002 +/- 0.000 | 0.080 +/- 0.002 |
| Hybrid (hysteresis) | 50% | 0.046 +/- 0.006 | 0.118 +/- 0.010 |
| Naive hybrid | 20% | 0.019 +/- 0.002, 92 switches | 0.091 +/- 0.004, 92 switches |
| Naive hybrid | 50% | 0.075 +/- 0.005, 361 switches | 0.143 +/- 0.014, 361 switches |

### 2.2 Outages (5 drones, steady-state error after recovery, m)

| Outage | Hold-last (assumed) | Hybrid (assumed) | Hold-last (fitted) | Hybrid (fitted) |
|---|---|---|---|---|
| 1 s | 0.078 | 0.083 | 0.167 | 0.342 |
| 2 s | 0.077 | 0.084 | 0.229 | 0.355 |
| 3 s | 0.081 | 0.130 | 0.868 | 0.659 |

### 2.3 GPS noise sigma = 1.5 m, minimum separation (m, mean of per-seed minima +/- CI)

| Controller | assumed | fitted_sitl |
|---|---|---|
| Hybrid | 1.519 +/- 0.121 | 1.475 +/- 0.128 |
| Centralized | 1.517 +/- 0.120 | 1.435 +/- 0.141 |
| Decentralized | 1.542 +/- 0.120 | 1.520 +/- 0.108 |

Collisions (< 0.70 m): 0/20 in every condition.

### 2.4 Feedforward ablation (analytic check)

Predicted steady lag without feedforward, e = (kd + cd)/kp * |v|:
- assumed: 1.139 m predicted, 1.140 m measured
- fitted: 1.347 m predicted (cd = 0.637), 1.343 m measured (hybrid, no feedforward)

With feedforward the steady error is 0.018 m (assumed) but 0.098 m (fitted). The feedforward only cancels drag; the larger lag tau = 0.992 s now leaves residual error.

## 3. Findings (what this means for the claims)

1. **The ranking is robust.** Hybrid ~ centralized under good links, both far better than decentralized, and the naive hybrid chatters (92 to 361 switches) while the hysteresis design stays near 0 to 1. This holds under both profiles.
2. **Absolute accuracy degrades by roughly 2x to 40x with fitted dynamics.** Hybrid at 0% loss goes from 0.002 m to 0.080 m; decentralized from 0.47 m to 1.9 m. Any thesis number quoted from the old defaults (e.g. "0.002 m") describes a vehicle that was too responsive. Report fitted-profile numbers.
3. **The 1.5 m clearance claim is not supported.** With fitted dynamics at sigma = 1.5 m the mean per-seed minimum separation is 1.475 +/- 0.128 m (below 1.50 m); even the assumed profile only gives 1.519 +/- 0.121 m, a CI that straddles 1.5 m. No collisions occur (0.70 m threshold), so word it as "no collisions, minimum separation about 1.4 to 1.5 m".
4. **Hybrid does not beat plain hold-last-command for short outages.** For 1 s and 2 s blackouts hold-last has lower post-recovery error under both profiles; the hybrid only wins at 3 s with fitted dynamics (0.659 vs 0.868 m). The hybrid's value is robustness to long outages and burst loss, plus bounded behavior; the thesis should not claim it wins for short outages. Its 2.0 s dwell time is the likely cost and is a candidate for tuning.
5. **The analytic lag derivation is validated again** against the fitted model (1.347 vs 1.343 m), which also confirms the simulator is now consistent with the config.
6. **Open item:** the controller gains (kp = 1.8, kd = 2.2) were chosen for the old fast dynamics. They remain stable with tau = 0.992 s but are not retuned; retuning is the next improvement.

## 4. Reproduce

```bash
pip install -e ".[dev]"
python experiments/run_all.py                                   # fitted dynamics, 20 seeds (about 6 min)
SWARM_DYNAMICS_PROFILE=assumed python experiments/run_all.py    # original dynamics
SWARM_NUM_SEEDS=5 python experiments/run_all.py                 # quick smoke run
```

Note: `experiments/results/` holds the fitted-profile figures and raw CSVs (the default). Raw CSVs of the assumed profile are in `experiments/results/assumed_profile/`.
