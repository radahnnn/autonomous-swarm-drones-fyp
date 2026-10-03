# Baseline Audit

## Audit date
October 3, 2026

---

## Repository state
- **Branch**: `master` (synchronized with `origin/master`)
- **Commit**: `f9ed7e35ed1d3cd99d3e345f266641a9a06d696b` (`docs: add sub-supervisor meeting briefing guide`)
- **Working Tree State**:
  - Tracked files are clean (no modified tracked files).
  - Untracked file in root: `swarm_drones_fyp_save_20261001.tar.gz` (18 MB backup tarball).
  - Tracked `.pyc` files in git index: 13 compiled Python bytecode files (`*.pyc`) inside `simulator/__pycache__/`, `swarm_core/__pycache__/`, and `swarm_core/controllers/__pycache__/` are currently committed and tracked by git despite `.gitignore`.
  - Ignored runtime artifacts present on disk: `.pytest_cache/`, `logs/*.BIN` (22 ArduPilot dataflash logs), `terrain/S36E149.DAT` (SRTM elevation cache), `eeprom.bin`, `mav.parm`, `mav.tlog`, `mav.tlog.raw`.

---

## Environment
- **Operating System**: Ubuntu 24.04.5 LTS (Noble Numbat), Linux Kernel `7.0.0-34-generic` x86_64
- **Python Version**: Python 3.12.3 (Active executable: `/home/drone/venv-ardupilot/bin/python3`)
- **Pip Version**: pip 26.2.1 (from `/home/drone/venv-ardupilot/lib/python3.12/site-packages/pip`)
- **Installed Dependencies**:
  - `numpy`: 2.5.3 (requirement in `requirements.txt`: `>=1.24.0`)
  - `scipy`: 1.18.1 (requirement in `requirements.txt`: `>=1.10.0`)
  - `matplotlib`: 3.11.2 (requirement in `requirements.txt`: `>=3.6.0`)
  - `pymavlink`: 2.4.50 (installed in venv and `~/.local`; **omitted from `requirements.txt`**)
  - `pytest`: 7.4.4 (installed in venv; **omitted from `requirements.txt`**)
  - `pytest-cov`: 4.1.0 (installed in venv)
  - `PyYAML`: 6.0.3 (installed in venv)
- **External Tools Verification**:
  - **ArduPilot SITL**: Binary exists at `/home/drone/ardupilot/build/sitl/bin/arducopter` (ELF 64-bit, 6.16 MB, ArduPilot V4.6.0-beta1 / V4.8.0-dev). **Not installed on system `$PATH`**.
  - **Gazebo**: Gazebo Sim version 8.15.0 (Harmonic) verified at `/usr/bin/gz`.
  - **pymavlink**: Installed and importable via Python 3.12 (`2.4.50`).
  - **pytest**: Installed and operational (`7.4.4`).
  - **QGroundControl**: AppImage binary verified at `/home/drone/QGroundControl.AppImage` (190.5 MB, dated Sep 5, 2024). Not on system `$PATH`.

---

## Test baseline

### 1. Documented Commands (from `README.md` and `CLAUDE.md`)
| Command | Result | Duration | Notes |
| :--- | :--- | :--- | :--- |
| `PYTHONPATH=. python3 tests/test_graph.py` | **PASSED** | 0.45s | 2 test functions executed directly. |
| `PYTHONPATH=. python3 tests/test_formations.py` | **PASSED** | 0.38s | 2 test functions executed directly. |
| `PYTHONPATH=. python3 tests/test_simulation.py` | **PASSED** | 1.15s | 4 test functions executed directly; 1 warning emitted (`UserWarning: Unable to import Axes3D`). |
| `PYTHONPATH=. pytest tests/` | **PASSED** | 1.07s | 15 tests collected, 15 passed, 0 failed, 1 warning. |

### 2. Standard Commands Without Path Configuration
| Command | Result | Collected | Passed | Failed | Error / Root Cause |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `python3 tests/test_graph.py` | **FAILED** | 0 | 0 | 0 | `ModuleNotFoundError: No module named 'swarm_core'`. `sys.path[0]` is set to `tests/`, and package is not installed via `pip install -e .` (no `setup.py` / `pyproject.toml`). |
| `python3 -m pytest -q` *(from repo root)* | **CRITICAL FAILURE** | N/A | 0 | 0 | **`SystemExit: 1` during test collection**. Pytest recursively scans root and collects `sitl/test_takeoff_diag.py`. This script has un-encapsulated top-level executable code that executes at import time, attempts to kill `arducopter`, spawns SITL, tries to arm/take off, times out, and calls `sys.exit(1)`. Pytest collection crashes completely; zero tests run. |
| `python3 -m pytest tests/ -q` | **PASSED** | 15 | 15 | 0 | When constrained explicitly to `tests/`, all 15 tests pass in 1.06s with 1 warning (`Axes3D`). |

### 3. Detailed Test Inventory & Code Coverage
Running `python3 -m pytest tests/ --cov=swarm_core --cov=simulator --cov=sitl --cov-report=term-missing` demonstrates:
- **Total Test Count**: 15 tests (100% pass rate when invoked on `tests/`).
- **Total Statements**: 2,001 statements across repo modules.
- **Statements Missed**: 1,212 statements.
- **Overall Code Coverage**: **39%**.
- **Coverage by Module**:
  - `swarm_core/`: **91%** coverage (`formations.py` 99%, `graph.py` 94%, `metrics.py` 94%, `controllers/hybrid.py` 100%, `controllers/decentralized.py` 100%, `controllers/centralized.py` 83%, `drone.py` 88%, `network.py` 72%).
  - `swarm_core/config.py`: **0% coverage** (Never imported or executed by any test or module).
  - `simulator/engine.py`: **100% coverage**.
  - `simulator/visualizer.py`: **11% coverage**.
  - `sitl/common_frame.py`: **100% coverage**.
  - `sitl/mavlink_swarm_adapter.py`: **55% coverage** (mock tests cover adapter dispatch, but network loop and physical connection branches are unexecuted).
  - `sitl/autonomous_wingman.py`, `sitl/interactive_swarm_flight.py`, `sitl/live_radar.py`, `sitl/sitl_bridge.py`, `sitl/swarm_3_drones.py`, `sitl/sync_hud.py`, `sitl/test_takeoff_diag.py`, `sitl/test_telemetry.py`: **0% coverage**.
- **SITL Dependency**: None of the 15 unit tests in `tests/` depend on a live ArduPilot process; all MAVLink communications in `tests/test_sitl_adapter.py` are executed against Python `unittest.mock.MagicMock` objects.

---

## Experiment baseline

Four deterministic experiments were benchmarked:

| Experiment Script | Command | Runtime | Random Seed | Generated Files | Reproducibility & Output Validation |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Formation Transition Demo** | `PYTHONPATH=. python3 experiments/run_demo.py` | 2.57s | Fixed (`np.random.default_rng(123)`) | `experiments/results/formation_line.png`<br>`experiments/results/formation_v_shape.png`<br>`experiments/results/formation_circle.png`<br>`experiments/results/formation_grid.png`<br>`experiments/results/demo_metrics.png` | **Fully Reproducible**. Min distance: 1.751m (> 0.70m threshold). Collisions: 0. Final formation error: 0.002m. Matches documentation. |
| **Feedforward Ablation** | `PYTHONPATH=. python3 experiments/test_feedforward_ablation.py` | 5.70s | Fixed (`seed=42`) | `experiments/results/feedforward_ablation_comparison.png` | **Fully Reproducible**. Analytically derived steady lag $1.1392\text{ m}$ matches empirical $1.140\pm 0.000\text{ m}$ without feedforward, dropping to $0.018\pm 0.000\text{ m}$ with feedforward ($98.4\%$ reduction). |
| **Outage & Burst Loss Sweep** | `PYTHONPATH=. python3 experiments/test_burst_outage_sweep.py` | 18.4s | Fixed (`seed=42`) | `experiments/results/outage_burst_comparison.png` | **Fully Reproducible**. Evaluates 4 baselines over 1s, 2s, 3s complete outages. Hybrid recovers in $0.85\text{ s}$ without chattering. |
| **GPS Noise Sweep** | `PYTHONPATH=. python3 experiments/test_gps_noise_sweep.py` | 14.2s | Fixed (`seeds=[101..110]`, 10 seeds) | `experiments/results/gps_noise_sweep_comparison.png` | **Fully Reproducible**. Evaluates $\sigma \in \{0.04, 0.5, 1.5, 2.5\}\text{ m}$. Min clearance under $\sigma=1.5\text{ m}$ is $1.52\text{ m}$ (> 1.50m threshold). |

*Warning Observed Across All Experiments*:
`UserWarning: Unable to import Axes3D. This may be due to multiple versions of Matplotlib being installed...` emitted by matplotlib projection loader (non-fatal, 2D plotting unaffected).

---

## Architecture summary

The framework operates across two execution pipelines sharing mathematical principles:

```
[Simulation Pipeline]
Target Geometry (FormationGenerator)
       │
       ▼
Wireless Channel Emulator (Gilbert-Elliott Loss + Latency Queue + Outage Injector)
       │
       ▼
Per-Drone Hybrid Supervisor (Asymmetric Hysteresis, Sliding Window w=20 / θ>=70%, Dwell-Time Lockout 2.0s)
       │
       ├── Centralized PD Tracking + Hungarian Optimal Slot Assignment + Feedforward (cd * v_target)
       └── Decentralized Laplacian Consensus (L = D - A) + Reynolds Cohesion + APF Safety Barrier
       │
       ▼
Continuous Linear Blending: u = α(t)*u_central + (1 - α(t))*u_decentral
       │
       ▼
Second-Order Vehicle Dynamics: dot{p} = v, dot{v} = a - cd*v, dot{a} = (a_cmd - a)/τ
       │
       ▼
Metrics Tracker: RMS Tracking Error, Min Separation, Algebraic Connectivity λ2(L), Chattering Switch Count

[SITL Pipeline]
ArduPilot SITL Fleet (SysID 1..3 @ TCP 5760..5780 / UDP 14552..14572)
       │
       ▼ (GLOBAL_POSITION_INT @ 10 Hz)
CommonCoordinateFrame (WGS84 GPS -> Shared Metric Tangent NED Plane, < 0.1 mm precision)
       │
       ▼
MAVLinkSwarmAdapter (Translates acceleration commands u to local NED position targets)
       │
       ▼ (SET_POSITION_TARGET_LOCAL_NED #84)
ArduPilot GUIDED Mode Position Controller (PSC_POSXY_P, PSC_VELXY_P, EKF3 Navigation)
```

---

## Findings

### BLOCKER
1. **Pytest Root Discovery Crashes Test Suite (`sitl/test_takeoff_diag.py`)**:
   Standard execution of `pytest` or `python3 -m pytest -q` from repository root fails completely. Pytest discovers `sitl/test_takeoff_diag.py` because of its filename prefix. The script executes top-level procedural code upon module import, invokes `pkill -9 arducopter`, launches an ArduPilot binary, attempts motor arming, times out, and raises an unhandled `SystemExit: 1`. Zero tests run unless the user explicitly passes `tests/`.
2. **Missing `pyproject.toml` / `setup.py` & Module Import Failure**:
   The repository cannot be installed as an editable package (`pip install -e .`). Direct script execution (e.g., `python3 tests/test_graph.py`) fails immediately with `ModuleNotFoundError: No module named 'swarm_core'`. Every command requires manual prepending of `PYTHONPATH=.`.

### HIGH
3. **Detached Configuration Architecture (`swarm_core/config.py`)**:
   `swarm_core/config.py` was constructed to document parameter provenance (`"fitted from SITL"`, `"assumed"`), but it is **not imported or referenced anywhere in the repository** (0% code coverage). Classes in `swarm_core/drone.py`, `swarm_core/controllers/`, and `simulator/engine.py` use hardcoded default arguments. In particular, `Drone.__init__` still defaults to $\tau = 0.18\text{ s}$ and $c_d = 0.20\text{ s}^{-1}$, while `config.py` specifies the fitted values $\tau = 0.992\text{ s}$ and $c_d = 0.637\text{ s}^{-1}$.
4. **Discrepancy Between SITL Adapter and Core Simulation Control Pipeline**:
   Documentation asserts that SITL executes the "exact same" control pipeline as `swarm_core`. In reality:
   - In centralized mode, `sitl/mavlink_swarm_adapter.py` (line 262) assigns target slots using fixed index mapping (`tgt = world_slots_3d[i]`), completely omitting the Hungarian optimal matching algorithm (`assign_optimal_slots`) executed in `swarm_core`.
   - In hybrid mode, `mavlink_swarm_adapter.py` (line 277) converts acceleration commands into position waypoints using a magic multiplier: `tgt = pos + u_hyb * dt * 2.0`. ArduPilot executes internal velocity/acceleration clamping on position setpoints, meaning vehicle dynamics in SITL are governed by ArduPilot's inner position controller rather than direct dynamic integration.
5. **Hardcoded User Paths and Brain Artifact Destinations**:
   Multiple files contain hardcoded absolute filesystem paths specific to the developer's environment:
   - `/home/drone/.local/lib/python3.12/site-packages` is hardcoded into `sys.path` in `sitl/test_takeoff_diag.py` and `experiments/validate_*.py`.
   - `/home/drone/swarm_drones_fyp/sitl/swarm_params.parm` is hardcoded in `sitl/launch_drone*.sh`, causing launch failures when cloned in any path other than `~/swarm_drones_fyp`.
   - `/home/drone/.gemini/antigravity/brain/28220ca6-e68a-487a-8a59-6e79ee58f6f6/...` is hardcoded in 6 experiment scripts for plot copying.
6. **Incomplete Dependency Manifest (`requirements.txt`)**:
   `requirements.txt` contains only `numpy`, `scipy`, and `matplotlib`. Critical packages required to run tests and SITL integration (`pytest`, `pytest-cov`, `pymavlink`, `PyYAML`) are omitted.

### MEDIUM
7. **Inconsistent Safety Radii and Thresholds**:
   - `swarm_core/config.py`: `collision_threshold = 1.5 m`, `drone_radius = 0.35 m`.
   - `swarm_core/controllers/centralized.py`: `collision_dist = 1.0 m`.
   - `swarm_core/controllers/decentralized.py`: `safe_radius = 1.2 m`.
   - `swarm_core/metrics.py`: `collision_threshold = 0.70 m` (or `drone.radius * 2 = 0.70 m`).
   - Formation spacing defaults vary between $2.5\text{ m}$ (`simulator/engine.py`) and $3.5\text{ m}$ (`sitl/mavlink_swarm_adapter.py`).
8. **Hardcoded Drone Fleet Limitations in SITL**:
   While `swarm_core` supports arbitrary $N$ agents, `sitl/mavlink_swarm_adapter.py` hardcodes `DRONE_SPECS` to exactly 3 drones and fixed UDP ports (`14552`, `14562`, `14572`). `sitl/swarm_3_drones.py`, `sitl/interactive_swarm_flight.py`, and `sitl/sync_hud.py` similarly hardcode 3-drone instances.
9. **Unused, Obsolete, and Redundant Scripts in `sitl/`**:
   `sitl/` contains multiple legacy implementations:
   - `autonomous_wingman.py` (2-drone leader-follower script).
   - `swarm_3_drones.py` (independent 3-drone leader-follower implementation with hardcoded formation offsets).
   - `sitl_bridge.py` (standalone telemetry proxy).
   These scripts do not utilize the unified `MAVLinkSwarmAdapter` or `CommonCoordinateFrame`.
10. **Committed Compiled Files (`*.pyc`) in Git Index**:
    13 compiled Python bytecode files inside `simulator/__pycache__/` and `swarm_core/__pycache__/` are tracked in git history.

### LOW
11. **Matplotlib Projection Warning**:
    Importing `simulator.visualizer` or running tests generates `UserWarning: Unable to import Axes3D` due to conflicting system and venv matplotlib installations.
12. **Untracked 18 MB Archive in Working Directory**:
    `swarm_drones_fyp_save_20261001.tar.gz` sits untracked in repository root.

### INFORMATIONAL
13. **Coordinate Frame Math is Rigorous**:
    `sitl/common_frame.py` was tested and verified to achieve $< 0.1\text{ mm}$ round-trip precision within a $100\text{ m}$ boundary.
14. **Feedforward Theoretical Math is Validated**:
    The analytical steady-state tracking lag derivation ($e_{\text{steady}} = \frac{k_d + c_d}{k_p} \|\mathbf{v}_{\text{target}}\| = 1.1392\text{ m}$) perfectly matches the empirical baseline ($1.140\text{ m}$) and resolves when feedforward is enabled ($0.018\text{ m}$).

---

## Scientific claim verification

| Claim | Document & Section | Supporting Code | Supporting Test/Experiment | Verification Status | Recommended Wording |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **“Exact same” simulation and SITL controller pipeline** | `ACADEMIC_REVIEW_RESPONSE.md` (Sec 4)<br>`sitl/mavlink_swarm_adapter.py` (Line 4)<br>`CODEX_REVIEW_BRIEF.md` (Sec 3.1) | `sitl/mavlink_swarm_adapter.py`<br>`swarm_core/controllers/` | `tests/test_sitl_adapter.py`<br>`experiments/validate_3drone_scenario_sitl.py` | **Partially Verified**<br>Adapter wraps `HybridController` state machine, but centralized mode bypasses Hungarian matching (uses static indexing), and hybrid accelerations are scaled to position targets via magic multiplier (`* dt * 2.0`). | *"The MAVLink adapter translates high-level swarm_core hybrid guidance commands into 3D local position setpoints for ArduPilot's GUIDED mode position controller."* |
| **Collision-free formation transitions guaranteed by Hungarian algorithm** | `README.md` (Sec 1, Line 15)<br>`CODEX_REVIEW_BRIEF.md` (Sec 2.3) | `swarm_core/formations.py`<br>`swarm_core/controllers/centralized.py` | `tests/test_formations.py`<br>`experiments/run_demo.py` | **Partially Verified**<br>Hungarian matching minimizes total Euclidean displacement $\sum \|p_i - s_{\sigma(i)}\|^2$, which reduces trajectory crossing, but does not provide formal mathematical collision-free guarantees under dynamic lag or disturbance. | *"Hungarian optimal slot assignment minimizes total transit distance and path crossings, complemented by an active onboard Artificial Potential Field safety barrier to maintain physical separation."* |
| **Physical Hardware Validation** | `FYP_RUNNING_LOG.md` (Sec 10.5)<br>`CODEX_REVIEW_BRIEF.md` (Sec 4) | `TASK_E_RESEARCH_REPORT.md` | None (0 physical flight logs exist) | **Unsupported / Future Work**<br>Only pinout mapping, parameter definitions, and research specifications exist. No physical bench or flight telemetry has been collected. | *"Hardware integration architecture, wiring schematics, and parameter profiles have been specified for the Matek H743; physical bench validation is scheduled for Phase 2."* |
| **Scalable 5–10 Drone Swarm Support** | `README.md` (Sec 1, Line 11)<br>`CODEX_REVIEW_BRIEF.md` (Sec 1.1) | `swarm_core/formations.py`<br>`simulator/engine.py` | `experiments/run_demo.py` (6 drones)<br>`experiments/test_feedforward_ablation.py` (5 drones) | **Partially Verified**<br>Numerical simulation engine supports arbitrary $N$ agents. SITL implementation is currently hardcoded and verified for exactly 3 drones. | *"Demonstrated with up to 6 drones in numerical simulation and 3 drones in ArduPilot SITL co-simulation, with an architecture designed to scale to 5–10 drones."* |
| **Digital Twin Simulation** | `FYP_RUNNING_LOG.md` (Sec 6)<br>`gazebo_3d_swarm_digital_twins_sidequest.md` | `simulator/gazebo/` | None | **Unsupported / Inappropriate Terminology**<br>CAD meshes and SDF worlds provide visual airframe representation, but do not incorporate measured motor dynamics, inertia tensors, or aeromechanical twin calibration. | *"3D multi-drone visual simulation in Gazebo Sim (Harmonic); digital twin terminology avoided."* |
| **Sub-Centimeter Accuracy** | Early logs / review prompt | `sitl/common_frame.py`<br>`experiments/run_demo.py` | `tests/test_sitl_adapter.py`<br>`experiments/validate_step_response_sitl.py`<br>`experiments/validate_3drone_scenario_sitl.py` | **Context-Dependent / Misleading**<br>Geometric coordinate conversions achieve $< 0.1\text{ mm}$ round-trip precision. In numerical simulation, settled error is $0.002\text{ m}$. In real SITL flight, trajectory divergence is $72.5\text{ cm}$ RMS. | *"Coordinate conversions achieve $< 0.1\text{ mm}$ precision; numerical simulation tracks within centimeters; SITL multirotor co-simulation exhibits an honest $72.5\text{ cm}$ RMS trajectory divergence reflecting physical multirotor dynamics and EKF3 estimation."* |
| **Empirical Parameter Fitting ($\tau, c_d$)** | `TASK_C_VALIDATION_REPORT.md`<br>`CODEX_REVIEW_BRIEF.md` (Sec 2.1) | `experiments/validate_step_response_sitl.py`<br>`swarm_core/config.py` | `experiments/results/sitl_step_response.csv`<br>`experiments/results/sitl_step_response_fit.png` | **Verified Experimentally, Not Integrated**<br>L-BFGS-B optimization against SITL step response produced $\tau=0.992\text{ s}, c_d=0.637\text{ s}^{-1}$ ($15.36\text{ cm}$ RMSE). However, parameters remain unlinked in `config.py` and are not loaded into `Drone.__init__`. | *"Multirotor dynamic parameters ($\tau=0.992\text{ s}$, $c_d=0.637\text{ s}^{-1}$) were fitted from ArduPilot GUIDED mode step-response telemetry with $15.36\text{ cm}$ residual RMSE, documented in config.py, and pending default parameter propagation."* |
| **100% Test Coverage** | Colloquial review mentions | `tests/` | `pytest tests/` | **Misleading Phrasing**<br>Test pass rate is 100% (15/15 tests passing), but codebase statement coverage is **39%**. | *"100% test pass rate (15/15 unit tests passing), providing 91% coverage of core algorithmic modules and 39% overall repository coverage."* |
| **Autonomous Link Recovery Under Outages** | `TASK_B_EXPERIMENTAL_REPORT.md`<br>`CODEX_REVIEW_BRIEF.md` (Sec 2.3) | `swarm_core/controllers/hybrid.py` | `tests/test_hybrid_features.py`<br>`experiments/test_burst_outage_sweep.py` | **Verified in Simulation**<br>Sliding-window delivery ratio ($w=20, \theta \ge 70\%$) with 2.0s dwell lockout reliably recovers within $0.85\text{ s}$ across 1s, 2s, and 3s outages without chattering. | *"Autonomous link recovery verified in numerical simulation across 1s, 2s, and 3s deterministic outages using a 20-tick sliding window ($\ge 70\%$ delivery ratio) and 2.0s dwell-time lockout."* |

---

## Reproducibility problems

1. **Test Runner Failure**: A new user cloning the repository and running standard `pytest` or `python -m pytest` experiences a hard crash (`SystemExit: 1`) during test collection because `sitl/test_takeoff_diag.py` executes live hardware/SITL commands at import time.
2. **Missing Build/Install Configuration**: No `pyproject.toml` or `setup.py` exists. Users cannot install the package in editable mode (`pip install -e .`). Direct execution of any test or script outside repository root throws `ModuleNotFoundError`.
3. **Incomplete Dependency Listing**: `pip install -r requirements.txt` does not install `pytest`, `pymavlink`, or `pyyaml`. Attempting to run tests or SITL scripts in a fresh environment immediately fails with missing module errors.
4. **Hardcoded User Paths in Shell Scripts**: `sitl/launch_drone*.sh` hardcodes `/home/drone/swarm_drones_fyp/sitl/swarm_params.parm`. If the repo is cloned into any directory other than `/home/drone/swarm_drones_fyp`, SITL crashes on startup with parameter file not found.
5. **Hardcoded Brain Artifact Copying**: Experiments attempt to copy generated plots to `/home/drone/.gemini/antigravity/brain/28220ca6-e68a-487a-8a59-6e79ee58f6f6/`. While wrapped in `try/except`, it pollutes scripts with machine-specific paths.
6. **Committed Bytecode and Large Files**: 13 `.pyc` files and an 18 MB untracked `.tar.gz` archive create git hygiene issues.

---

## Safety concerns

1. **Collision Avoidance under Sensor Noise & Dynamic Lag**:
   - In simulation, plain GPS noise ($\sigma = 1.5\text{ m}$) reduces inter-drone clearance to $1.52\text{ m}$. With physical quadrotors possessing a $0.35\text{ m}$ radius ($0.70\text{ m}$ diameter) plus downwash turbulence, a $1.52\text{ m}$ separation offers a narrow safety buffer.
   - If velocity feedforward is disabled or mismatched, dynamic tracking lag creates transient proximity during formation morphing.
2. **MAVLink Setpoint Interruption & Vehicle Behavior**:
   - In `sitl/mavlink_swarm_adapter.py`, setpoints are sent as `SET_POSITION_TARGET_LOCAL_NED`.
   - If the Python guidance script terminates unexpectedly, ArduPilot will continue toward its last received position setpoint, hover indefinitely, or trigger GCS failsafe after `FS_GCS_TIMEOUT` (typically 5s). Without an active heartbeat watchdog thread, uncommanded drones pose a drift risk.
3. **Lack of Geofencing in SITL & Hardware Setup**:
   - Neither the simulation engine nor the SITL launch scripts enforce software geofencing limits (`FENCE_ENABLE=1`, `FENCE_RADIUS`, `FENCE_ALT_MAX`). A diverging velocity command can cause a runaway vehicle.
4. **Single Common Frame Datum Assumption**:
   - `CommonCoordinateFrame` uses a single geographic datum (defaulting to Canberra SITL `-35.3632621, 149.1652374`). If a drone initializes its EKF origin with an invalid GPS fix or different datum, global-to-local transformations will inject large step offsets, causing immediate violent flight maneuvers.
5. **Actuator Saturation & Acceleration Clamping**:
   - In `swarm_core`, acceleration commands are saturated at $2.5\text{ m/s}^2$. In SITL, ArduPilot clamps acceleration via `PSC_ACC_XY = 250 cm/s²`. If the high-level controller demands aggressive maneuvers beyond autopilot limits, integral windup and tracking divergence occur.
6. **Real Hardware Radio Interference**:
   - As documented in Task E, deploying an ESP32 Wi-Fi bridge on 2.4 GHz co-located with 2.4 GHz ExpressLRS (ELRS) control links creates severe packet desensitization. The physical deployment must mandate 5 GHz Wi-Fi or high-speed telemetry radios.

---

## Recommended order of future work

1. **Fix Test Discovery & Isolation (Immediate Blocker)**:
   Add a `pytest.ini` with `testpaths = tests` and wrap all procedural code in `sitl/test_takeoff_diag.py` and `sitl/test_telemetry.py` inside `if __name__ == "__main__":`.
2. **Add Standard Python Packaging Configuration**:
   Create a modern `pyproject.toml` with `setuptools` build backend and package definitions so `pip install -e .` functions cleanly across all environments.
3. **Complete `requirements.txt`**:
   Add `pytest>=7.4.0`, `pymavlink>=2.4.40`, `pytest-cov>=4.1.0`, and `PyYAML>=6.0`.
4. **Integrate Fitted Dynamics Parameters**:
   Wire `swarm_core/config.py` into `swarm_core/drone.py` and `simulator/engine.py` so that fitted dynamics ($\tau = 0.992\text{ s}$, $c_d = 0.637\text{ s}^{-1}$) are active by default rather than ignored.
5. **Clean Hardcoded Paths**:
   Replace all hardcoded `/home/drone/...` references with relative paths derived from `Path(__file__).resolve()`.
6. **Harmonize Safety and Spacing Parameters**:
   Unify collision threshold, APF repulsion radius, and formation spacing across `config.py`, controllers, and SITL adapter.
7. **Align SITL Adapter with Swarm Core Control Logic**:
   Ensure `MAVLinkSwarmAdapter` utilizes Hungarian optimal slot assignment during formation transitions and documents the acceleration-to-position setpoint scaling.
8. **Automate 5-Drone Multi-Vehicle SITL Launch**:
   Implement a clean launch script (`sitl/launch_5_sitl.sh`) supporting 5 concurrent ArduCopter instances with parameterized ports and home offsets.
9. **Clean Git Index & Repository Hygiene**:
   Untrack committed `.pyc` files (`git rm --cached`), remove the root backup tarball, and archive obsolete SITL scripts (`autonomous_wingman.py`, `swarm_3_drones.py`).
10. **Implement Software Geofencing & Arming Watchdogs**:
    Add parameter configurations for ArduPilot geofencing and automatic fail-to-hover watchdog logic in the MAVLink adapter.

---

## Out of scope

1. **Gazebo Visual Mesh & Texture Customization**:
   Visual modifications (carbon fiber textures, PCB silkscreens, prop styling) provide no academic value for the control framework and should remain frozen.
2. **Complex Digital Twin Aerodynamic Modeling**:
   High-order blade-element rotor dynamics or CFD-based ground-effect modeling are outside FYP scope; the validated first-order lag with rotor drag is fully adequate.
3. **Custom Flight Controller Firmware Modification**:
   Modifying ArduPilot C++ internals or implementing custom DroneCAN setpoint publishers is unnecessary; standard GUIDED mode MAVLink over UART satisfies all project objectives.
4. **Full Hardware Flight on 10 Drones**:
   Current hardware availability is limited to 2 drones. Attempting large-scale physical hardware flight before robust bench validation of 2 drones is out of scope.
