# Autonomous Swarm Drones: Coordinated Controlled Movement

**Final Year Project (BEE-60, Department of Electrical Engineering)**  
**Supervisor:** Dr. Abdul Ghafoor  
**Target Completion:** December 31, 2026  

---

## 1. Project Overview

This repository provides an autonomous swarm control framework enabling groups of 5–10 multirotors to perform dynamic formation flying, smooth topology reconfiguration, and decentralized collision avoidance under lossy, delayed wireless communication constraints.

### Key Capabilities
* **Four Formation Geometries**: Line, V-Formation (Chevron), Circle, and Grid with dynamic reconfiguration.
* **Hungarian Algorithm Slot Assignment**: Solves the Linear Sum Assignment Problem to minimize total swarm displacement during formation transitions, preventing trajectory crossing and collisions.
* **Three Control Regimes**:
  1. **Centralized**: Global mission coordinator assigning optimal trajectory slots and monitoring swarm centroid.
  2. **Decentralized**: Distributed consensus ($\dot{v}_i = -L v$) and Reynolds/Olfati-Saber flocking relying strictly on 1-hop neighbor broadcasts.
  3. **Hybrid (Proposed Framework)**: Hierarchical architecture where centralized guidance directs formation geometry while onboard decentralized safety barriers prevent collisions, with automatic fallback to local flocking if coordinator heartbeats drop.
* **Wireless Channel Emulation**: Models realistic RF propagation constraints including packet loss ($0\%\dots 50\%$), transmission latency queues ($10\dots 400\text{ ms}$), and finite communication radii.
* **Dual-Tier Validation**:
  - **Tier 1**: High-speed numerical engine for Monte Carlo parameter sweeps and thesis metric generation.
  - **Tier 2**: Headless ArduPilot SITL multi-vehicle validation via MAVLink (`pymavlink`).

---

## 2. Repository Layout

```
.
├── swarm_core/              # Pure-Python algorithms (no simulator / MAVLink dependency)
│   ├── drone.py             # Second-order kinematics, lag, drag, sensor noise
│   ├── graph.py             # Adjacency, Laplacian, algebraic connectivity
│   ├── network.py           # Wireless channel emulator (loss, delay, range, bursts)
│   ├── formations.py        # Line, V, Circle, Grid + Hungarian slot assignment
│   ├── metrics.py           # Tracking error, separation, convergence, chattering
│   ├── config.py            # Parameter provenance (fitted vs assumed) -- not yet wired in
│   └── controllers/         # centralized.py, decentralized.py, hybrid.py
├── simulator/               # Tier 1: numerical engine, Matplotlib visualizer, Gazebo models/world
├── sitl/                    # Tier 2: ArduPilot SITL / MAVLink adapter, launch scripts, diagnostics
├── experiments/             # Reproducible sweeps and SITL validation; outputs in experiments/results/
├── tests/                   # Unit tests (pytest)
└── docs/                    # Reports, logs, audit and review material
    ├── BASELINE_AUDIT.md
    ├── reports/             # TASK_B / TASK_C / TASK_E reports
    └── logs/                # FYP_RUNNING_LOG.md
```

---

## 3. Quick Start

Requires Python 3.10+.

```bash
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scriptsctivate
pip install -e ".[dev]"          # core + test tools
pip install -e ".[sitl,dev]"     # additionally pymavlink/pandas for SITL scripts
```

### Run the tests
```bash
pytest
```
`pytest` is configured to collect only `tests/`. None of the unit tests need ArduPilot.

### Run the experiments
Executed from the repo root; figures are written to `experiments/results/`.
```bash
python experiments/run_demo.py                    # 6-drone Line -> V -> Circle -> Grid mission
python experiments/test_network_sweep.py          # packet-loss sweep, 3 controllers
python experiments/test_burst_outage_sweep.py     # Gilbert-Elliott burst loss + outages
python experiments/test_gps_noise_sweep.py        # GPS noise sweep
python experiments/test_feedforward_ablation.py   # feedforward ablation
```
The two `experiments/validate_*_sitl.py` scripts need ArduPilot SITL:
```bash
export ARDUCOPTER_BIN=$HOME/ardupilot/build/sitl/bin/arducopter   # default shown
python experiments/validate_step_response_sitl.py
python experiments/validate_3drone_scenario_sitl.py
```

### Gazebo / multi-drone SITL (optional)
Launch scripts in `sitl/` assume ArduPilot in `~/ardupilot`, a venv at `~/venv-ardupilot`, and the
`ardupilot_gazebo` plugin built in `~/ardupilot_gazebo` (override with `ARDUPILOT_GAZEBO_DIR`).
`sitl/stop_all.sh` terminates all related processes.

---

## 4. Documentation
* [`docs/BASELINE_AUDIT.md`](docs/BASELINE_AUDIT.md) -- audit of tests, experiments and claims
* [`docs/logs/FYP_RUNNING_LOG.md`](docs/logs/FYP_RUNNING_LOG.md) -- chronological project log
* [`docs/reports/`](docs/reports) -- experimental, validation and hardware research reports
