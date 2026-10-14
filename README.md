# Autonomous Swarm Drones: Coordinated Controlled Movement

**Final Year Project (BEE-60, Department of Electrical Engineering)**  
**Supervisor:** Dr. Abdul Ghafoor  
**Target Completion:** December 31, 2026  

---

## 1. Project Overview

This repository provides an autonomous swarm control framework enabling groups of multirotors to perform dynamic formation flying, smooth topology reconfiguration, and decentralized collision avoidance under lossy, delayed wireless communication constraints.

### Current Implementation & Validation Status
* **Numerical simulation**: The research simulator contains centralized, decentralized, and hybrid control regimes. Its fallback behavior has not been integrated into the unified live console.
* **ArduPilot standalone SITL**: A five-drone 2D run completed takeoff, formation changes, movement, patrol, injected failure and rejoin, and confirmed landing on 12 October 2026. The unified console accepts 1–10 drones; that range is not a claim of live validation at every size. See [console guide](docs/CONSOLE_GUIDE.md).
* **Gazebo 3D**: The five-drone world rendered and all SITL links connected, but the EKF readiness gate blocked takeoff. No completed five-drone Gazebo flight has been demonstrated.
* **Physical aircraft**: One quad is assembled and has not been flight-tested. Physical swarm communication and fallback remain unverified.
* **Tests**: The consolidated demo checkout passed 161 tests on 12 October 2026. This count is not coverage or flight evidence. The local consolidation collection contains `VALIDATION_2026-10-12.md` with the run details.
* **Run evidence**: The unified console now writes per-run commands, telemetry, link ages, separation, tracking error, and an outcome summary under `runs/console/`. The [criteria and limitations](docs/RUN_EVIDENCE.md) are specified before each new run; no post-instrumentation run is claimed here.

### Key Framework Capabilities
* **Four Formation Geometries**: Line, V-Formation (Chevron), Circle, and Grid with dynamic reconfiguration.
* **Hungarian Algorithm Slot Assignment**: Solves the Linear Sum Assignment Problem to minimize total swarm displacement during formation transitions, reducing trajectory crossing risk (note: does not provide a formal collision-free guarantee; proximity risk is empirically mitigated by APF separation control).
* **Three Control Regimes**:
  1. **Centralized**: Global coordinator computing optimal slot assignments, PD tracking, and moving centroid feedforward.
  2. **Decentralized**: Distributed Laplacian velocity consensus ($\dot{v}_i = -k_v L v$) and Reynolds/Olfati-Saber flocking relying strictly on 1-hop neighbor broadcasts.
  3. **Hybrid**: Centralized trajectory guidance blended with an always-on decentralized Artificial Potential Field (APF) safety barrier, featuring asymmetric hysteresis, message age validation, and automatic fallback upon coordinator silence.
* **Explicit Configuration Profiles**:
  - `assumed_baseline`: Engineering literature baseline ($\tau=0.18\text{ s}$, $c_d=0.20\text{ s}^{-1}$).
  - `sitl_fitted`: Closed-loop model calibrated against ArduPilot SITL GUIDED-mode step response ($\tau=0.992\text{ s}$, $c_d=0.637\text{ s}^{-1}$, sensor noise $\sigma=1.5\text{ m}$).

* **Wireless Channel Emulation**: Realistic packet drop ($0\%\dots 50\%$), latency queues ($10\dots 400\text{ ms}$), Gilbert-Elliott burst outages, and RF communication range cutoffs.

The `sitl_fitted` values above are historical. The identification work found that an uneven first timestamp interval biased that fit; do not use those values as a validated hardware match. The correction and newer tracking model remain on the separate `hw-matched-identification` research branch.

---

## 2. Directory Structure

```
swarm_drones_fyp/
├── swarm_core/
│   ├── config.py            # Explicit configuration profiles & parameter provenance
│   ├── drone.py             # Multirotor kinematics, first-order lag, rotor drag
│   ├── graph.py             # Graph Laplacian, adjacency, algebraic connectivity
│   ├── network.py           # Wireless channel emulator (loss, delay, burst outage)
│   ├── formations.py        # Line, V, Circle, Grid geometries & Hungarian assignment
│   ├── metrics.py           # Tracking error, convergence time, safety distance
│   └── controllers/
│       ├── centralized.py   # Global Hungarian + PD tracking + APF safety
│       ├── decentralized.py # Laplacian velocity consensus + Reynolds flocking
│       └── hybrid.py        # Asymmetric hysteresis & continuous alpha(t) blending
├── simulator/
│   ├── engine.py            # Physics integration, wireless channel, and control step
│   └── visualizer.py        # Telemetry renderer and static figure generation
├── sitl/
│   ├── common_frame.py      # WGS84 GPS to local/global NED datum frame conversion
│   ├── mavlink_swarm_adapter.py # MAVLink adapter translating setpoints to ArduPilot
│   └── launch_drone*.sh     # Headless SITL drone launcher scripts
├── experiments/
│   ├── run_demo.py          # 4-formation transition mission demo with JSON export
│   ├── test_network_sweep.py# Monte Carlo packet loss sweep
│   └── results/             # Generated figures, plots, and JSON metadata
├── tests/
│   ├── test_config_and_safety.py # Profile, provenance, and safety hierarchy tests
│   ├── test_formations.py        # Geometry & Hungarian assignment tests
│   ├── test_graph.py             # Graph Laplacian & connectivity tests
│   ├── test_hybrid_features.py   # Hysteresis, blending, and burst loss tests
│   ├── test_simulation.py        # Multi-regime simulation tests
│   └── test_sitl_adapter.py      # Frame round-trip & mocked MAVLink tests
├── docs/                    # Audit, parity report, reviews, project log and experiment reports
│   ├── BASELINE_AUDIT.md, SIMULATION_SITL_PARITY.md
│   ├── logs/FYP_RUNNING_LOG.md
│   └── reports/             # TASK_B / TASK_C / TASK_E and PHASE2_CONFIG_VALIDATION reports
├── pyproject.toml           # Standard PEP 517/621 packaging configuration
├── requirements.txt         # Pinned runtime dependencies
└── pytest.ini               # Test discovery configuration
```

---

## 3. Quick Start & Installation

### Standard Installation
Install the package and optional development dependencies in editable mode:
```bash
# Clone the repository
git clone https://github.com/radahnnn/autonomous-swarm-drones-fyp.git
cd autonomous-swarm-drones-fyp

# Install in editable mode with development dependencies
pip install -e ".[dev]"

# Optional: install SITL dependencies (pymavlink) if running live SITL simulations
pip install -e ".[sitl]"
```

### Running Unit Tests
No `PYTHONPATH` workaround is needed. Run pytest directly from the repository root:
```bash
python3 -m pytest -q
```

### Running the 4-Formation Mission Demo
Executes a 6-drone mission transitioning through Line $\to$ V-Shape $\to$ Circle $\to$ Grid with reproducible fixed seed and profile recording:
```bash
python3 experiments/run_demo.py --profile assumed_baseline --seed 123
```
Snapshots, telemetry plots, and machine-readable JSON metadata will be written to `experiments/results/demo_summary.json`.
