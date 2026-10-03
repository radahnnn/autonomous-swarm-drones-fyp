# Autonomous Swarm Drones: Coordinated Controlled Movement

**Final Year Project (BEE-60, Department of Electrical Engineering)**  
**Supervisor:** Dr. Abdul Ghafoor  
**Target Completion:** December 31, 2026  

---

## 1. Project Overview

This repository provides an autonomous swarm control framework enabling groups of multirotors to perform dynamic formation flying, smooth topology reconfiguration, and decentralized collision avoidance under lossy, delayed wireless communication constraints.

### Current Implementation & Validation Status
* **Numerical Simulation Engine**: Validated for arbitrary swarm sizes ($N \ge 3$, tested up to 6 drones) in 2D/3D continuous time.
* **ArduPilot SITL Software-in-the-Loop**: High-fidelity integration verified for 3 drones (`sitl/mavlink_swarm_adapter.py`). Fleet scaling to 5+ SITL instances is scheduled for Phase 2.
* **Trajectory Discrepancy**: A 72.5 cm trajectory discrepancy was measured between the lightweight simulation model and ArduPilot SITL GUIDED mode tracking (simulation-to-SITL discrepancy; physical hardware flight validation remains future work).
* **Test Suite**: 23/23 unit tests passing via `pytest` (verifying imports, profile propagation, graph Laplacians, Hungarian assignment, hybrid state machines, and MAVLink adapter mocks). Passing test count reflects regression coverage, not 100% statement or branch coverage.

### Key Framework Capabilities
* **Four Formation Geometries**: Line, V-Formation (Chevron), Circle, and Grid with dynamic reconfiguration.
* **Hungarian Algorithm Slot Assignment**: Solves the Linear Sum Assignment Problem to minimize total swarm displacement during formation transitions, reducing trajectory crossing risk (note: does not provide a formal collision-free guarantee; collision safety is guaranteed by active APF repulsion barriers).
* **Three Control Regimes**:
  1. **Centralized**: Global coordinator computing optimal slot assignments, PD tracking, and moving centroid feedforward.
  2. **Decentralized**: Distributed Laplacian velocity consensus ($\dot{v}_i = -k_v L v$) and Reynolds/Olfati-Saber flocking relying strictly on 1-hop neighbor broadcasts.
  3. **Hybrid**: Centralized trajectory guidance blended with an always-on decentralized Artificial Potential Field (APF) safety barrier, featuring asymmetric hysteresis, message age validation, and automatic fallback upon coordinator silence.
* **Explicit Configuration Profiles**:
  - `assumed_baseline`: Engineering literature baseline ($\tau=0.18\text{ s}$, $c_d=0.20\text{ s}^{-1}$).
  - `sitl_fitted`: Closed-loop model calibrated against ArduPilot SITL GUIDED-mode step response ($\tau=0.992\text{ s}$, $c_d=0.637\text{ s}^{-1}$, sensor noise $\sigma=1.5\text{ m}$).
* **Wireless Channel Emulation**: Realistic packet drop ($0\%\dots 50\%$), latency queues ($10\dots 400\text{ ms}$), Gilbert-Elliott burst outages, and RF communication range cutoffs.

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
