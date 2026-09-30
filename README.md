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

## 2. Directory Structure

```
swarm_drones_fyp/
├── swarm_core/
│   ├── drone.py             # 2D/3D kinematic state & acceleration limits
│   ├── graph.py             # Graph Laplacian, adjacency, algebraic connectivity
│   ├── network.py           # Wireless channel emulator (loss, delay, range)
│   ├── formations.py        # Line, V, Circle, Grid geometries & Hungarian assignment
│   ├── metrics.py           # Tracking error, convergence time, safety distance
│   └── controllers/
│       ├── centralized.py   # Global Hungarian + PD tracking + APF safety
│       ├── decentralized.py # Laplacian velocity consensus + Reynolds flocking
│       └── hybrid.py        # Heartbeat monitor & autonomous fallback state machine
├── simulator/
│   ├── engine.py            # Main simulation loop (physics + network + control)
│   └── visualizer.py        # Matplotlib 2D animation & HUD telemetry renderer
├── experiments/
│   ├── run_demo.py          # 4-formation transition mission demo
│   ├── test_network_sweep.py# Monte Carlo packet loss sweep (0% to 50%)
│   └── results/             # Generated publication figures and logs
├── tests/
│   ├── test_graph.py        # Graph theory & Fiedler value tests
│   ├── test_formations.py   # Geometry & assignment tests
│   └── test_simulation.py   # Multi-regime simulation tests
└── requirements.txt
```

---

## 3. Quick Start

### Prerequisites
Ubuntu 22.04 / 24.04 with standard Python 3.10+:
```bash
pip install -r requirements.txt
```

### Running Unit Tests
```bash
PYTHONPATH=. python3 tests/test_graph.py
PYTHONPATH=. python3 tests/test_formations.py
PYTHONPATH=. python3 tests/test_simulation.py
```

### Running the 4-Formation Mission Demo
Executes a 6-drone mission transitioning through Line $\to$ V-Shape $\to$ Circle $\to$ Grid:
```bash
PYTHONPATH=. python3 experiments/run_demo.py
```
Snapshots and telemetry plots will be saved to `experiments/results/`.

### Running Network Packet Loss Sweeps
Compares Centralized vs Decentralized vs Hybrid robustness across $0\%\dots 50\%$ packet drop:
```bash
PYTHONPATH=. python3 experiments/test_network_sweep.py
```
Outputs comparison figures to `experiments/results/network_loss_comparison.png`.
