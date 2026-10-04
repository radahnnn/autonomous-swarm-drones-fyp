# Swarm Drones FYP — Claude Code Project Context & Review Guide

**Project:** Autonomous Swarm Drones Coordinated Movement (BEE-60, Dept. of EE)  
**Supervisor:** Dr. Abdul Ghafoor  
**Target Completion:** December 31, 2026  
**Current Phase:** Phase 1 Complete (Foundations, Packaging, and Reproducibility Verified)  

---

## 1. Project Overview & Scope
Autonomous swarm control framework enabling multirotors to achieve coordinated movement:
* **4 Formations**: Line, V-Formation, Circle, Grid with dynamic topology reconfiguration.
* **Hungarian Algorithm Slot Assignment**: Solves Linear Sum Assignment to minimize total swarm displacement and reduce crossing risk (not a formal collision-free guarantee; proximity risk is empirically mitigated by APF separation control).
* **Three Control Modes**:
  1. Centralized (global planner + moving centroid feedforward + APF safety).
  2. Decentralized (Laplacian velocity consensus $\dot{v} = -k_v Lv$ + Reynolds flocking + relative cohesion).
  3. Hybrid (centralized trajectory tracking with always-on onboard APF safety + automatic fallback to local flocking upon coordinator heartbeat timeout $t > 0.5\text{s}$).
* **Explicit Configuration Profiles**:
  - `assumed_baseline`: Engineering literature baseline ($\tau=0.18\text{s}$, $c_d=0.20\text{ 1/s}$).
  - `sitl_fitted`: SITL-calibrated closed-loop model parameters ($\tau=0.992\text{s}$, $c_d=0.637\text{ 1/s}$, GPS noise $\sigma=1.5\text{m}$).
* **Wireless Channel Emulation**: Packet loss ($0\%\dots 50\%$), latency queue ($10\dots 400\text{ ms}$), Gilbert-Elliott burst outages, and RF range limits.
* **Status & Honest Scope**:
  - Numerical simulation engine: supports arbitrary $N$ (tested up to 6 drones in demo scenarios).
  - SITL integration: 3 drones verified in ArduPilot GUIDED mode (`sitl/mavlink_swarm_adapter.py`). 5-drone scaling scheduled for Phase 2.
  - Sim-to-SITL trajectory discrepancy: 72.5 cm measured between kinematic simulator and ArduPilot SITL GUIDED tracking (simulation-to-SITL; physical hardware flight validation remains future work).
  - Test coverage: 23 unit tests passing in pytest (distinguishes passing test count from full statement/branch coverage).

---

## 2. Directory Structure & Key Files
* `swarm_core/`:
  - `config.py`: Explicit configuration profiles (`assumed_baseline`, `sitl_fitted`) and parameter provenance.
  - `drone.py`: Kinematic state, first-order attitude lag, rotor drag, sensor noise.
  - `graph.py`: Adjacency, Laplacian ($L = D - A$), Fiedler algebraic connectivity ($\lambda_2$).
  - `network.py`: Wireless channel emulator (loss, delay, Gilbert-Elliott burst outages).
  - `formations.py`: Line, V, Circle, Grid geometry generators + Hungarian assignment (`scipy.optimize.linear_sum_assignment`).
  - `controllers/`:
    - `centralized.py`: Global Hungarian matching + PD tracking + APF repulsion.
    - `decentralized.py`: Local consensus on 1-hop neighbor broadcasts + Reynolds flocking.
    - `hybrid.py`: Heartbeat monitoring, asymmetric hysteresis, and smooth $\alpha(t)$ blending.
  - `metrics.py`: RMS tracking error, min inter-drone distance, convergence time.
* `simulator/`:
  - `engine.py`: Integrated physics, network, and profile-driven control loop.
  - `visualizer.py`: 2D HUD telemetry renderer and static plot generator.
* `experiments/`:
  - `run_demo.py`: 6-drone 4-formation transition scenario with JSON metadata output.
  - `test_network_sweep.py`: Monte Carlo parameter sweep across $0\%\dots 50\%$ packet loss.
  - `results/`: Output plots and machine-readable `demo_summary.json`.
* `sitl/`:
  - `common_frame.py`: WGS84 flat-earth tangent plane coordinate frame anchoring all SITL drones to shared datum.
  - `mavlink_swarm_adapter.py`: MAVLink adapter translating `swarm_core` control outputs into ArduPilot position setpoints.
  - `swarm_3_drones.py`: Multi-drone coordinated takeoff and V-formation flight.
  - `interactive_swarm_flight.py`: Interactive flight maneuver and patrol tool.
* `tests/`:
  - `test_config_and_safety.py`, `test_graph.py`, `test_formations.py`, `test_simulation.py`, `test_hybrid_features.py`, `test_sitl_adapter.py`.
* `pyproject.toml` & `requirements.txt`: PEP 517/621 package specification and locked dependencies.
* `docs/BASELINE_AUDIT.md`: Phase 0 comprehensive baseline audit report.

---

## 3. How to Run & Verify
```bash
# Install in editable mode
pip install -e ".[dev]"

# Run full test suite without PYTHONPATH
python3 -m pytest -q

# Run reproducible 4-formation demo with fixed seed and JSON export
python3 experiments/run_demo.py --profile assumed_baseline --seed 123
```

---

## 4. Key Engineering & Scientific Principles
1. **Safety Spatial Hierarchy**:
   $2 \times r_{\text{drone}} = 0.70\text{m} \le d_{\text{collision}} < d_{\text{APF}} (1.20\dots 1.50\text{m}) < d_{\text{spacing}} (2.50\text{m}) \ll R_{\text{comm}} (12.0\text{m})$.
2. **Hybrid Hysteresis & Blending**:
   - Degrades to fallback after 0.5s of heartbeat silence.
   - Recovers only after $\ge 70\%$ packet delivery ratio in sliding 20-packet window AND a 2.0s dwell time lockout.
   - Smooth control blending $\alpha(t)$ across 0.8s prevents acceleration step jumps while local APF safety barrier remains 100% active.
3. **Simulation/SITL Parity**:
   The SITL adapter reuses the core controller components, but commands are translated through ArduPilot's onboard position controller.
