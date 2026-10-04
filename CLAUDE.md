# Swarm Drones FYP — Claude Code Project Context & Review Guide

**Project:** Autonomous Swarm Drones Coordinated Movement (BEE-60, Dept. of EE)  
**Supervisor:** Dr. Abdul Ghafoor  
**Target Completion:** December 31, 2026  
**Primary Status:** Phase 1 Complete (Python Engine, 4 Formations, 3 Control Modes, Network Sweeps)  

---

## 1. Project Overview & Scope
Develop an autonomous swarm control framework for 5–10 multirotors capable of:
* **4 Formations**: Line, V-Formation, Circle, Grid with dynamic switching.
* **Hungarian Algorithm Slot Assignment**: Solves Linear Sum Assignment to minimize displacement and avoid trajectory crossings.
* **Three Control Modes**:
  1. Centralized (global planner + APF safety barrier).
  2. Decentralized (Laplacian velocity consensus $\dot{v} = -Lv$ + Reynolds flocking + relative cohesion).
  3. Hybrid (centralized trajectory tracking with onboard APF safety + automatic degradation to local flocking upon coordinator heartbeat timeout $t > 0.5\text{s}$).
* **Wireless Channel Emulation**: Packet loss ($0\%\dots 50\%$), latency queue ($10\dots 400\text{ ms}$), and range limits.

---

## 2. Directory Structure & Key Files
* `swarm_core/`:
  - `drone.py`: 2D/3D kinematic state, velocity/accel saturation, trajectory logging.
  - `graph.py`: Adjacency, Laplacian ($L = D - A$), Fiedler algebraic connectivity ($\lambda_2$).
  - `network.py`: Lossy delayed wireless channel.
  - `formations.py`: Line, V, Circle, Grid geometry generators + Hungarian assignment (`scipy.optimize.linear_sum_assignment`).
  - `controllers/`:
    - `centralized.py`: Global Hungarian matching + PD tracking + APF repulsion.
    - `decentralized.py`: Local consensus on 1-hop neighbor broadcasts + Reynolds flocking.
    - `hybrid.py`: Heartbeat monitoring and autonomous fallback state machine.
  - `metrics.py`: RMS tracking error, min inter-drone distance, convergence time.
* `simulator/`:
  - `engine.py`: Integrated physics, network, and control loop.
  - `visualizer.py`: 2D HUD telemetry renderer and animation exporter.
* `experiments/`:
  - `run_demo.py`: 6-drone 4-formation transition scenario (Line -> V -> Circle -> Grid).
  - `test_network_sweep.py`: Monte Carlo parameter sweep across $0\%\dots 50\%$ packet loss.
  - `results/`: Output plots (`demo_metrics.png`, `network_loss_comparison.png`, formation snapshots).
* `sitl/`:
  - `common_frame.py`: WGS84 flat-earth tangent plane coordinate frame anchoring all SITL drones to shared datum.
  - `mavlink_swarm_adapter.py`: Unified adapter directly executing swarm_core controllers on live SITL MAVLink.
  - `swarm_3_drones.py`: Multi-drone coordinated takeoff and V-formation flight.
  - `interactive_swarm_flight.py`: Interactive 1-key flight maneuver and patrol tool.
  - `sync_hud.py`: Real-time 5 Hz ASCII synchronization radar HUD.
* `tests/`:
  - `test_graph.py`, `test_formations.py`, `test_simulation.py`, `test_hybrid_features.py`.
* `docs/logs/FYP_RUNNING_LOG.md`: Comprehensive running history, architectural details, and defense notes.
* `docs/ACADEMIC_REVIEW_RESPONSE.md`: Point-by-point technical responses to all 8 concerns from the 01 Oct 2026 review.

---

## 3. How to Run & Verify
```bash
# Run test suite
PYTHONPATH=. pytest tests/

# Run dynamic stress-tested network sweep (moving centroid + mid-flight morph)
PYTHONPATH=. python3 experiments/test_network_sweep.py

# Run unified MAVLink Swarm Adapter against SITL
PYTHONPATH=. python3 sitl/mavlink_swarm_adapter.py
```

---

## 4. Academic Review Hardening Status (01 Oct 2026)
All 8 reviewer concerns have been formally resolved:
1. **Hybrid Recovery**: Upgraded to 20-tick sliding window delivery ratio ($\ge 70\%$) replacing brittle consecutive counter.
2. **Adaptive Stale-Age**: Dynamically scales with nominal latency ($\tau_{\text{stale}} = \max(3\tau_{\text{lat}}, 0.15\text{s})$).
3. **Multirotor Dynamics**: Added first-order attitude lag ($\tau=0.18\text{s}$), aerodynamic drag ($c_d=0.20$), and sensor noise ($\sigma=0.04\text{m}$).
4. **Unified Codebase**: `mavlink_swarm_adapter.py` bridges the exact `swarm_core` algorithms to SITL.
5. **Honest Scope**: Core engine: $90\%$, SITL fleet integration: $35\%$, Hardware: $10\%$.
6. **Gold-Plating Ceased**: Retracted "digital twin" misnomer, standardized geometry ($7.00\text{m}$ wingspan).
7. **Coordinate Frames**: `common_frame.py` anchors all SITL drones to common datum.
8. **Defense Preparation**: Full mathematical derivations and examiner Q&A documented in `docs/logs/FYP_RUNNING_LOG.md`.
