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
* `tests/`:
  - `test_graph.py`, `test_formations.py`, `test_simulation.py`.
* `FYP_RUNNING_LOG.md`: Comprehensive history from project inception, all 9 problems solved, and current status.

---

## 3. How to Run & Verify
```bash
# Run test suite
PYTHONPATH=. python3 tests/test_graph.py
PYTHONPATH=. python3 tests/test_formations.py
PYTHONPATH=. python3 tests/test_simulation.py

# Run 4-formation demo
PYTHONPATH=. python3 experiments/run_demo.py

# Run network sweep
PYTHONPATH=. python3 experiments/test_network_sweep.py
```

---

## 4. Key Questions for Second-Opinion Review
1. Are there mathematical or numerical stability edge cases in the Laplacian consensus, APF repulsion, or Hungarian assignment?
2. Is the Hybrid switching logic robust against intermittent packet drop (chattering/hysteresis)?
3. What considerations should be kept in mind when mapping these velocity/acceleration commands to ArduPilot SITL via `pymavlink` (`SET_POSITION_TARGET_LOCAL_NED`)?
