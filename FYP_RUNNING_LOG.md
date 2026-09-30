# Swarm Drones FYP: Comprehensive Running Activity & Setup Log

**Project:** Swarm Drones Coordinated Controlled Movement (BEE-60, Department of Electrical Engineering)  
**Supervisor:** Dr. Abdul Ghafoor  
**Target Completion:** December 31, 2026 (Accelerated Timeline)  
**Workspace:** `/home/drone/.gemini/antigravity/scratch/swarm_drones_fyp` (symlink: `~/swarm_drones_fyp`)  

---

## 1. Project Background and Objective

The goal of this Final Year Project (FYP) is to develop the complete autonomous control software for a swarm of 5–10 multirotor UAVs capable of:
1. **Dynamic Formations**: Flying in Line, V-Formation (Chevron), Circle, and Grid geometries.
2. **Smooth Reconfiguration**: Switching dynamically between formations without trajectory crossings or mid-air collisions.
3. **Three Control Modes**:
   - **Centralized**: Global coordinator plans trajectory and assigns formation slots.
   - **Decentralized**: Local consensus ($\dot{\mathbf{v}}_i = -L \mathbf{v}$) and Reynolds/Olfati-Saber flocking relying strictly on 1-hop neighbor wireless broadcasts.
   - **Hybrid**: Hierarchical framework where centralized commands guide high-level geometry while onboard decentralized safety barriers prevent collisions, with automatic fallback to local flocking if communication drops.
4. **Resilience to Network Constraints**: Maintaining cohesion and collision safety under wireless packet loss ($0\%\dots 50\%$), transmission latency ($10\dots 400\text{ ms}$), and finite communication range.

---

## 2. Chronological History: What We've Done From the Beginning

### Phase A: Initial Environment Exploration & Setup (29–30 Sep 2026)
* **Host System Inspected**:
  - Ubuntu 24.04.5 LTS (Noble), x86_64.
  - AMD Ryzen 5 3500X (6 cores), 15 GiB RAM, NVIDIA GeForce GTX 1660 Super.
  - Installed base: ROS 2 Jazzy Desktop.
* **Physical Hardware Inspected (Photos & Documentation)**:
  - 10 quadcopters owned by the department; 2 in possession of the team.
  - Flight Controller: Matek H743-SLIM V3 (confirmed from board photos and PDF).
  - Ports available per wiring sheet: UART4, UART6, UART7, UART8, I2C, CAN.
  - Hardware team plans to interface an ESP32 as a telemetry link.
  - Firmware on board: Unknown (team mentioned Betaflight and Mission Planner; needs bench check).
* **The PX4 Exploration Track**:
  - Cloned `~/PX4-Autopilot` and built PX4 SITL with Gazebo Harmonic (`make px4_sitl gz_x500`).
  - Installed Micro XRCE-DDS Agent v2.4.3 in `/usr/local`.
  - Echoed PX4 ROS 2 topics (`/fmu/out/vehicle_local_position_v1`).
* **The ArduPilot Exploration Track**:
  - Cloned ArduPilot in `~/ardu_ws/src/ardupilot` and installed prerequisites.
  - Built `Micro-XRCE-DDS-Gen` v4.7.1.
  - Built `ardupilot_msgs`, `micro_ros_agent`, and `ardupilot_sitl` with `colcon`.
  - Launched ArduPilot SITL with MAVProxy and bridged to ROS 2 topics.
  - Verified `/ap/status` telemetry on ROS 2 from a single disarmed simulated quad.

---

### Phase B: The Strategic Diagnosis & Pivot (30 Sep 2026 Afternoon)
* **The Problem Identified**:
  - Almost all time and effort was spent troubleshooting simulation plumbing (Java versions, Fast-DDS version warnings, Gazebo SDF frames, micro-ros-agent bridges).
  - Not a single line of swarm algorithms (graph theory, consensus, collision avoidance, formation planning) had been written.
  - Multi-vehicle Gazebo + ROS 2 DDS simulations for 5–10 drones would overwhelm the 6-core Ryzen 3500X CPU and GPU, causing severe simulation lag and endless debugging.
  - The project target was accelerated to **December 31, 2026** (3 months remaining).
* **The 3-Tier Architecture Pivot Decided**:
  1. **Tier 1 (Core Python Swarm Engine)**: Fast, lightweight 2D/3D numerical simulation for rapid algorithm design and automated Monte Carlo parameter sweeps. Generates all thesis evaluation plots in seconds.
  2. **Tier 2 (Headless ArduPilot SITL via MAVLink)**: Use ArduPilot's native multi-vehicle SITL (`sim_vehicle.py -v ArduCopter -I 0`, `-I 1`) connected via `pymavlink`. Cuts out ROS 2 DDS complexity, saves CPU/RAM, and runs 5 drones reliably.
  3. **Tier 3 (Visuals & Hardware)**: Gazebo and real ESP32 drones treated as visual/demo layers, never blockers.

---

### Phase C: Implementation of the Swarm Engine (30 Sep 2026 Late Afternoon)
* **Initialized Project Workspace**:
  - Location: `/home/drone/.gemini/antigravity/scratch/swarm_drones_fyp` (symlink: `~/swarm_drones_fyp`).
  - Initialized Git repository and committed initial milestone.
* **Core Modules Developed (`swarm_core/`)**:
  - `drone.py`: Second-order kinematic state $(\mathbf{p}_i, \mathbf{v}_i, \mathbf{a}_i)$, velocity/acceleration limits, and trajectory history.
  - `graph.py`: Proximity graph topology, adjacency matrix $A$, degree matrix $D$, graph Laplacian $L = D - A$, and algebraic connectivity (Fiedler eigenvalue $\lambda_2(L)$).
  - `network.py`: Lossy wireless medium modeling stochastic packet drop ($0\%\dots 50\%$), latency queuing ($10\dots 400\text{ ms}$), and communication radius bounds.
  - `formations.py`: Geometric slot generators for Line, V-Shape, Circle, and Grid. Integrated Hungarian algorithm (`scipy.optimize.linear_sum_assignment`) to find optimal slot assignments and eliminate trajectory crossings.
  - `controllers/centralized.py`: Global mission coordinator with Hungarian slot assignment, PD tracking, and global APF safety barrier.
  - `controllers/decentralized.py`: Distributed Laplacian velocity consensus ($\dot{\mathbf{v}}_i = -k \sum (\mathbf{v}_i - \mathbf{v}_j)$), Reynolds separation (APF repulsion), and relative geometric cohesion.
  - `controllers/hybrid.py`: Finite state machine tracking central coordinator heartbeat. Seamlessly degrades to decentralized flocking if link times out ($t > 0.5\text{ s}$), maintaining onboard APF safety in both modes.
  - `metrics.py`: Real-time logging of RMS formation tracking error, minimum inter-drone distance, velocity variance, and packet delivery diagnostics.
* **Simulation Loop & Visualizer (`simulator/`)**:
  - `engine.py`: Integrated physics integration (semi-implicit Euler), wireless packet delivery, and controller execution.
  - `visualizer.py`: 2D Matplotlib HUD telemetry renderer and animated GIF/MP4 export pipeline.

---

### Phase D: Unit Testing & Experiments (30 Sep 2026 Evening)
* **Unit Tests (`tests/`)**:
  - `test_graph.py`: Verified Laplacian row-sum invariance, connected Fiedler values ($\lambda_2 > 0$), and disconnected graph detection. **(PASSED)**
  - `test_formations.py`: Verified centering, geometry coordinates, and Hungarian assignment swapping. **(PASSED)**
  - `test_simulation.py`: Verified multi-regime simulation execution under Centralized, Decentralized, and Hybrid modes. **(PASSED)**
* **Mission Demo (`experiments/run_demo.py`)**:
  - 6 drones dynamically transitioning through all 4 formations: **Line $\to$ V-Shape $\to$ Circle $\to$ Grid**.
  - **Results**:
    - Minimum distance between any two drones: **$1.761\text{ m}$** (Safety threshold $0.70\text{ m}$ $\to$ **0 collisions**).
    - Steady-state formation tracking error: **$0.002\text{ m}$**.
    - Generated snapshots: `formation_line.png`, `formation_v_shape.png`, `formation_circle.png`, `formation_grid.png`, and `demo_metrics.png`.
* **Network Packet Loss Sweep (`experiments/test_network_sweep.py`)**:
  - Monte Carlo parameter sweep across $0\%\dots 50\%$ packet drop comparing Centralized, Decentralized, and Hybrid modes.
  - **Results**:
    - **Centralized**: Fast convergence ($2.30\text{ s}$), but completely dependent on ground station packets.
    - **Decentralized**: Fully autonomous, error increased smoothly from $0.054\text{ m}$ to $0.118\text{ m}$ under $50\%$ loss with zero collisions.
    - **Hybrid (Proposed)**: Best of both worlds — achieved $2.30\dots 2.51\text{ s}$ convergence and sub-centimeter error ($0.006\text{ m}$) even at $50\%$ packet loss.
    - Generated comparison plot: `network_loss_comparison.png`.

---

## 3. Comprehensive Problems Encountered and Solutions Applied

| # | Problem Encountered | Root Cause | Solution Applied |
|---|---|---|---|
| 1 | **Micro-XRCE-DDS-Gen build failure** | Ubuntu 24.04 had Java 21 active; Gradle 7.6 wrapper requires Java 17 (`Unsupported class file major version 65`). | Installed and selected Java 17 for the build. |
| 2 | **MAVProxy command not found** | MAVProxy was installed in Python virtual environment `~/venv-ardupilot`, not in default shell `PATH`. | Activated virtual environment via `source ~/venv-ardupilot/bin/activate`. |
| 3 | **"Message type invalid" on ROS 2 echo** | Sourced environment was stale or sourced in conflicting order. | Re-sourced `/opt/ros/jazzy/setup.bash` followed by `~/ardu_ws/install/setup.bash`. |
| 4 | **Matplotlib Axes3D import warning** | Deprecated 3D axis import in MAVProxy console. | Identified as non-fatal; affects only the MAVProxy map's 3D view. Ignored safely. |
| 5 | **Fast-DDS / Fast-CDR version conflict warning** | `/usr/local` had XRCE-DDS binaries that clashed with ROS 2 Jazzy system packages. | Monitored; did not block single-vehicle DDS telemetry. |
| 6 | **Tooling rabbit hole / Zero swarm code** | Focusing on ROS 2 multi-instance Gazebo bridges consumed weeks without progress on core FYP requirements. | Pivoted to 3-tier architecture: Python fast simulator first, headless ArduPilot MAVLink second, Gazebo as visual layer only. |
| 7 | **H543 vs H743 Board Target confusion** | Early teammate notes cited "H543", but hardware photos and pinout sheets showed Matek H743-SLIM V3. | Standardized on Matek H743-SLIM V3 per board markings. |
| 8 | **`ModuleNotFoundError: No module named 'swarm_core'` in tests** | Running tests without package installation or root path in `PYTHONPATH`. | Executed test scripts with `PYTHONPATH=.`. |
| 9 | **Test collision assertion failure at step 0** | Drones randomly spawned in $[-2, 2]\text{ m}$ happened to spawn $0.527\text{ m}$ apart (violating $0.70\text{ m}$ collision threshold at $t=0$). | Created `create_non_overlapping_drones()` fixture ensuring minimum initial $1.2\text{ m}$ separation. Tests passed 100%. |

---

## 4. Current State (As of 30 Sep 2026 Night)

* **Codebase**: Fully functional Phase 1 Python Swarm Engine committed to Git.
* **Tested Regimes**: Centralized, Decentralized, and Hybrid control working and verified.
* **Tested Formations**: Line, V-Formation, Circle, and Grid working with Hungarian slot matching.
* **Verified Safety**: 0 collisions across dynamic switching and $50\%$ packet drop sweeps.
* **Documentation & Artifacts**:
  - `README.md` in repository root.
  - Detailed scientific report: `swarm_simulation_phase1_report.md`.
  - Publication-ready figures saved in `experiments/results/` and artifact directory.
* **Hardware Status**: Untouched, safe, disarmed. Awaiting confirmation from teammates regarding ArduPilot firmware and GPS.

---

## 5. Immediate Next Steps (Phase 1, Track B)

1. **Dual ArduCopter SITL Startup**:
   - Write a shell script to launch 2 headless ArduCopter SITL instances side-by-side:
     - Drone 1: Instance 0 $\to$ MAVLink UDP `127.0.0.1:14550`
     - Drone 2: Instance 1 $\to$ MAVLink UDP `127.0.0.1:14560`
2. **MAVLink Controller Bridge (`pymavlink`)**:
   - Connect the Swarm Controller to the SITL UDP ports.
   - Send `SET_POSITION_TARGET_LOCAL_NED` commands so simulated ArduPilot copters mirror the Python formation algorithms.
3. **Supervisor Alignment**:
   - Share the definition of Hybrid Control (Macro Centralized Planner + Micro Decentralized Collision Avoidance + Flocking Fallback on link loss).
