# Final Year Project: Sub-Supervisor Meeting Briefing Guide
**Project:** Autonomous Swarm Drones Coordinated Movement Framework (BEE-60)  
**Date:** October 2, 2026  
**Audience:** MS Sub-Supervisor / Research Assistant  
**Goal:** Present progress clearly, demonstrate engineering rigor, and defend design decisions with empirical data.

---

## 1. The 30-Second Elevator Pitch
> *"Our project tackles the fundamental reliability problem of autonomous drone swarms flying dynamic geometric formations. Centralized ground control is accurate but crashes when wireless packets drop; decentralized peer flocking survives dropouts but has high tracking error and slow convergence.*  
> 
> *We have developed an **asymmetric hybrid switching controller** that tracks optimal formations under good link conditions, drops into decentralized consensus flocking during outages with zero rapid chattering, and blends smoothly back when communication recovers. We have validated this not just in Python, but directly in **ArduPilot SITL multi-vehicle simulation** with fitted quadrotor dynamics, and prepared the complete hardware deployment pipeline for our Matek H743 flight controllers."*

---

## 2. Key Progress Pillars (What We Accomplished)

### Pillar 1: Algorithmic Architecture & The Hybrid Switch
* **Centralized Mode**: Implemented Hungarian minimum-displacement slot matching (`scipy.optimize.linear_sum_assignment`) so drones never cross paths when changing formations. Added analytical velocity feedforward ($c_d \mathbf{v}_{\text{target}}$), reducing steady-state tracking error from **$1.14\text{ m}$ down to $0.018\text{ m}$ ($98.4\%$ reduction)**.
* **Decentralized Mode**: 1-hop peer velocity consensus ($\dot{\mathbf{v}} = -\mathbf{L}\mathbf{v}$) with an onboard Artificial Potential Field (APF) safety barrier that is permanently active directly on each drone.
* **Hybrid Hardening**:
  * **Degrade Trigger**: Drops to fallback if coordinator heartbeats are missing for $> 0.50\text{ s}$.
  * **Sliding-Window Recovery**: Only recovers if $\ge 70\%$ of the last 20 heartbeats are received (replaces naive consecutive packet rules that fail on lossy links).
  * **Dwell-Time Lockout**: Enforces a $2.0\text{ s}$ freeze in decentralized mode after a drop, completely eliminating rapid switching ("chattering").
  * **Continuous Blending**: Smoothly ramps control authority ($\alpha(t) \in [0, 1]$ over $0.5\text{ s}$) upon recovery.

### Pillar 2: Empirical Testing & Stress Benchmarks
* **Deterministic Outages**: Tested complete $1.0\text{ s}$, $2.0\text{ s}$, and $3.0\text{ s}$ communication blackouts. Drones cleanly enter fallback, hold formation via consensus, and recover within $0.85\text{ s}$ without collisions.
* **Gilbert-Elliott Burst Loss**: Evaluated correlated RF fading channels across loss rates from $0\%$ to $80\%$.
* **GPS Noise Sweep**: Evaluated sensor noise $\sigma \in \{0.04, 0.5, 1.5, 2.5\}\text{ m}$ using a realistic decomposed model (shared atmospheric error + independent multipath per drone). Swarm maintained a minimum separation of $1.52\text{ m}$, safely exceeding our $1.50\text{ m}$ physical threshold.
* **Rigorous Baseline Comparison**: Benchmarked on identical random seeds across 4 controllers: Centralized (Hold-Last), Decentralized, Naive Hybrid, and Hardened Hybrid.

### Pillar 3: Sim-to-Real Cross-Validation in ArduPilot SITL
* **Dynamic Parameter Identification**:
  * Ran a $5.0\text{ m}$ position step response test on ArduCopter in GUIDED mode.
  * Fitted the Python engine's lag ($\tau$) and drag ($c_d$) to SITL telemetry:
    * $\tau = \mathbf{0.992\text{ s}}$ (effective translation time constant)
    * $c_d = \mathbf{0.637\text{ s}^{-1}}$ (aerodynamic rotor drag)
    * Residual RMSE between model and SITL: **$15.36\text{ cm}$**.
* **3-Drone Formation Scenario Overlay**:
  * Ran an identical 3-drone V-formation translation scenario in both Python `swarm_core` and ArduPilot SITL on a synchronized $10\text{ Hz}$ timebase.
  * Real trajectory discrepancy across an $8\text{ m}$ flight was **$72.50\text{ cm}$ RMS**. (Explains to the supervisor: this is an honest, physical sim-to-real discrepancy due to motor spool-up and EKF3 estimation lag, rather than unrealistic claims of 2 mm accuracy).
* **Coordinate Frame Integrity**:
  * Built `CommonCoordinateFrame` converting individual drone GPS origins to a unified metric tangent plane with **$< 0.1\text{ mm}$ round-trip error** up to $100\text{ m}$.

### Pillar 4: Physical Hardware & Systems Blueprint
* Completed full research and parameter mapping for our target hardware:
  * **Flight Controller**: Matek H743-SLIM V3 running ArduPilot Copter 4.6 in GUIDED mode.
  * **Serial & Radio Link**: ESP32 Wi-Fi bridge connected to UART7 (`SERIAL1`) @ 921,600 baud using MAVLink2.
  * **Motor Protocol**: Bi-directional DShot600 with RPM notch filtering for 5-inch quadcopters.
  * **Failsafes**: Configured `FS_GCS_ENABLE = 2` (RTL) and `FS_OPTIONS = 32`, with ArduPilot holding position if MAVLink `#84` setpoints cease for $> 3\text{ s}$.

---

## 3. Key Numbers & Metrics to Quote

If she asks for specific data, cite these numbers confidently:
* **Test Suite**: 15 out of 15 unit tests passing.
* **Feedforward Benefit**: Reduced steady-state tracking error from **$1.14\text{ m}$ to $0.018\text{ m}$ ($98.4\%$)**.
* **Sim-to-SITL Step Fit Residual**: **$15.36\text{ cm}$ RMSE** ($\tau = 0.992\text{ s}$, $c_d = 0.637$).
* **Swarm SITL Trajectory Difference**: **$72.50\text{ cm}$ RMS** over an $8\text{ m}$ flight.
* **WGS84 Tangent Frame Round-Trip Error**: **$< 0.1\text{ mm}$** within $100\text{ m}$.
* **GPS Noise Tolerance**: Maintained $\ge 1.52\text{ m}$ physical clearance under $1.5\text{ m}$ GPS noise.

---

## 4. Likely Questions & How to Answer Them

### Q1: *"Why did you write a custom Python engine instead of using ROS 2 / Mavros?"*
* **Answer**: *"ROS 2 and Mavros add significant middleware overhead, complex serialization, and high CPU usage that makes emulating 5+ high-fidelity SITL drones and running 100-seed Monte Carlo network sweeps slow and fragile. By building a clean MAVLink adapter layer (`MAVLinkSwarmAdapter`), our core algorithms run identically in lightweight Python simulations and directly against MAVLink autopilots over standard UDP/serial sockets. If required later, ROS 2 nodes can wrap this adapter in less than a day."*

### Q2: *"How do you handle plain GPS inaccuracies (1–2.5 m) without RTK?"*
* **Answer**: *"We explicitly evaluated GPS noise with common-mode atmospheric drift (shared across drones within 50 m) plus independent receiver multipath. Because co-located receivers share the ionospheric error, their relative baseline error is lower ($1.0 - 1.5\text{ m}$) than absolute error ($2.5\text{ m}$). We set our formation inter-drone clearance to $2.5\text{ m}$ with an APF safety repulsion boundary at $1.5\text{ m}$, ensuring drones maintain physical separation even under $1.5\text{ m}$ GPS noise. We also documented the hardware upgrade path to dual u-blox F9P RTK if department funding allows."*

### Q3: *"Why use UART serial instead of DroneCAN for the ESP32 connection?"*
* **Answer**: *"We investigated ArduPilot's DroneCAN implementation in depth. While DroneCAN is excellent for sensors and actuators (GPS, compass, ESCs), ArduPilot does not currently expose subscriber nodes for GUIDED mode trajectory setpoints (`SET_POSITION_TARGET_LOCAL_NED`) over CAN. Connecting the ESP32 via UART7 (`SERIAL1`) running MAVLink2 at 921,600 baud is the native, battle-tested standard for offboard companion computers."*

### Q4: *"What prevents the drones from colliding when you morph formations?"*
* **Answer**: *"We use Hungarian optimal assignment (`linear_sum_assignment`) on the Euclidean distance matrix between current drone positions and new formation slots. This guarantees minimum total displacement and eliminates crossing trajectories. In addition, the Artificial Potential Field repulsion runs locally on each drone at all times, providing a safety net if dynamic lag causes temporary proximity."*

---

## 5. Next Steps to Propose for the Coming Week

1. **Scale SITL to 5 Drones**:
   * Automate parallel launch for 5 ArduCopter instances (SysID 1–5, Ports 5760–5800).
   * Demonstrate dynamic formation transitions (V-shape $\to$ Line $\to$ Circle $\to$ Grid) in SITL under emulated packet loss.
2. **Hardware Bench Testing (2 Drones)**:
   * Connect an ESP32 running the UDP-to-UART bridge to the Matek H743 flight controller on the bench.
   * Verify telemetry stream rate (10 Hz) and MAVLink setpoint reception with QGroundControl monitoring.
