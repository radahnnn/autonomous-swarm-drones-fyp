# Simulation/SITL Parity

## Purpose

The Autonomous Swarm Drones Coordinated Movement Framework employs a dual-tier evaluation architecture comprising both a lightweight numerical simulator (`simulator/engine.py`) and an ArduPilot Software-in-the-Loop (SITL) integration environment (`sitl/mavlink_swarm_adapter.py`).

Both tiers serve complementary, distinct scientific purposes:
1. **Lightweight Numerical Simulator:** Enables rapid Monte Carlo simulations, reproducible parameter sweeps (over packet loss, latency, GPS noise, and burst outages), algorithmic verification, and graph-theoretic metrics computation (e.g. algebraic connectivity $\lambda_2$) at fast execution speeds ($20\text{ Hz}$ or faster).
2. **ArduPilot SITL Simulator:** Bridges pure mathematical control algorithms to real-world flight avionics. It executes the official ArduPilot autopilot firmware (`arducopter` binary) with full Extended Kalman Filter state estimation (EKF3), sensor emulation, and realistic multirotor physics (`SIM_MULT_DRAG`, rotor inertia), verifying that high-level guidance outputs can be successfully tracked over MAVLink telecommunication links.

---

## Shared components

The following classes, functions, and configuration structures from `swarm_core` are shared across both simulation and SITL:

1. **Formation Geometry & Optimal Assignment (`swarm_core/formations.py`):**
   - `FormationGenerator.get_formation_offsets()`: Computes relative 2D slot coordinates for `LINE`, `V_SHAPE`, `CIRCLE`, and `GRID`.
   - `create_world_slots()`: Translates local formation offsets to world coordinates anchored at the swarm centroid.
   - `assign_optimal_slots()`: Solves the Linear Sum Assignment Problem (Hungarian Algorithm) using `scipy.optimize.linear_sum_assignment` to minimize total squared transit distance and prevent path crossings.
   - `compute_formation_slots()`: High-level entrypoint combining offset generation, world translation, and Hungarian slot matching.
   - `compute_desired_neighbor_offsets()`: Computes relative displacement consensus vectors $(p_{\text{target}, i} - p_{\text{target}, j})$ for decentralized flocking cohesion.

2. **Configuration Profiles & Controller Factory (`swarm_core/config.py`):**
   - `SwarmConfigProfile` & `ParameterProvenance`: Unified source-of-truth ensuring profiles (`assumed_baseline`, `sitl_fitted`) define identical gains and physical parameters.
   - `build_controllers_from_profile()`: Shared builder provisioning `(CentralizedController, DecentralizedController, HybridController)` with identical gains ($K_p, K_d, k_{\text{rep}}, d_{\text{col}}$), consensus gains ($k_{\text{sep}}, k_{\text{align}}, k_{\text{form}}, r_{\text{safe}}$), and hybrid thresholds ($\tau_{\text{deg}}, \tau_{\text{dwell}}, \tau_{\text{ramp}}, w_{\text{size}}, r_{\text{rec}}$).

3. **Guidance Controllers (`swarm_core/controllers/`):**
   - `CentralizedController`: Computes global trajectory tracking and APF collision repulsion.
   - `DecentralizedController`: Computes local 1-hop neighbor consensus, Laplacian velocity alignment, and Reynolds separation.
   - `HybridController`: Evaluates message freshness, sequence numbers, asymmetric hysteresis, and smooth $\alpha(t)$ blending.

4. **Network Channel Emulator (`swarm_core/network.py`):**
   - `WirelessChannel`: Models packet dropouts, latency queues, and Gilbert-Elliott burst outages.

---

## Simulation-only components

The numerical simulator (`simulator/engine.py`) contains elements that do not exist in SITL:

1. **Reduced-Order Drone Dynamics (`swarm_core/drone.py`):**
   - Point-mass Newtonian integration: $\dot{v} = a - c_d v$, $\dot{p} = v$.
   - First-order attitude thrust lag $\tau$: $\dot{a} = (u - a) / \tau$.
2. **Direct Acceleration Integration:**
   - Guidance commands $u \in \mathbb{R}^2$ ($m/s^2$) directly drive the point-mass acceleration state.
3. **Synthetic Sensor Noise Injection:**
   - Synthetic 2D Gaussian GPS noise with dual-component spatial correlation (common-mode fraction + independent white noise).
4. **Simulator-Only Online Metrics Tracking (`swarm_core/metrics.py`):**
   - Real-time computation of Laplacian algebraic connectivity (Fiedler eigenvalue $\lambda_2$), communication graph adjacency, minimum pairwise distance time-history, and cumulative collision counts.
5. **Interactive 2D Visualizer (`simulator/visualizer.py`):**
   - Live Matplotlib animation of drone positions, velocity vectors, target slots, and wireless links.

---

## SITL-only components

The SITL integration environment (`sitl/mavlink_swarm_adapter.py`) contains avionics components that do not exist in the lightweight simulator:

1. **MAVLink Transport Layer (`sitl/mavlink_swarm_adapter.py`):**
   - Serial/UDP socket management (`udpin:127.0.0.1:14552`, `14562`, `14572`) via `pymavlink`.
   - Asynchronous telemetry polling (`GLOBAL_POSITION_INT`, `ATTITUDE`, `LOCAL_POSITION_NED`) and heartbeat dispatch.
2. **Common Global Reference Frame (`sitl/common_frame.py`):**
   - Converts between WGS84 geodetic coordinates (Latitude, Longitude, Altitude) and a shared metric Cartesian tangent plane centered at a common datum (`Lat0 = -35.3632621`, `Lon0 = 149.1652374`, `Alt0 = 584.0m`).
   - Translates global datum coordinates to vehicle-specific local NED frames anchored at each drone's home position.
3. **ArduPilot GUIDED Mode & Position Controller:**
   - Guidance outputs are dispatched as `SET_POSITION_TARGET_LOCAL_NED` messages.
   - ArduPilot's internal cascaded control loops (`POS_XYZ_P` $\rightarrow$ `VEL_XYZ_PID` $\rightarrow$ `ACC_XYZ_PID` $\rightarrow$ attitude rate loops $\rightarrow$ motor mixer) execute inside the autopilot firmware.
4. **EKF3 Sensor Fusion:**
   - Multi-state Extended Kalman Filter fusing simulated IMU (accelerometers, gyroscopes), GNSS receiver, barometer, and compass.
5. **Real Process and Telemetry Timing:**
   - Discrete 10 Hz companion computer execution loop, asynchronous operating system scheduling, socket buffers, and network jitter.

---

## Known differences

| Pipeline Aspect | Simulation (`simulator/engine.py`) | SITL Adapter (`sitl/mavlink_swarm_adapter.py`) |
|:---|:---|:---|
| **Control Output** | Direct acceleration vector $u \in \mathbb{R}^2$ ($m/s^2$). | 3D Local-NED position setpoints $P_{\text{sp}} \in \mathbb{R}^3$ ($m$). |
| **Control Conversion** | None (acceleration fed directly to point mass). | Kinematic forward integration: $P_{\text{sp}} = [p_x + u_x \Delta t \gamma, p_y + u_y \Delta t \gamma, -z_{\text{cruise}}]$. |
| **Lookahead Lead Scale** | Not applicable ($\gamma = 1.0$). | $\gamma = 1.0$ (Decentralized), $\gamma = 2.0$ (Hybrid lead compensation). |
| **Inner Loop Control** | None (idealized 1st-order lag $\tau$). | ArduPilot cascaded Position/Velocity/Attitude PID loops. |
| **Update Frequency** | $20\text{ Hz}$ ($\Delta t = 0.05\text{ s}$). | $10\text{ Hz}$ ($\Delta t = 0.10\text{ s}$) matching companion computer MAVLink stream rate. |
| **Coordinate Space** | 2D Euclidean plane ($x, y$). | 3D NED space ($x, y$ lateral, $z = -5.05\text{ m}$ cruise altitude). |
| **Telemetry Delay** | Simulated mean latency (e.g. $30\text{ ms}$). | Real IPC socket transport delay + MAVLink serialization + EKF filter lag. |
| **Slot Assignment** | Hungarian assignment via `compute_formation_slots()`. | Hungarian assignment via `compute_formation_slots()`. |
| **Profile Usage** | Selected profile (e.g. `assumed_baseline` or `sitl_fitted`). | Explicitly uses `sitl_fitted` profile parameters. |

### Control Conversion Semantics
```text
[swarm_core Guidance Output]
       │  (u_hyb ∈ ℝ², m/s²)
       ▼
[acceleration_to_position_setpoint]
       │  P_sp = P_curr + u_hyb · Δt · γ  (γ = 2.0 Lead Filter)
       ▼
[MAVLink SET_POSITION_TARGET_LOCAL_NED]
       │  (Local NED frame relative to vehicle EKF origin)
       ▼
[ArduPilot GUIDED Mode Position Controller]
       │  (POS_XYZ_P ──► VEL_XYZ_PID ──► ACC_XYZ_PID)
       ▼
[SITL / Gazebo Multirotor 6-DOF Dynamics]
```

---

## Scientific impact

1. **Tracking Error:**
   - *Impact:* SITL exhibits slightly higher transient tracking error ($0.10 - 0.25\text{ m}$) during high-rate formation morphs than simulation ($< 0.05\text{ m}$).
   - *Cause:* Autopilot position controller phase lag and physical drag deceleration.
2. **Settling Time:**
   - *Impact:* Settling time in SITL is approximately $1.5\text{ s}$ longer than in idealized simulation.
   - *Cause:* The identified translation time constant in SITL is $\tau = 0.992\text{ s}$, compared to the assumed fast literature baseline $\tau = 0.18\text{ s}$.
3. **Formation Assignment:**
   - *Impact:* **Zero difference.** Both pipelines use the identical Hungarian matching algorithm (`compute_formation_slots`), producing identical slot assignment matrices for identical vehicle coordinate inputs.
4. **Collision Risk:**
   - *Impact:* APF repulsion is computed using the identical formulation ($r < r_{\text{safe}}$) in both paths. However, in SITL, the autopilot's tracking lag requires a larger safety buffer; hence `sitl_fitted` uses $r_{\text{safe}} = 1.50\text{ m}$ rather than $1.20\text{ m}$.
5. **Latency Response:**
   - *Impact:* In both environments, packet latency $> 150\text{ ms}$ triggers degradation to decentralized fallback. SITL incurs additional telemetry buffer delay, making the effective degradation window slightly more conservative.
6. **Comparison Validity:**
   - *Impact:* Direct quantitative comparison of raw trajectories between simulation and SITL is scientifically invalid unless the difference in inner-loop controller architecture is explicitly stated. Comparisons must focus on high-level metrics: mode transition stability, formation convergence, network loss tolerance, and inter-drone clearance.

---

## Claims that are allowed

### Permitted Academic Phrasing
- "The SITL adapter reuses swarm_core controller components and translates their guidance outputs into ArduPilot local-NED position setpoints. ArduPilot’s onboard position controller and SITL dynamics remain part of the execution path."
- "The Hungarian slot assignment and hybrid supervisor state machine execute identically across simulation and SITL."
- "Inter-drone collision avoidance is empirically mitigated by APF separation control and distance-minimizing slot assignment in tested scenarios."

### Prohibited Phrasing
- Do **NOT** claim: "The simulator and SITL use the exact same pipeline." (Guidance-to-setpoint conversion and autopilot loops differ).
- Do **NOT** claim: "The simulator and SITL have the exact same dynamics." (Point mass vs. 6-DOF copter physics).
- Do **NOT** claim: "The framework has undergone sim-to-real hardware validation." (Tested in SITL simulation only; physical 6S Matek H743 hardware flights have not yet occurred).
- Do **NOT** claim: "The framework provides a mathematical collision-free guarantee." (APF and Hungarian assignment empirically reduce risk; formal safety proofs are not implemented).

---

*Verified under Phase 4 closeout. Three-drone configuration verified.*
