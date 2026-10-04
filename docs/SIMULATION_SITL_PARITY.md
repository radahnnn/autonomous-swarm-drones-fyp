# Simulation/SITL Parity

> [!IMPORTANT]
> **SUPERSEDED NOTICE (Task B Closeout)**  
> All earlier parity drafts, preliminary reports, and exploratory notes generated prior to the Phase 1 closeout protocol (specifically those utilizing open-loop `.values` array overwrites, target-filled missing telemetry frames, unverified parameter assignments, or zero-initialized Gauss-Markov noise) are hereby **SUPERSEDED** by this document and the accompanying validation artifacts.

---

## 1. Purpose

The Autonomous Swarm Drones Coordinated Movement Framework employs a dual-tier evaluation architecture comprising both a lightweight numerical simulator (`simulator/engine.py`) and an ArduPilot Software-in-the-Loop (SITL) integration environment (`sitl/mavlink_swarm_adapter.py`).

Both tiers serve complementary, distinct scientific purposes:
1. **Lightweight Numerical Simulator (`simulator/engine.py`):** Enables rapid Monte Carlo simulations, reproducible parameter sweeps (over packet loss, latency, GPS noise, and burst outages), algorithmic verification, and graph-theoretic metrics computation (e.g. algebraic connectivity $\lambda_2$) at fast execution speeds ($20\text{ Hz}$ or faster).
2. **ArduPilot SITL Simulator (`sitl/mavlink_swarm_adapter.py`):** Bridges pure mathematical control algorithms to real-world flight avionics. It executes the official ArduPilot autopilot firmware (`arducopter` binary) with full Extended Kalman Filter state estimation (EKF3), sensor emulation, and realistic multirotor physics (`SIM_MULT_DRAG`, rotor inertia), verifying that high-level guidance outputs can be successfully tracked over MAVLink telecommunication links.

---

## 2. Shared Components

The following classes, functions, and configuration structures from `swarm_core` are shared across both simulation and SITL:

1. **Formation Geometry & Optimal Assignment (`swarm_core/formations.py`):**
   - `FormationGenerator.get_formation_offsets()`: Computes relative 2D slot coordinates for `LINE`, `V_SHAPE`, `CIRCLE`, and `GRID`.
   - `create_world_slots()`: Translates local formation offsets to world coordinates anchored at the swarm centroid.
   - `assign_optimal_slots()`: Solves the Linear Sum Assignment Problem (Hungarian Algorithm) using `scipy.optimize.linear_sum_assignment` to minimize total squared transit distance and prevent path crossings.
   - `compute_formation_slots()`: Single entrypoint combining offset generation, world translation, and Hungarian slot matching.
   - `compute_desired_neighbor_offsets()`: Computes relative displacement consensus vectors $(p_{\text{target}, i} - p_{\text{target}, j})$ for decentralized flocking cohesion.

2. **Configuration Profiles & Controller Factory (`swarm_core/config.py`):**
   - `SwarmConfigProfile` & `ParameterProvenance`: Unified source-of-truth ensuring profiles (`assumed_baseline`, `sitl_default_quad`) define identical gains and physical parameters.
   - `build_controllers_from_profile()`: Shared builder provisioning `(CentralizedController, DecentralizedController, HybridController)` with identical gains ($K_p, K_d, k_{\text{rep}}, d_{\text{col}}$), consensus gains ($k_{\text{sep}}, k_{\text{align}}, k_{\text{form}}, r_{\text{safe}}$), and hybrid thresholds ($\tau_{\text{deg}}, \tau_{\text{dwell}}, \tau_{\text{ramp}}, w_{\text{size}}, r_{\text{rec}}$).

3. **Guidance Controllers (`swarm_core/controllers/`):**
   - `CentralizedController`: Computes global trajectory tracking and APF collision repulsion.
   - `DecentralizedController`: Computes local 1-hop neighbor consensus, Laplacian velocity alignment, Reynolds separation, and $k_{\text{goal}}$ waypoint attraction.
   - `HybridController`: Evaluates message freshness, sequence numbers, asymmetric hysteresis, and smooth $\alpha(t)$ blending.

4. **Network Channel Emulator (`swarm_core/network.py`):**
   - `WirelessChannel`: Models packet dropouts, latency queues, targeted network partitions, and Gilbert-Elliott burst outages advanced per-tick per-recipient.

---

## 3. Simulation-Only Components

The numerical simulator (`simulator/engine.py`) contains elements that do not exist in SITL:

1. **Reduced-Order Drone Dynamics (`swarm_core/drone.py`):**
   - Point-mass Newtonian integration: $\dot{v} = a - c_d v$, $\dot{p} = v$.
   - First-order attitude thrust lag $\tau$: $\dot{a} = (u - a) / \tau$.
2. **Direct Acceleration Integration:**
   - Guidance commands $u \in \mathbb{R}^2$ ($m/s^2$) directly drive the point-mass acceleration state.
3. **Synthetic Sensor Noise Injection:**
   - Synthetic 2D Gauss-Markov time-correlated GPS noise (correlation time $\tau_c = 30\text{ s}$) initialized directly from stationary distribution $\mathcal{N}(0, \sigma^2)$ (Item 28).
4. **Simulator-Only Online Metrics Tracking (`swarm_core/metrics.py`):**
   - Real-time computation of Laplacian algebraic connectivity (Fiedler eigenvalue $\lambda_2$), communication graph adjacency, centroid-removed formation shape error, minimum pairwise distance time-history, and cumulative collision counts.
5. **Interactive 2D Visualizer (`simulator/visualizer.py`):**
   - Live Matplotlib animation of drone positions, velocity vectors, target slots, and wireless links.

---

## 4. SITL-Only Components

The SITL integration environment (`sitl/mavlink_swarm_adapter.py`) contains avionics components that do not exist in the lightweight simulator:

1. **MAVLink Transport Layer (`sitl/mavlink_swarm_adapter.py`):**
   - Serial/UDP socket management (`udpin:127.0.0.1:14552`, `14562`, `14572`) via `pymavlink`.
   - Asynchronous telemetry polling (`GLOBAL_POSITION_INT`, `ATTITUDE`, `LOCAL_POSITION_NED`) and heartbeat dispatch.
   - Setpoint dispatch using exact 16-argument MAVLink `SET_POSITION_TARGET_LOCAL_NED` messages (Item 24).
2. **Common Global Reference Frame (`sitl/common_frame.py`):**
   - Converts between WGS84 geodetic coordinates (Latitude, Longitude, Altitude) and a shared metric Cartesian tangent plane centered at a common datum (`Lat0 = -35.3632621`, `Lon0 = 149.1652374`, `Alt0 = 584.0m`).
   - Dynamic origin computation: $p_{\text{origin}} = p_{\text{global\_ned}} - p_{\text{local\_ned}}$ from simultaneous telemetry messages, guaranteeing drift invariance (Item 17).
3. **ArduPilot GUIDED Mode & Position Controller:**
   - Velocity setpoints (mask `0x0DC7`) or position + velocity feedforward setpoints (mask `0x0DC0`).
   - Autopilot internal cascaded control loops (`POS_XYZ_P` $\rightarrow$ `VEL_XYZ_PID` $\rightarrow$ `ACC_XYZ_PID` $\rightarrow$ attitude rate loops $\rightarrow$ motor mixer) execute inside autopilot firmware.
4. **EKF3 Sensor Fusion:**
   - Multi-state Extended Kalman Filter fusing simulated IMU (accelerometers, gyroscopes), GNSS receiver, barometer, and compass.
5. **Flight Safety Guardrails (Item 20):**
   - GUIDED-mode checks, telemetry watchdogs (1.5s timeout), 3D geofence enforcement, setpoint distance clamping, and emergency stop / kill handling.

---

## 5. Known Differences

| Pipeline Aspect | Simulation (`simulator/engine.py`) | SITL Adapter (`sitl/mavlink_swarm_adapter.py`) |
|:---|:---|:---|
| **Control Output** | Direct acceleration vector $u \in \mathbb{R}^2$ ($m/s^2$). | Velocity setpoints (mask `0x0DC7`) or Pos+Vel setpoints (`0x0DC0`). |
| **Control Conversion** | None (acceleration fed directly to point mass). | Kinematic forward integration through `IntegratedPlant` matching `engine.py`. |
| **Inner Loop Control** | None (idealized 1st-order lag $\tau$). | ArduPilot cascaded Position/Velocity/Attitude PID loops. |
| **Update Frequency** | $20\text{ Hz}$ ($\Delta t = 0.05\text{ s}$). | $10\text{ Hz}$ ($\Delta t = 0.10\text{ s}$) matching companion computer MAVLink stream rate. |
| **Coordinate Space** | 2D Euclidean plane ($x, y$). | 3D NED space ($x, y$ lateral, $z = -5.0\text{ m}$ cruise altitude). |
| **Slot Assignment** | Hungarian assignment via `compute_formation_slots()`. | Hungarian assignment via `compute_formation_slots()`. |
| **Slot Lock** | Locked at morph start to prevent transit chattering. | Locked at morph start to prevent transit chattering. |
| **Profile Usage** | Selected profile (e.g. `assumed_baseline` or `sitl_default_quad`). | Explicitly uses `sitl_default_quad` profile parameters. |
| **Lumped Dynamics** | Point-mass lag $\tau = 0.18\text{ s}$ (`assumed`) or $\tau = 0.992\text{ s}$ (`sitl_default_quad`). | Identified lumped translation time constant $\tau_{\text{trans}} = 0.992\text{ s}$. |
| **Sensor Model** | Gauss-Markov noise initialized from stationary distribution $\mathcal{N}(0, \sigma^2)$. | ArduPilot simulated GNSS coupled to EKF3 state estimator. |

---

## 6. Scientific Impact and Metrics Parity

1. **Trajectory Tracking Error:**
   - *Result:* Absolute trajectory RMS discrepancy between simulation and SITL is $4.48\text{ m}$ during high-rate dynamic translation, primarily reflecting autopilot position controller phase lag and drag braking.
2. **Formation Shape Error (Centroid-Removed) (Item 29):**
   - *Result:* Shape distortion is $3.68\text{ m}$ RMS, isolating internal geometry deformation from rigid translation lag.
3. **Physical Clearance & Minimum Separation (Item 29):**
   - *Result:* True minimum separation is $3.00\text{ m}$ in simulation and $3.86\text{ m}$ in SITL, both comfortably exceeding the $0.70\text{ m}$ physical collision threshold and respecting the active APF barrier ($1.20 - 1.50\text{ m}$).
4. **Assembly Convergence Protocol (Item 30):**
   - *Result:* Both tiers ensure formation assembly reaches $< 0.25\text{ m}$ slot convergence prior to starting dynamic evaluation scenarios, with initial error explicitly logged.
5. **Raw Telemetry Provenance (Item 25):**
   - *Result:* Every SITL run archives raw telemetry with firmware version, parameter dump (`sitl/swarm_params.parm`), type mask, and UTC timestamps. Silent cached-CSV fallbacks and open-loop `.values` overwrites are removed.

---

## 7. Claims That Are Allowed

### Permitted Academic Phrasing
- "The SITL adapter reuses swarm_core controller components and translates guidance commands into ArduPilot MAVLink setpoints, validating tracking under full EKF3 estimation and cascaded autopilot loops."
- "The Hungarian slot assignment, neighbor memory age-out, and hybrid supervisory state machine execute identically across numerical simulation and SITL."
- "Centroid-removed formation shape error and true physical minimum separation confirm stable inter-vehicle clearance across simulated link outages."

### Prohibited Phrasing
- Do **NOT** claim: "The simulator and SITL share an identical execution pipeline." (Kinematic conversion and firmware inner loops differ).
- Do **NOT** claim: "The framework has completed sim-to-real flight testing." (Software-in-the-loop only; physical flight tests on 6S Matek H743 hardware have not yet been conducted).
- Do **NOT** claim: "Zero trajectory discrepancy exists between simulation and SITL." (Honest RMS trajectory divergence is reported and documented).

---

*Updated and verified under Task B / Phase 1 closeout protocol. Supersedes all prior parity reports.*
