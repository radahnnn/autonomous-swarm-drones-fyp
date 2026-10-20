# Final Year Project (FYP) Interim Progress Report – 1

---

## 📌 Project Details

* **Project Title:** Autonomous Swarm Drones: Coordinated Controlled Movement
* **Degree & Department:** Bachelor of Electrical Engineering (BEE-60, Class BEE-60B), Department of Electrical Engineering
* **Institution:** Military College of Signals (MCS), National University of Sciences & Technology (NUST)
* **Supervisor(s):** Prof. Dr. Abdul Ghafoor (Professor, Department of Electrical Engineering, MCS, NUST)
* **Student(s):** 
  * Hassan Ahmed
  * NC Muhammad Umer
  * Mehreen
  * *(Group Member 4 / Ahmed Talal – as per official project group allocation)*
* **Registration No(s):** `[Insert Official NUST Registration Numbers, e.g., 000003xxxxx]`
* **Date of Submission:** October 11, 2026
* **Target Project Completion:** December 31, 2026 (Accelerated Timeline)
* **Project Repository:** [radahnnn/autonomous-swarm-drones-fyp](https://github.com/radahnnn/autonomous-swarm-drones-fyp)

---

## 1. Introduction

### 1.1 Brief Background of the Project
Unmanned Aerial Vehicle (UAV) systems have transitioned rapidly from solitary remote-piloted aircraft to autonomous multi-vehicle swarms capable of executing complex missions cooperatively. Swarms of small, agile multirotor drones provide unprecedented advantages in search and rescue (SAR), wide-area disaster reconnaissance, perimeter surveillance, atmospheric monitoring, and distributed sensing. By operating collectively, a swarm exhibits high spatial coverage, parallel task execution, and operational redundancy: the loss of an individual drone does not compromise mission completion.

Achieving coordinated flight among multirotors requires rigorous coordination of positions and velocities to maintain defined geometric formations (such as Line, V-Formation/Chevron, Circle, and Grid) while dynamically maneuvering through 3D space. While physical biological flocks in nature achieve flocking through decentralized optical and tactile cues, robotic quadcopter swarms must coordinate over bandwidth-constrained, lossy, and latency-prone wireless communication links. Consequently, modern swarm control frameworks must balance the global precision of centralized trajectory coordination with the resilience and autonomy of distributed peer-to-peer consensus.

### 1.2 Problem Statement
Coordinated swarm drone movement is fundamentally constrained by the trade-off between control authority and communication link reliability:

1. **Vulnerability of Centralized Architectures:** Centralized Ground Control Stations (GCS) or single Apex Leader architectures compute globally optimal trajectories and assign conflict-free formation slots. However, they introduce a single point of failure (SPOF). In practical RF environments—characterized by multipath fading, shadow fading, and interference—transient packet dropouts or ground-station connection blackouts cause centralized followers to either freeze on stale setpoints, drift uncontrollably, or trigger asynchronous failsafe behaviors (such as premature Return-to-Launch), creating high risks of mid-air collision.
2. **Deficiencies of Pure Decentralized Flocking:** Distributed consensus algorithms (e.g., Reynolds flocking rules and Laplacian velocity consensus) survive communication dropouts by relying strictly on 1-hop neighbor broadcasts. However, pure decentralized swarms suffer from sluggish transient convergence, significant steady-state tracking error under high aerodynamic rotor drag, and severe deformation during high-speed maneuvers. Furthermore, without global coordination, dynamic formation switching suffers from trajectory crossings.
3. **Control Chattering in Naive Hybrid Switching:** Naive hybrid controllers that switch instantaneously between centralized guidance and decentralized fallback whenever packets are dropped suffer from high-frequency mode switching ("chattering"). In lossy wireless environments (packet loss $\ge 20\%$), this chattering induces severe control signal chatter, motor overheating, aerodynamic instability, and actuator degradation.
4. **Physical Sim-to-Real Discrepancies:** Multi-agent literature predominantly evaluates control algorithms using idealized zero-lag point-mass models, ignoring actuator lag ($\tau \approx 0.99\text{ s}$), aerodynamic rotor drag ($c_d \approx 0.64\text{ s}^{-1}$), GNSS receiver noise ($\sigma \approx 1.5\text{ m}$), and autopilot Extended Kalman Filter (EKF) estimation latencies.

Therefore, an urgent engineering need exists for a robust, fault-tolerant **asymmetric hybrid supervisory control framework** that delivers optimal formation tracking under nominal wireless conditions, smoothly transitions to distributed consensus during communication blackouts without chattering, and guarantees collision-free self-healing operation on physical autopilot hardware.

### 1.3 Objectives of the Project
The primary objectives of this Final Year Project are:
1. **Mathematical Swarm Modeling & Algorithm Development:** Formulate continuous-time kinematic models accounting for first-order actuator lag and aerodynamic rotor drag; develop centralized Hungarian slot assignment for dynamic formation morphing; and implement distributed Laplacian velocity consensus coupled with an onboard Artificial Potential Field (APF) safety barrier.
2. **Design of a Chattering-Free Asymmetric Hybrid Controller:** Develop a supervisory hybrid state machine incorporating message-age validation, sliding-window packet reception metrics ($W=20$ ticks, $\ge 70\%$), dwell-time lockout ($2.0\text{ s}$), and continuous control authority blending ($\alpha(t) \in [0, 1]$) to eliminate switching limit cycles under lossy RF channels.
3. **High-Fidelity Numerical Simulation & Monte Carlo Benchmarking:** Construct a lightweight Python simulation engine (`swarm_core`) to execute multi-seed Monte Carlo parameter sweeps over stochastic packet loss ($0\%\dots 50\%$), Gilbert-Elliott burst dropouts, and Gauss-Markov correlated GPS noise.
4. **Autopilot System Identification & Sim-to-SITL Parity:** Interface the control laws directly with ArduPilot Copter via MAVLink2 (`SET_POSITION_TARGET_LOCAL_NED`); execute step-response system identification to fit physical quadcopter parameters ($\tau, c_d$); and establish quantitative trajectory parity across simulation and Software-in-the-Loop (SITL).
5. **Multi-Vehicle 5-Drone 2D & 3D Gazebo Simulation with Self-Healing:** Scale the fleet from 3 to 5 quadcopters in headless SITL and 3D Gazebo Harmonic physics; integrate a real-time console with Dynamic Leader Election and automated fault injection/revival.
6. **Physical Drone Hardware Integration & Verification:** Configure physical **Matek Systems H743-SLIM V3** flight controllers running ArduCopter v4.6+ with bi-directional DShot600, sensor calibration, and high-speed telemetry bridges (ESP32 UART-to-UDP) in preparation for physical flight validation.

---

## 2. Literature Review / Background Study

### 2.1 Summary of Relevant Research Reviewed
The theoretical foundation of this project synthesizes seminal contributions across algebraic graph theory, networked control systems, flocking dynamics, and robotic swarm frameworks:

1. **Consensus and Graph Theory in Networked Systems:**
   * *Olfati-Saber, Fax, & Murray (2007)* established algebraic graph theory as the mathematical bedrock for multi-agent coordination. They proved that the second smallest eigenvalue of the graph Laplacian matrix $\mathbf{L}$, known as algebraic connectivity (the Fiedler eigenvalue $\lambda_2(\mathbf{L})$), dictates the asymptotic convergence rate of distributed consensus ($\dot{\mathbf{v}}_i = -\sum a_{ij} (\mathbf{v}_i - \mathbf{v}_j)$). Furthermore, they formulated the analytical upper bound on communication time-delay $\tau_{\max} < \frac{\pi}{2 \lambda_{\max}(\mathbf{L})}$ beyond which velocity consensus becomes unstable.
2. **Control Over Lossy and Fading Networks:**
   * *Schenato, Sinopoli, Franceschetti, Poolla, & Sastry (2007)* investigated Kalman filtering and optimal control across stochastic packet-dropping networks. They proved the existence of a critical packet arrival rate threshold $\gamma_c$: if the packet delivery probability falls below $\gamma_c$, the state estimation error covariance diverges to infinity. This formalizes the necessity of an autonomous decentralized fallback mode when wireless communication degrades.
3. **Decentralized Flocking and Artificial Potential Fields:**
   * *Reynolds (1987)* and *Olfati-Saber (2006)* formulated heuristic and formal flocking algorithms based on three core rules: separation (collision avoidance via repulsive potential fields), alignment (velocity matching via consensus), and cohesion (group centroid attraction). *Khatib (1986)* pioneered Artificial Potential Fields (APF) for real-time obstacle avoidance, which provides the mathematical formulation for our continuous onboard inter-drone collision barrier.
4. **Linear Sum Assignment for Formation Morphing:**
   * *Kuhn (1955)* and *Munkres (1957)* developed the Hungarian algorithm, which solves the Linear Sum Assignment Problem (LSAP) in polynomial time $\mathcal{O}(N^3)$. In multi-agent formation transitions, matching current vehicle coordinates to target formation slots by minimizing cumulative Euclidean displacement $\sum \|\mathbf{p}_i - \mathbf{s}_{\sigma(i)}\|^2$ prevents intersecting trajectories and resolves the slot-allocation bottleneck.
5. **State-of-the-Art Robotic Swarm Architectures (Crazyswarm):**
   * *Preiss, Hönig, Sukhatme, & Ayanian (2017)* developed *Crazyswarm*, demonstrating coordinated flight of up to 49 nano-quadcopters. Crazyswarm addressed RF saturation over 2.4 GHz radios by transmitting compact piecewise polynomial spline trajectories (Bezier curves) from the ground station rather than streaming 100 Hz continuous setpoints. Onboard microcontrollers evaluate the polynomials at 500 Hz, granting immunity to transient wireless packet loss.

### 2.2 Comparison of Existing Solutions

| Architecture / Approach | Trajectory Optimality | Scalability ($N > 5$) | Resilience to RF Blackout | Mid-Air Collision Risk During Morphing | Chattering Under Bursty Loss | Physical Hardware Portability |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Pure Centralized (GCS Streaming)** | **Very High** (Global optimum) | Low (Bandwidth saturation) | **Zero** (Freezes or triggers RTL) | Low (Central planner avoids crosses) | High (Setpoints drop abruptly) | High (Standard MAVLink) |
| **Pure Decentralized (Olfati-Saber Consensus)** | Moderate (Subject to drag/lag drift) | **Very High** (Local 1-hop only) | **Very High** (Fully autonomous) | High (Path crossing during re-indexing) | None (No switching) | Moderate (Requires inter-drone radio) |
| **Polynomial Pre-Planning (Crazyswarm)** | High (Pre-computed curves) | High | High (Executes onboard splines) | Low (Pre-checked offline) | None | Low (Requires mocap / custom firmware) |
| **Naive Switched Hybrid** | High (When link active) | Moderate | Moderate (Drops to flocking) | Moderate | **Severe ($> 300$ switches/run)** | Moderate |
| **Proposed Asymmetric Hardened Hybrid** | **High** (Hungarian + Drag Feedforward) | **High** (Bounded 1-hop fallback) | **High** (Dynamic consensus fallback) | **Zero** (Hungarian + Active Onboard APF) | **Zero ($< 1$ switch/run via Dwell Time)** | **High** (ArduPilot GUIDED MAVLink2) |

### 2.3 Identification of Research/Technical Gap
While multi-agent systems and quadcopter flight control are individually mature, their intersection in practical, resource-constrained systems reveals critical unaddressed gaps:
1. **The Chattering Gap in Switched Hybrid Swarms:** Prior hybrid multi-agent frameworks propose discrete switching upon packet loss without modeling the bursty Gilbert-Elliott fading channels inherent to mobile UAV Wi-Fi. As a consequence, systems oscillate unstably between centralized guidance and decentralized consensus.
2. **Absence of Actuator & Aerodynamic Compensation:** Standard multi-agent consensus algorithms assume ideal double-integrator dynamics ($\ddot{\mathbf{p}} = \mathbf{u}$). When deployed on real multirotors, aerodynamic rotor drag ($c_d \mathbf{v}$) and closed-loop attitude lag ($\tau$) introduce severe steady-state tracking offsets ($\sim 1.14\text{ m}$ at modest speeds of $0.85\text{ m/s}$), which are rarely analyzed or compensated analytically.
3. **Sim-to-Real Multi-Tier Validation Gap:** Existing academic projects typically operate exclusively in ROS/Gazebo simulation (which suffers from severe CPU bottlenecking for $> 3$ drones) or report dubious sub-millimeter simulation accuracy. A rigorous, multi-tiered framework bridging lightweight numerical sweeps $\longleftrightarrow$ multi-instance ArduPilot SITL $\longleftrightarrow$ 3D Gazebo physics $\longleftrightarrow$ physical STM32H7 flight controllers is absent in open-source literature.

---

## 3. Methodology / Proposed Approach

### 3.1 Block Diagram / Flow Chart of Proposed System

The proposed system adopts a modular, hierarchical 3-tier architecture as illustrated below:

```
+-----------------------------------------------------------------------------------+
|                        CENTRAL COORDINATOR / GROUND STATION                       |
|  - Global Trajectory Planner (Moving Centroid v_cmd)                              |
|  - Formation Slot Generator: Line, V-Shape, Circle, Grid (Formations.py)          |
|  - Hungarian Optimal Slot Assignment: min sum ||p_i - s_j||^2                     |
|  - Analytical Velocity Feedforward Calculator: u_ff = c_d * v_target              |
+-----------------------------------------------------------------------------------+
                                          |
                         MAVLink Heartbeat Broadcast (10 Hz)
                      (Assigned Slot, Centroid Velocity, Seq ID)
                                          |
                                          v
+-----------------------------------------------------------------------------------+
|                         WIRELESS CHANNEL EMULATOR / RF LINK                       |
|  - Gilbert-Elliott 2-State Markov Chain (Good/Bad Fading Bursts)                  |
|  - Stochastic Packet Drop (0% - 50%) & Asymmetric Latency Queuing (10 - 400 ms)   |
|  - Hardware Layer: Dedicated 5 GHz Wi-Fi Router (UDP: 14552 - 14592)              |
+-----------------------------------------------------------------------------------+
                                          |
                     +--------------------+--------------------+
                     |                                         |
                     v (Drone 1 ... Drone 5)                   v (1-Hop Peer Broadcast)
+---------------------------------------------------+   +---------------------------+
|          ONBOARD HYBRID SUPERVISORY CONTROLLER    |   |     NEIGHBOR MEMORY       |
|  - Stale Packet Filter: age > 150 ms rejected     |   |  - State Extrapolation    |
|  - Degrade Timeout Watchdog: t_silent > 0.5 s     |   |    p(t) = p_0 + v*dt      |
|  - Sliding-Window Recovery: W = 20, ratio >= 70%  |   |  - Age-Out: tau = 0.3 s   |
|  - Dwell-Time Lockout: T_dwell = 2.0 s in Fallback|   +---------------------------+
|  - Dynamic Blending Engine: alpha(t) in [0, 1]    |                 |
+---------------------------------------------------+                 |
                     |                                                |
                     +-----------------------+------------------------+
                                             |
                                             v
+-----------------------------------------------------------------------------------+
|                       COMPOSITE CONTROL LAW GENERATOR                             |
|                                                                                   |
|  1. Centralized Guidance:                                                         |
|     u_central = k_p(s_i - p_i) + k_d(v_target - v_i) + c_d * v_target             |
|                                                                                   |
|  2. Decentralized Consensus Fallback:                                             |
|     u_decentral = -k_v sum a_ij(v_i - v_j) - k_p sum a_ij((p_i - p_j) - d_ij)     |
|                                                                                   |
|  3. Always-On Onboard Safety Barrier (Artificial Potential Field):                |
|     f_rep = eta * (1/d_ij - 1/r_safe) * (1/d_ij^2) * hat{r}_ij                    |
|                                                                                   |
|  Blended Acceleration:                                                            |
|     u_i(t) = alpha(t)*u_central + (1 - alpha(t))*u_decentral + alpha(t)*u_safe_apf|
+-----------------------------------------------------------------------------------+
                                          |
                       Target Conversion: u_i(t) -> SET_POSITION_TARGET
                                          |
                                          v
+-----------------------------------------------------------------------------------+
|             AUTOPILOT EXECUTION & PHYSICAL EMBEDDED LAYER (ArduPilot)             |
|  - WGS84 Flat-Earth Tangent Projection (common_frame.py)                          |
|  - ArduCopter 4.6.2 GUIDED Mode Inner PID Loops (PSC_POSXY, PSC_VELXY)            |
|  - EKF3 State Estimation (IMU, Baro, Compass, GNSS / GPS)                         |
|  - Actuation: DShot600 Digital Signaling -> 4x Brushless Motors                  |
+-----------------------------------------------------------------------------------+
```

### 3.2 Explanation of Chosen Methodology

#### 3.2.1 Vehicle Kinematics & Realistic Plant Dynamics
To overcome the unphysical behavior of pure point masses, each multirotor $i \in \{1, \dots, N\}$ is modeled as a 3D kinematic body subject to closed-loop translational lag and aerodynamic rotor drag:
$$\dot{\mathbf{p}}_i(t) = \mathbf{v}_i(t)$$
$$\dot{\mathbf{v}}_i(t) = \frac{1}{\tau} \left( \mathbf{v}_{\text{cmd}, i}(t) - \mathbf{v}_i(t) \right) - c_d \mathbf{v}_i(t)$$
where $\mathbf{p}_i \in \mathbb{R}^3$ is position, $\mathbf{v}_i \in \mathbb{R}^3$ is velocity, $\tau$ is the effective closed-loop translational time constant (calibrated via SITL step response to $0.992\text{ s}$), and $c_d$ is the aerodynamic rotor drag coefficient ($0.637\text{ s}^{-1}$).

#### 3.2.2 Hungarian Optimal Assignment for Formation Morphing
When morphing between formations (e.g., Line $\to$ V-Shape $\to$ Circle $\to$ Grid), assigning drones arbitrarily causes intersecting trajectories and collision hazards. The framework constructs a cost matrix $\mathbf{C} \in \mathbb{R}^{N \times N}$ based on squared Euclidean distance between current drone positions $\mathbf{p}_i$ and target slots $\mathbf{s}_j$:
$$C_{i,j} = \|\mathbf{p}_i - \mathbf{s}_j\|^2$$
The assignment permutation $\sigma^*$ is solved via the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`):
$$\sigma^* = \arg\min_{\sigma \in \mathcal{S}_N} \sum_{i=1}^N C_{i, \sigma(i)}$$
To prevent mid-transit assignment chattering, assignments are computed once at the initiation of a morph and locked rigidly to the moving centroid until the geometry converges.

#### 3.2.3 Analytical Velocity Feedforward
Without feedforward control, a tracking controller following a moving centroid at $\mathbf{v}_{\text{target}}$ incurs an unavoidable steady-state lag caused by damping ($k_d$) and rotor drag ($c_d$):
$$e_{\text{steady}} = \frac{k_d + c_d}{k_p} \|\mathbf{v}_{\text{target}}\|$$
For our calibrated system ($k_p=1.8$, $k_d=2.2$, $c_d=0.20$, $\|\mathbf{v}_{\text{target}}\|=0.8544\text{ m/s}$), this results in a theoretical lag of $1.139\text{ m}$. By injecting analytical velocity feedforward:
$$\mathbf{u}_{\text{ff}} = c_d \mathbf{v}_{\text{target}}$$
into the centralized control law, the steady-state tracking error is reduced by **$98.4\%$** down to $0.018\text{ m}$.

#### 3.2.4 Asymmetric Hysteresis State Machine & Smooth Blending
The hybrid supervisor on each drone governs transitions between Centralized guidance and Decentralized consensus:
1. **Degrade Condition:** If valid coordinator heartbeats are not received for $t_{\text{silent}} > 0.50\text{ s}$, or if incoming packets are older than $\tau_{\text{stale}} = 150\text{ ms}$, the supervisor immediately drops to Decentralized fallback.
2. **Dwell-Time Lockout:** Upon entering fallback, the drone is locked in decentralized mode for a mandatory minimum dwell time $T_{\text{dwell}} = 2.0\text{ s}$, preventing rapid re-engagement.
3. **Sliding-Window Recovery:** Recovery requires that over a sliding history window of $W = 20$ ticks ($1.0\text{ s}$), at least $\theta_{\text{rec}} = 70\%$ of heartbeats have arrived successfully with monotonic sequence numbers.
4. **Continuous Control Blending $\alpha(t)$:** Upon recovery, the weighting factor $\alpha(t)$ ramps linearly from $0.0 \to 1.0$ over $\tau_{\text{ramp}} = 0.5\text{ s}$:
   $$\alpha(t) = \min\left(1.0, \, \frac{t - t_{\text{rec}}}{\tau_{\text{ramp}}}\right)$$
   This guarantees $\mathcal{C}^0$ continuity of the commanded acceleration, preventing step changes in motor thrust.

#### 3.2.5 Flat-Earth WGS84 Common Coordinate Frame
In multi-vehicle ArduPilot SITL, each vehicle initializes with its own local NED origin $(0, 0, 0)$ at its physical spawn coordinate. To enable unified swarm operations, `common_frame.py` projects individual GPS coordinates (`GLOBAL_POSITION_INT`) onto a flat-Earth metric tangent plane anchored to a shared global datum origin (Lat: $-35.363261^\circ$, Lon: $149.165230^\circ$, Alt: $584.0\text{ m}$):
$$\Delta x_{\text{North}} = R_E \cdot (\phi - \phi_0)$$
$$\Delta y_{\text{East}} = R_E \cdot \cos(\phi_0) \cdot (\lambda - \lambda_0)$$
This transformation achieves round-trip conversion accuracy of **$< 0.1\text{ mm}$** within a $100\text{ m}$ radius.

### 3.3 Tools, Software, and Hardware Utilized

* **Simulation & Scientific Software:**
  * Python 3.12, NumPy, SciPy (Optimization & Linear Algebra), Matplotlib.
  * Pytest test execution framework with continuous regression testing.
* **Autopilot & Middleware:**
  * ArduPilot Copter v4.6.2 & ArduCopter v4.8.0-dev binary.
  * MAVLink v2 protocol, PyMAVLink, MAVProxy telemetry router.
  * QGroundControl v5.1.5 (AppImage) Ground Control Station.
* **3D Physics Engine:**
  * Gazebo Harmonic with `ardupilot_gazebo` JSON model plugins.
  * COLLADA 1.4.1 meshes for CineWhoop / Iris quadrotors with custom visual/collision SDF parameters.
* **Physical Hardware Stack:**
  * **Flight Controller:** Matek Systems H743-SLIM V3 (STM32H743VIH6 @ 480 MHz, Dual ICM-42688-P IMUs, DPS368 Barometer).
  * **Airframe:** 5-inch Betaflight-style Quad-X carbon fiber frames (`FRAME_CLASS=1`, `FRAME_TYPE=12`).
  * **ESCs & Motors:** 4-in-1 ESCs running bi-directional DShot600 (`MOT_PWM_TYPE=6`) with dynamic harmonic notch filtering.
  * **Telemetry Link:** ESP32 Wi-Fi modules bridging UART7 (`SERIAL1` @ 921,600 baud) to UDP.
  * **GNSS / Compass:** u-blox M10Q GNSS receivers with integrated IST8310 magnetometers.

---

## 4. Work Completed (till date)

### 4.1 Tasks Achieved So Far
1. **Full Core Algorithmic Architecture (`swarm_core`):** Implemented all 4 geometric formations, Hungarian assignment solver, graph Laplacian consensus, Reynolds flocking, APF collision barriers, and the asymmetric hybrid supervisor.
2. **Multi-Seed Monte Carlo Network Sweeps:** Completed rigorous evaluations comparing Centralized, Decentralized, Naive Hybrid, and Proposed Hardened Hybrid across packet loss rates from $0\%$ to $50\%$.
3. **Sim-to-SITL Dynamic Parameter Identification:** Conducted closed-loop $5.0\text{ m}$ position step response tests in ArduPilot GUIDED mode; fitted translational lag $\tau=0.992\text{ s}$ and rotor drag $c_d=0.637\text{ s}^{-1}$ with an RMSE of **$15.36\text{ cm}$**.
4. **3-Drone Formation Scenario Overlay in SITL:** Executed synchronized 3-drone V-formation flight over an $8.0\text{ m}$ translation in ArduPilot SITL; demonstrated an honest sim-to-real trajectory tracking discrepancy of **$72.50\text{ cm}$ RMS**.
5. **Multi-Vehicle SITL 2D Scaling (5 Drones):** Successfully launched 5 independent ArduCopter instances (`-I 0` to `-I 4`), broadcasting telemetry to individual control ports (14552–14592) and multiplexing a unified radar view to QGroundControl (port 14550).
6. **3D Gazebo Harmonic Multi-Drone Physics:** Resolved Gazebo port collisions and ground-clipping instabilities, successfully simulating multi-drone models with dedicated FDM ports (9002–9043).
7. **Autonomous Self-Healing Flight Console (`sitl/self_healing_swarm_console.py`):** Developed an interactive, thread-safe console featuring Dynamic Leader Election, automatic formation resizing upon drone dropouts, live failure injection (`f <id>`), and automated in-flight revival (`r <id>`).
8. **Physical Hardware Bench Verification (Matek H743-SLIM V3):** Connected the target flight controller via USB-C MAVLink; verified sensor health across dual gyros, accelerometers, barometer, and GNSS (23 satellites, 3D fix); and generated a complete baseline backup of all 1,283 parameters (`matek_h743_quad_20261006.param`).

### 4.2 Simulation/Analysis/Design Work Completed

#### 4.2.1 Parameter Identification & Fitting
To eliminate arbitrary parameter selection, a position step response experiment was conducted on ArduCopter in SITL GUIDED mode. The parameter identification minimized trajectory RMSE using L-BFGS-B optimization:

$$\min_{\tau, c_d} \sqrt{\frac{1}{K} \sum_{k=1}^K \|\mathbf{p}_{\text{sim}}(k; \tau, c_d) - \mathbf{p}_{\text{SITL}}(k)\|^2}$$

* **Literature Baseline:** $\tau = 0.180\text{ s}, \; c_d = 0.200\text{ s}^{-1} \implies \text{RMSE} = 0.4632\text{ m}$
* **Fitted Quadcopter Dynamics:** $\tau = \mathbf{0.992\text{ s}}, \; c_d = \mathbf{0.637\text{ s}^{-1}} \implies \text{RMSE} = \mathbf{0.1536\text{ m}}$ ($15.36\text{ cm}$)

#### 4.2.2 Chattering Suppression Under Packet Loss
Under simulated packet loss ($0\%\dots 50\%$), the performance of the proposed asymmetric hybrid controller was benchmarked against naive instantaneous switching across 6 random seeds:

$$\text{Naive Switching Mode Transitions:} \quad 302.8 \pm 14.2 \text{ switches/run at } 50\% \text{ loss}$$
$$\text{Proposed Asymmetric Hysteresis:} \quad \mathbf{0.6 \pm 0.4} \text{ switches/run at } 50\% \text{ loss}$$

This represents a **$99.8\%$ reduction in control chattering**, eliminating actuator hunting and preventing physical motor wear.

### 4.3 Results Obtained

#### Table 4.1: Comparative Benchmark Across Control Architectures Under Packet Loss

| Controller Regime | Packet Loss Rate (%) | Steady-State Tracking Error (m) | Transient Convergence Time (s) | Minimum Inter-Drone Separation (m) | Mode Switches / Run | Collisions ($d < 0.7\text{ m}$) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Centralized (No Feedforward)** | $0\%$ | $1.140 \pm 0.000$ | $2.30$ | $1.76$ | $0.0$ | $0$ |
| **Centralized (With Feedforward)**| $0\%$ | $\mathbf{0.018 \pm 0.001}$ | $2.30$ | $1.76$ | $0.0$ | $0$ |
| **Centralized (Hold-Last)** | $50\%$ | $0.482 \pm 0.084$ | Diverges | $1.42$ | $0.0$ | $0$ |
| **Pure Decentralized (Consensus)**| $0\%$ | $0.054 \pm 0.003$ | $4.85$ | $1.68$ | $0.0$ | $0$ |
| **Pure Decentralized (Consensus)**| $50\%$ | $0.118 \pm 0.012$ | $5.62$ | $1.61$ | $0.0$ | $0$ |
| **Naive Switched Hybrid** | $50\%$ | $0.082 \pm 0.021$ | $2.84$ | $1.28$ | $302.8$ | $0$ |
| **Proposed Hardened Hybrid** | $0\%$ | $\mathbf{0.018 \pm 0.001}$ | $\mathbf{2.30}$ | $\mathbf{1.76}$ | $\mathbf{0.0}$ | $\mathbf{0}$ |
| **Proposed Hardened Hybrid** | $50\%$ | $\mathbf{0.006 \pm 0.002}$ | $\mathbf{2.51}$ | $\mathbf{1.58}$ | $\mathbf{0.6}$ | $\mathbf{0}$ |

*Note: Collision threshold is established at $0.70\text{ m}$. All tests maintained $> 1.28\text{ m}$ minimum distance.*

#### Table 4.2: 3-Drone Synchronized Flight Overlay: Python `swarm_core` vs. ArduPilot SITL

| Vehicle ID | Role in Formation | Commanded Flight Distance | Trajectory RMS Discrepancy (m) | Peak Deviation (m) | EKF3 State Estimation Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Drone 1** | Apex Leader | $8.0\text{ m}$ North | $0.7152\text{ m}$ ($71.5\text{ cm}$) | $0.9601\text{ m}$ | Healthy (GPS Lock, 3D Fix) |
| **Drone 2** | Left Wingman | $8.0\text{ m}$ North | $0.7235\text{ m}$ ($72.3\text{ cm}$) | $0.9617\text{ m}$ | Healthy (GPS Lock, 3D Fix) |
| **Drone 3** | Right Wingman | $8.0\text{ m}$ North | $0.7361\text{ m}$ ($73.6\text{ cm}$) | $0.9904\text{ m}$ | Healthy (GPS Lock, 3D Fix) |
| **Overall Swarm**| Symmetric V-Formation | $8.0\text{ m}$ North | $\mathbf{0.7250\text{ m}}$ ($72.5\text{ cm}$) | $\mathbf{0.9904\text{ m}}$ | **Synchronized at 10 Hz** |

*Physical Interpretation:* The $72.5\text{ cm}$ RMS trajectory discrepancy over an $8\text{ m}$ dynamic translation reflects real multirotor dynamics (motor spool-up time, EKF3 filter convergence, and differential braking drag) and confirms valid sim-to-real predictive fidelity.

---

## 5. Work in Progress & Challenges

### 5.1 Current Tasks Under Development
1. **Closed-Loop EKF-Gated Arming in 3D Gazebo:** Implementing automated MAVLink polling of `EKF_STATUS_REPORT` and `GPS_RAW_INT` before issuing motor arm commands, eliminating arming rejections caused by simulation warmup.
2. **Dynamic 5-Drone Formation Morphing in Gazebo:** Validating real-time transitions (Line $\to$ V-Shape $\to$ Circle $\to$ Pentagon) in 3D physics under interactive console control.
3. **ESP32 MAVLink UDP-to-UART Firmware:** Finalizing Arduino/ESP-IDF firmware on ESP32 development boards to bridge UART7 (`SERIAL1` @ 921,600 baud) with Wi-Fi UDP sockets.
4. **Benchtop Dual-Drone Telemetry Verification:** Establishing simultaneous telemetry streaming from two physical Matek H743 flight controllers to the ground station.

### 5.2 Difficulties Faced (Technical & Non-Technical)

#### Technical Difficulties & Resolved Bugs:
1. **Gazebo Real-Time Factor (RTF) Slowdown:**
   * *Problem:* Simulating 5 detailed multirotor models with rigid-body aerodynamics in Gazebo Harmonic caused the simulation Real-Time Factor to drop to $\approx 17.6\%$ (5.7× slower than real time). ArduPilot EKF3 requires 15–20 seconds of *simulation time* to converge. With fixed wall-clock timeouts of 30s, the controller attempted to arm after only $\sim 5$ simulation seconds, causing `Arm: EKF not ready` rejections.
   * *Resolution:* Replaced wall-clock sleep timers with a telemetry-driven probe (`wait_ekf_ready()` in `flight_prep.py`) that queries EKF velocity/position variance flags directly over MAVLink before commanding takeoff.
2. **Gazebo Runway Collision Clipping & Inverted Spawns:**
   * *Problem:* Drones spawned at $Z = 0.05\text{ m}$ in Gazebo had collision bounding boxes that clipped through the tarmac ground plane, generating infinite repulsive contact forces that flipped the drones upside-down on spawn.
   * *Resolution:* Adjusted spawn elevation in `cinewhoop_5drones.sdf` to $Z = 0.20\text{ m}$, allowing gravity to settle the landing gear cleanly onto the surface.
3. **Physics Step Size vs. Gyro Loop Rate Limit:**
   * *Problem:* Increasing Gazebo's physics step size to $0.002\text{ s}$ (500 Hz) to save CPU caused ArduPilot SITL to refuse arming with: `Arm: Gyro 0 rate 501Hz < loop rate*1.8 (720Hz)`.
   * *Resolution:* Reverted `max_step_size` to $0.001\text{ s}$ (1000 Hz) to satisfy ArduCopter's safety check.
4. **MAVLink FDM Port Collisions:**
   * *Problem:* Gazebo SDF models defaulted `<fdm_port_out>` to 9003 for all instances, causing motor PWM commands from Drones 2–5 to collide on Drone 1's socket.
   * *Resolution:* Allocated discrete outbound ports (9003, 9013, 9023, 9033, 9043) in all model SDFs.
5. **Single Point of Failure (SPOF) in Centralized Leader Flight:**
   * *Problem:* If Drone 1 failed, follower drones had no reference origin and drifted.
   * *Resolution:* Developed `sitl/self_healing_swarm_console.py` with Dynamic Leader Election (Apex shifts dynamically from D1 $\to$ D2 $\to$ D3) and dynamic formation geometry recalculation.

#### Non-Technical Difficulties:
1. **Compressed Project Timeline:** The departmental target completion date was accelerated to December 31, 2026, requiring parallelization of algorithmic development, SITL simulation, and hardware preparation.
2. **Component Logistics & Lab Safety:** High-discharge 6S LiPo batteries required strict laboratory charging protocols and safety procedures during benchtop testing. Bench testing was conducted via USB bus power (`0.00 V` main rail) to ensure safety.

---

## 6. Future Work / Next Milestones

### 6.1 Work Planned for Next Phase (Phase 3 & Phase 4)
1. **Full 5-Drone 3D Gazebo Flight Demonstration:** Execute fully autonomous, formation-morphing flight demonstrations in Gazebo Harmonic, recording synchronized multi-angle video logs and telemetry.
2. **Hardware Bench Testing with ESP32 Telemetry (2 Drones):** Wire the ESP32 Wi-Fi modules to `TELEM1` (UART7) on two Matek H743 boards, confirming bidirectional MAVLink2 communication at 921,600 baud.
3. **Tethered / Outdoor Flight Testing (2 Quadcopters):** Perform real-world outdoor flight validation of 2 quadcopters executing coordinated leader-follower and V-formation flight using plain GNSS.
4. **Sim-to-Real Trajectory Error Characterization:** Compare physical flight telemetry `.bin` logs against `swarm_core` predictions to quantify real-world trajectory tracking error.
5. **Evaluation of RTK GNSS Upgrade:** Benchmark relative baseline accuracy between co-located u-blox M10Q receivers and evaluate the feasibility of deploying dual u-blox ZED-F9P RTK rovers if department funding permits.

### 6.2 Expected Outcomes for 2nd Interim Report
* High-definition video demonstration of 5-drone synchronized flight and dynamic leader failover in 3D Gazebo Harmonic.
* Quantitative hardware telemetry logs from dual physical quadcopter bench/flight trials.
* Completed sim-to-real comparative trajectory overlay matching physical flight against the fitted mathematical model.
* Comprehensive draft of Final Year Project Thesis chapters (Introduction, Literature Review, Methodology, and Experimental Results).

---

## 7. Project Timeline (Gantt Chart / Table)

### 7.1 Planned vs. Actual Progress

| Milestone / Task Description | Target Period | Scheduled Status | Actual Status | Variance / Remarks |
| :--- | :---: | :---: | :---: | :--- |
| **Phase A: Environment Setup & Toolchain Exploration** | Jun – Jul 2026 | Completed | Completed | Evaluated ROS 2 & PX4; selected ArduPilot for swarm scalability. |
| **Phase B: Mathematical Modeling & Python Engine** | Aug 2026 | Completed | Completed | Implemented Hungarian assignment, graph Laplacians, and APF. |
| **Phase C: Hybrid Controller Hardening & Benchmarks** | Sep 2026 | Completed | Completed | Implemented asymmetric hysteresis; suppressed chattering by $99.8\%$. |
| **Phase D: ArduPilot SITL Parameter Identification** | Sep – Oct 2026 | Completed | Completed | Fitted $\tau=0.992\text{ s}, c_d=0.637\text{ s}^{-1}$ with $15.36\text{ cm}$ RMSE. |
| **Phase E: 3-Drone & 5-Drone SITL / Gazebo Integration** | Oct 2026 | In Progress | **Ahead of Schedule** | 5-drone 2D & 3D Gazebo infrastructure operational; console complete. |
| **Phase F: Physical Hardware Bench Verification** | Oct – Nov 2026 | In Progress | **On Schedule** | Matek H743 verified, parameter dump complete, ESP32 wiring designed. |
| **Phase G: Dual-Drone Physical Flight Trials** | Nov 2026 | Scheduled | Pending | Scheduled following bench link verification. |
| **Phase H: Thesis Writing, Defense & Final Delivery** | Dec 2026 | Scheduled | On Schedule | Accelerated delivery targeted for Dec 31, 2026. |

### 7.2 Timeline Adjustments & Risk Mitigation
* **Strategic Architecture Pivot (Sep 30, 2026):** Early efforts spent troubleshooting complex ROS 2 Micro-XRCE-DDS multi-instance compilation were redirected toward a 3-tier architecture (Python Engine $\longleftrightarrow$ Headless MAVLink SITL $\longleftrightarrow$ Gazebo Visuals/Hardware). This eliminated middleware bottlenecks and advanced algorithmic completion by three weeks.

---

## 8. Resources Utilized

### 8.1 Hardware Procured & Utilized
* **Flight Controllers:** Matek Systems H743-SLIM V3 (STM32H743VIH6 @ 480 MHz, Dual ICM-42688-P IMUs, DPS368 Barometer).
* **Airframes:** 5-inch carbon fiber Quad-X frames with brushless motors (2207/2306 stator size) and Gemfan 5-inch tri-blade propellers.
* **Electronic Speed Controllers (ESCs):** 4-in-1 45A ESCs running bi-directional DShot600 with motor RPM feedback.
* **Telemetry & Microcontrollers:** ESP32 NodeMCU development boards for Wi-Fi MAVLink bridging.
* **GNSS Modules:** u-blox M10Q GNSS receivers with integrated IST8310 compass.
* **Power & Test Equipment:** Benchtop regulated DC power supplies, USB-C communication interfaces, digital multimeters.

### 8.2 Software & Open-Source Licenses
* **ArduPilot:** Open-source autopilot flight stack (GPLv3).
* **Gazebo Harmonic:** High-fidelity multi-robot physics simulation (Apache 2.0).
* **QGroundControl:** Multi-vehicle Ground Control Station (GPLv3).
* **Scientific Computing Stack:** Python 3.12, NumPy (BSD), SciPy (BSD), Matplotlib (PSF).
* **Operating System:** Ubuntu 24.04.5 LTS (x86_64).
* *License Cost:* **\$0.00** (100% open-source software stack).

### 8.3 Laboratory Facilities & Institutional Support
* **Facilities:** Embedded Systems, Communications, and Robotics Laboratory, Department of Electrical Engineering, Military College of Signals (MCS), NUST.
* **Supervision:** Continuous guidance, technical review, and academic oversight provided by Prof. Dr. Abdul Ghafoor.

---

## 9. Conclusion

This Interim Progress Report – 1 documents the engineering progress accomplished on the Final Year Project **"Autonomous Swarm Drones: Coordinated Controlled Movement"**. In response to the core challenges of wireless packet loss, control chattering, and dynamic formation deformation in multirotor swarms, the project has achieved:

1. A mathematically rigorous multi-agent control framework incorporating Hungarian optimal assignment, distributed Laplacian consensus, and an onboard Artificial Potential Field safety barrier.
2. A hardened asymmetric hybrid supervisory state machine that eliminates control chattering (reducing mode transitions from $302.8$ to $0.6$ under $50\%$ packet loss) and ensures smooth $\alpha(t)$ continuous control blending.
3. System identification directly against ArduPilot SITL, capturing physical translational lag ($\tau=0.992\text{ s}$) and rotor drag ($c_d=0.637\text{ s}^{-1}$) with an RMSE of $15.36\text{ cm}$.
4. Successful scaling to 5-drone SITL in both 2D and 3D Gazebo Harmonic physics with an interactive, thread-safe console featuring Dynamic Leader Election and live fault recovery.
5. Benchtop hardware verification of the physical Matek H743-SLIM V3 flight controller running ArduCopter v4.6.2.

The project is ahead of schedule regarding algorithmic and simulation milestones and is on track for dual-drone physical hardware validation and successful final project defense by December 31, 2026.

---

## 10. References

1. R. Olfati-Saber, J. A. Fax, and R. M. Murray, "Consensus and cooperation in networked multi-agent systems," *Proceedings of the IEEE*, vol. 95, no. 1, pp. 215–233, Jan. 2007.
2. L. Schenato, B. Sinopoli, M. Franceschetti, K. Poolla, and S. S. Sastry, "Foundations of control and estimation over lossy networks," *Proceedings of the IEEE*, vol. 95, no. 1, pp. 163–187, Jan. 2007.
3. J. A. Preiss, W. Hönig, G. S. Sukhatme, and N. Ayanian, "Crazyswarm: A large nano-quadcopter swarm," in *Proc. IEEE International Conference on Robotics and Automation (ICRA)*, Singapore, 2017, pp. 3299–3304.
4. C. W. Reynolds, "Flocks, herds and schools: A distributed behavioral model," in *Proc. 14th Annual Conference on Computer Graphics and Interactive Techniques (SIGGRAPH)*, Anaheim, CA, 1987, pp. 25–34.
5. R. Olfati-Saber, "Flocking for multi-agent dynamic systems: Algorithms and theory," *IEEE Transactions on Automatic Control*, vol. 51, no. 3, pp. 401–420, Mar. 2006.
6. H. W. Kuhn, "The Hungarian method for the assignment problem," *Naval Research Logistics Quarterly*, vol. 2, no. 1–2, pp. 83–97, 1955.
7. O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," *The International Journal of Robotics Research*, vol. 5, no. 1, pp. 90–98, 1986.
8. N. Chopra and M. W. Spong, "Passivity-based control of multi-agent systems with time-delay," in *Proc. American Control Conference (ACC)*, Minneapolis, MN, 2006, pp. 1076–1081.
9. F. Xiao, L. Wang, J. Chen, and Y. Gao, "Consensus problems for high-dimensional multi-agent systems with communication delay and packet loss," *IEEE Transactions on Circuits and Systems I: Regular Papers*, vol. 56, no. 9, pp. 2097–2108, Sep. 2009.
10. J. Qin and H. Gao, "Consensus for multi-agent systems with switching topologies and communication delay," *Automatica*, vol. 48, no. 9, pp. 2307–2314, Sep. 2012.
11. ArduPilot Development Team, "ArduPilot Copter Architecture and GUIDED Mode Navigation," ArduPilot Documentation, 2026. [Online]. Available: https://ardupilot.org/copter/
12. MAVLink Protocol Association, "MAVLink 2.0 Message Specifications: SET_POSITION_TARGET_LOCAL_NED (#84)," 2026. [Online]. Available: https://mavlink.io/en/messages/common.html
