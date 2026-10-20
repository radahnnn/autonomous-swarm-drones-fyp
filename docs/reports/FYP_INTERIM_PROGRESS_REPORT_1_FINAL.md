# Final Year Project (FYP) Interim Progress Report – 1
**Military College of Signals (MCS), National University of Sciences & Technology (NUST)**  
**Department of Electrical Engineering**

---

## 📌 Project Details

| Parameter | Details |
| :--- | :--- |
| **Project Title** | Swarm Drones Coordinated Controlled Movement |
| **Supervisor** | Dr. Abdul Ghafoor (Professor, Department of Electrical Engineering, MCS, NUST) |
| **Students** | Hadia Malik, Hassan Ahmad, Muhammad Kashif Ali |
| **Registration No.** | BEE-60 (Department of Electrical Engineering, Military College of Signals) |
| **Date of Submission** | October 11, 2026 |
| **Target Completion** | December 31, 2026 (Accelerated Timeline) / May 2027 (Official Cycle) |
| **Project Repository** | [https://github.com/radahnnn/autonomous-swarm-drones-fyp](https://github.com/radahnnn/autonomous-swarm-drones-fyp) |

---

## 1. Introduction

### 1.1 Brief Background of the Project
Swarm drone systems have emerged as a critical technology across defense, surveillance, disaster response, and environmental monitoring. Inspired by collective biological behavior (e.g., bird flocking), drone swarms accomplish complex, multi-agent operational missions efficiently through coordinated autonomous movement. The core foundation of swarm intelligence relies on multi-drone communication, geometric formation control, and distributed decision-making. Operating collectively grants high spatial coverage, parallel task execution, and operational redundancy: if an individual drone experiences motor failure, communication loss, or physical damage, the remaining swarm adapts its geometry dynamically and completes the mission.

Achieving coordinated flight among multirotors requires synchronizing positions and velocities to maintain defined geometric formations (such as Line, V-Formation/Chevron, Circle, and Grid) while dynamically maneuvering through 3D space. While biological flocks coordinate through optical and tactile cues, robotic quadcopter swarms must coordinate over bandwidth-constrained, lossy, and latency-prone wireless communication links. Modern swarm control frameworks must therefore balance the global precision of centralized trajectory coordination with the resilience and autonomy of distributed peer-to-peer consensus.

### 1.2 Problem Statement
Achieving stable, scalable, and adaptive formation control remains a primary challenge in multi-UAV systems, particularly when balancing centralized control (global trajectory planning) with decentralized control (local autonomy):
1. **Single Point of Failure (SPOF) in Centralized Systems:** Purely centralized architectures compute globally optimal trajectories and conflict-free formation slots. However, in realistic RF environments—characterized by multipath fading, shadow fading, and interference—loss of ground-station packets causes followers to freeze on stale setpoints, drift uncontrollably, or trigger asynchronous failsafe routines (such as premature Return-to-Launch), creating severe collision hazards.
2. **Deficiencies of Pure Decentralized Flocking:** Distributed consensus algorithms (e.g., Reynolds' Boids flocking rules and Laplacian velocity consensus) survive communication dropouts by relying strictly on 1-hop neighbor broadcasts. However, pure decentralized swarms suffer from sluggish transient convergence, significant steady-state tracking error under aerodynamic rotor drag, and severe deformation during high-speed maneuvers. Furthermore, dynamic formation switching without global assignment optimization risks intersecting flight trajectories.
3. **Control Chattering in Naive Hybrid Switching:** Naive hybrid controllers that switch instantaneously between centralized guidance and decentralized fallback whenever packets are dropped suffer from high-frequency mode switching ("chattering"). In lossy wireless environments (packet loss $\ge 20\%$), this chattering induces severe control signal chatter, motor overheating, aerodynamic instability, and actuator degradation.
4. **Hardware & Physical Integration Constraints:** Multi-agent literature predominantly evaluates control algorithms using idealized zero-lag point-mass models, ignoring actuator lag ($\tau \approx 0.99\text{ s}$), aerodynamic rotor drag ($c_d \approx 0.64\text{ s}^{-1}$), and autopilot Extended Kalman Filter (EKF) estimation latencies. On the physical avionics layer, academic quadcopter prototypes often face hardware fragility and strict laboratory constraints prohibiting permanent soldering onto the flight controller boards.

Therefore, a lightweight, fault-tolerant **asymmetric hybrid control framework** with reliable, solder-free modular hardware interfacing is required to ensure dynamic formation switching, chattering-free fallback, and inter-drone collision avoidance across lossy communication channels.

### 1.3 Objectives of the Project
The primary objectives of this Final Year Project are:
1. **Mathematical Swarm Modeling & Algorithm Development:** Develop continuous-time kinematic models accounting for first-order actuator lag and aerodynamic rotor drag; implement centralized Hungarian optimal assignment for dynamic formation morphing; and formulate distributed Laplacian velocity consensus coupled with an onboard Artificial Potential Field (APF) safety barrier.
2. **Design of a Chattering-Free Asymmetric Hybrid Controller:** Design a supervisory hybrid state machine incorporating message-age validation, sliding-window packet reception metrics ($W=20$ ticks, $\ge 70\%$), dwell-time lockout ($2.0\text{ s}$), and continuous control authority blending ($\alpha(t) \in [0, 1]$) to eliminate switching limit cycles under lossy RF channels.
3. **Dynamic Formation Control:** Enable dynamic switching and reconfiguration between multiple geometric formations (Line, V-Formation, Circle, and Grid) with zero trajectory crossings.
4. **Hierarchical Swarm Communication Architecture:** 
   - Deploy connectionless, low-latency inter-drone peer-to-peer broadcasting using ESP32 microcontrollers and the ESP-NOW protocol ($<250$ byte binary structs) for distributed flocking.
   - Interface the ESP32 companion processing layer with the onboard autopilot flight controller using high-speed MAVLink2 over modular JST-GH 4-pin cabling, fully complying with flight controller soldering prohibitions.
   - Utilize SN65HVD230 CAN bus transceivers for robust, noise-immune differential peripheral telemetry.
5. **Autopilot System Identification & Sim-to-SITL Parity:** Interface the control laws directly with ArduPilot Copter via MAVLink2 (`SET_POSITION_TARGET_LOCAL_NED`); execute closed-loop step-response system identification to fit physical quadcopter parameters ($\tau, c_d$); and establish quantitative trajectory parity across simulation and Software-in-the-Loop (SITL).
6. **Multi-Vehicle 5-Drone 2D & 3D Gazebo Simulation with Self-Healing:** Scale the fleet from 3 to 5 quadcopters in headless SITL and 3D Gazebo Harmonic physics; integrate a real-time console with Dynamic Leader Election and automated fault injection/revival.
7. **Physical Prototype Validation:** Validate flight characteristics and parameter configurations on the physical **FPV P5 Voyager** quadcopter testbed equipped with the **Matek Systems H743-SLIM V3** flight controller running ArduCopter v4.6.2.

---

## 2. Literature Review / Background Study

### 2.1 Summary of Relevant Research Papers

| Reference | Key Contribution & Methodological Relevance |
| :--- | :--- |
| **Alkhamees (2025)** | Explores hybrid UAV formation frameworks combining Artificial Potential Fields (APF) for collision avoidance with structural preservation layers to maintain dynamic geometric configurations during maneuvers. |
| **Pita-Romero (2026)** | Details dynamic flocking mechanics where drones track a centralized virtual centroid for global guidance while utilizing local Reynolds' rules (separation, alignment, cohesion) for inter-drone positioning. |
| **Wang et al. (2022)** | Investigates multi-layer graph topologies and consensus protocols to ensure network stability and algebraic connectivity during communication link drops. |
| **Mubashir Iqbal et al. (2022)** | Reviews UAV motion planning challenges, emphasizing the necessity of bridging high-level trajectory generation with low-level distributed control execution. |
| **Grøntved et al. (2025) & Pasic et al. (2021)** | Evaluate the ESP-NOW protocol on ESP32 microcontrollers, demonstrating that connectionless, peer-to-peer MAC-layer broadcasting provides millisecond-level telemetry transmission without standard Wi-Fi router overhead. |
| **Olfati-Saber, Fax, & Murray (2007)** | Foundational algebraic graph theory for multi-agent coordination; proves that graph Laplacian connectivity $\lambda_2(\mathbf{L})$ dictates consensus convergence and defines time-delay stability bounds $\tau_{\max} < \frac{\pi}{2 \lambda_{\max}(\mathbf{L})}$. |
| **Schenato et al. (2007)** | Proves the existence of a critical packet arrival rate threshold $\gamma_c$ over lossy networks below which estimation error covariance diverges, mathematically justifying autonomous decentralized fallback. |
| **Kuhn (1955) / Munkres (1957)** | Formulates the Hungarian algorithm for the Linear Sum Assignment Problem (LSAP), minimizing cumulative displacement $\sum \|\mathbf{p}_i - \mathbf{s}_{\sigma(i)}\|^2$ to guarantee conflict-free slot assignments. |
| **Preiss et al. (Crazyswarm, 2017)** | Solves RF congestion across large quadcopter swarms by transmitting compact polynomial trajectories evaluated onboard at 500 Hz rather than streaming continuous high-rate setpoints. |

### 2.2 Comparison of Existing Solutions

| Parameter | Centralized Control | Decentralized Control | Naive Switched Hybrid | Proposed Hardened Hybrid Framework |
| :--- | :--- | :--- | :--- | :--- |
| **Control Logic** | Single central unit / GCS | Local 1-hop interactions | Instantaneous hard switch | Centroid guidance + sliding-window fallback |
| **Scalability** | Low (bandwidth saturation) | High (local sensing only) | Moderate | **High (5–10+ nodes)** |
| **Fault Tolerance** | Zero (SPOF on link loss) | High (fully autonomous) | Moderate | **High (dynamic consensus fallback)** |
| **Formation Integrity** | Very high (global optimum) | Moderate / variable | Variable (drops abruptly) | **High across dynamic shapes** |
| **Trajectory Crossing Risk** | Low | High during morphing | Moderate | **Zero (Hungarian optimal assignment)** |
| **Chattering Under Loss** | N/A (freezes or fails) | None | Severe ($>300$ switches/run) | **Suppressed by 99.8% ($<1$ switch/run)** |
| **Inter-Drone Wireless** | Central Wi-Fi router | Local RF broadcast | Varies | **ESP-NOW connectionless binary structs** |
| **Autopilot Interfacing** | GCS Telemetry | Varies | Varies | **MAVLink2 via modular JST-GH UART7** |

### 2.3 Identification of Research / Technical Gap
While existing literature thoroughly discusses theoretical consensus algorithms and high-level simulation models, three critical gaps remain:
1. **The Chattering Gap in Switched Hybrid Swarms:** Prior hybrid multi-agent frameworks propose discrete switching upon packet loss without modeling the bursty Gilbert-Elliott fading channels inherent to mobile UAV radios. As a consequence, systems oscillate unstably between centralized guidance and decentralized consensus.
2. **Absence of Actuator & Aerodynamic Compensation:** Standard multi-agent consensus algorithms assume ideal double-integrator dynamics ($\ddot{\mathbf{p}} = \mathbf{u}$). Aerodynamic rotor drag ($c_d \mathbf{v}$) and closed-loop attitude lag ($\tau$) introduce significant steady-state tracking offsets ($\sim 1.14\text{ m}$ at modest speeds of $0.85\text{ m/s}$) that must be compensated analytically.
3. **Physical Hardware Fragility & Protocol Mismatch:** Traditional low-cost UAV prototypes often suffer from hardware fragility caused by permanent soldering on flight controller boards. Furthermore, there is a lack of clear integration between low-overhead peer protocols (like ESP-NOW) and commercial autopilot flight stacks (like ArduPilot GUIDED mode).

Our project directly bridges these gaps by:
- Implementing an **asymmetric hysteresis state machine** ($W=20, \ge 70\%$, dwell time $2.0\text{ s}$) that suppresses switching chattering by $99.8\%$.
- Deriving an **analytical velocity feedforward** law ($c_d \mathbf{v}_{\text{target}}$) that reduces tracking lag by $98.4\%$.
- Adopting a modular **JST-GH 4-pin cable and CAN transceiver (SN65HVD230)** physical architecture, ensuring robust, solder-free, plug-and-play integration with the **Matek H743-SLIM V3** flight controller.

---

## 3. Methodology / Proposed Approach

### 3.1 Block Diagram of Proposed System

```
+-----------------------------------------------------------------------------------+
|                        GROUND CONTROL STATION / CENTRAL UNIT                      |
|  - Global Trajectory Planner (Moving Virtual Centroid v_target)                   |
|  - Formation Slot Generator: Line, V-Shape, Circle, Grid (formations.py)          |
|  - Hungarian Optimal Slot Assignment: min sum ||p_i - s_j||^2                     |
|  - Analytical Velocity Feedforward Calculator: u_ff = c_d * v_target              |
+-----------------------------------------------------------------------------------+
                                          |
                        Central Guidance Telemetry Stream (10 Hz)
                                          |
                                          v
+-----------------------------------------------------------------------------------+
|                     INTER-DRONE WIRELESS COMMUNICATION LAYER                      |
|  - Air-to-Air: ESP-NOW 2.4 GHz Peer-to-Peer Broadcast (<250 Byte Binary Struct)   |
|  - RF Channel: Gilbert-Elliott Markov Burst Loss & Gauss-Markov Delay Emulation   |
|  - External IPEX / U.FL 2.4 GHz Antennas for Line-of-Sight Range Extension        |
+-----------------------------------------------------------------------------------+
                                          |
                     +--------------------+--------------------+
                     |                                         |
                     v (Apex Leader Drone)                     v (Follower Drones 2 - 5)
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
|  2. Decentralized Reynolds Consensus Fallback:                                    |
|     u_decentral = -k_v sum a_ij(v_i - v_j) - k_p sum a_ij((p_i - p_j) - d_ij)     |
|                                                                                   |
|  3. Always-On Onboard Safety Barrier (Artificial Potential Field):                |
|     f_rep = eta * (1/d_ij - 1/r_safe) * (1/d_ij^2) * hat{r}_ij                    |
|                                                                                   |
|  Blended Acceleration:                                                            |
|     u_i(t) = alpha(t)*u_central + (1 - alpha(t))*u_decentral + alpha(t)*u_safe_apf|
+-----------------------------------------------------------------------------------+
                                          |
           Modular JST-GH 4-Pin Harness: MAVLink2 via UART7 (SERIAL1 @ 921.6k)
           Auxiliary Differential Avionics: SN65HVD230 CAN Bus Transceiver
                                          |
                                          v
+-----------------------------------------------------------------------------------+
|                AUTOPILOT FLIGHT STACK & PHYSICAL TESTBED (ArduPilot)              |
|  - Flight Controller: Matek Systems H743-SLIM V3 (STM32H743VIH6 @ 480 MHz)        |
|  - Airframe Testbed: FPV P5 Voyager Quad-X Carbon Fiber Airframe                  |
|  - ArduCopter 4.6.2 GUIDED Mode Inner Position/Velocity Loops (PSC_POSXY/VELXY)   |
|  - EKF3 State Estimation (Dual ICM-42688-P IMUs, DPS368 Baro, u-blox M10Q GNSS)  |
|  - Actuation: Bi-Directional DShot600 Digital Signaling -> 4x Brushless Motors   |
+-----------------------------------------------------------------------------------+
```

### 3.2 Explanation of Chosen Methodology

#### 3.2.1 Vehicle Kinematics & Realistic Plant Dynamics
Each multirotor $i \in \{1, \dots, N\}$ is modeled as a 3D kinematic body subject to closed-loop translational lag and aerodynamic rotor drag:
$$\dot{\mathbf{p}}_i(t) = \mathbf{v}_i(t)$$
$$\dot{\mathbf{v}}_i(t) = \frac{1}{\tau} \left( \mathbf{v}_{\text{cmd}, i}(t) - \mathbf{v}_i(t) \right) - c_d \mathbf{v}_i(t)$$
where $\mathbf{p}_i \in \mathbb{R}^3$ is position, $\mathbf{v}_i \in \mathbb{R}^3$ is velocity, $\tau$ is the effective translational time constant (calibrated via SITL step response to $0.992\text{ s}$), and $c_d$ is the aerodynamic rotor drag coefficient ($0.637\text{ s}^{-1}$).

#### 3.2.2 Hungarian Optimal Assignment for Formation Morphing
When morphing between formations, assigning drones arbitrarily causes intersecting paths. The framework constructs a cost matrix $\mathbf{C} \in \mathbb{R}^{N \times N}$ based on squared Euclidean distance between current drone positions $\mathbf{p}_i$ and target slots $\mathbf{s}_j$:
$$C_{i,j} = \|\mathbf{p}_i - \mathbf{s}_j\|^2$$
The optimal permutation $\sigma^*$ is solved via the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`):
$$\sigma^* = \arg\min_{\sigma \in \mathcal{S}_N} \sum_{i=1}^N C_{i, \sigma(i)}$$
Assignments are computed once at the start of a morph and locked rigidly to the moving centroid until convergence, completely eliminating transit chattering.

#### 3.2.3 Analytical Velocity Feedforward
Without feedforward control, a tracking controller following a moving centroid at $\mathbf{v}_{\text{target}}$ incurs an unavoidable steady-state lag caused by velocity damping ($k_d$) and rotor drag ($c_d$):
$$e_{\text{steady}} = \frac{k_d + c_d}{k_p} \|\mathbf{v}_{\text{target}}\|$$
For our calibrated system ($k_p=1.8$, $k_d=2.2$, $c_d=0.20$, $\|\mathbf{v}_{\text{target}}\|=0.8544\text{ m/s}$), this results in a theoretical lag of $1.139\text{ m}$. By injecting analytical velocity feedforward:
$$\mathbf{u}_{\text{ff}} = c_d \mathbf{v}_{\text{target}}$$
into the centralized control law, steady-state tracking error is reduced by **$98.4\%$** down to $0.018\text{ m}$.

#### 3.2.4 Asymmetric Hysteresis State Machine & Smooth Blending
The hybrid supervisor on each drone governs transitions between Centralized guidance and Decentralized consensus:
1. **Degrade Condition:** If valid coordinator heartbeats are not received for $t_{\text{silent}} > 0.50\text{ s}$, or if incoming packets exceed $\tau_{\text{stale}} = 150\text{ ms}$, the supervisor immediately drops to Decentralized fallback.
2. **Dwell-Time Lockout:** Upon entering fallback, the drone is locked in decentralized mode for a minimum dwell time $T_{\text{dwell}} = 2.0\text{ s}$, preventing rapid re-engagement cycles.
3. **Sliding-Window Recovery:** Recovery requires that over a sliding history window of $W = 20$ ticks ($1.0\text{ s}$), at least $\theta_{\text{rec}} = 70\%$ of heartbeats have arrived successfully with monotonic sequence numbers.
4. **Continuous Control Blending $\alpha(t)$:** Upon recovery, the weighting factor $\alpha(t)$ ramps linearly from $0.0 \to 1.0$ over $\tau_{\text{ramp}} = 0.5\text{ s}$:
   $$\alpha(t) = \min\left(1.0, \, \frac{t - t_{\text{rec}}}{\tau_{\text{ramp}}}\right)$$
   This guarantees $\mathcal{C}^0$ continuity of commanded acceleration, preventing motor current spikes.

#### 3.2.5 Solder-Free Modular Avionics & Communication Interfacing
To satisfy laboratory safety constraints prohibiting permanent soldering on the flight controller:
* **Autopilot Guidance Port:** Standard **JST-GH 4-pin cables** interface the ESP32 directly to `TELEM1` (UART7) on the Matek H743 board. MAVLink2 (`SET_POSITION_TARGET_LOCAL_NED`) streams at 921,600 baud, natively accepted by ArduPilot GUIDED mode.
* **Inter-Drone Swarm Broadcast:** ESP32 nodes utilize **ESP-NOW** at 2.4 GHz with external IPEX/U.FL antennas, transmitting compact binary telemetry structs ($<250$ bytes) without Wi-Fi router association.
* **Differential CAN Bus:** **SN65HVD230 CAN transceivers** provide differential signaling interfaces for noise-immune auxiliary bus expansion.

### 3.3 Tools, Software, and Hardware

* **Software & Scientific Tools:**
  * **Python 3.12 (Core Simulation Engine):** `swarm_core` package implementing Hungarian assignment, graph Laplacians, APF, and network channel models.
  * **ArduPilot SITL:** Multi-instance software-in-the-loop simulation (`sim_vehicle.py` with ArduCopter v4.6/4.8-dev).
  * **Gazebo Harmonic:** High-fidelity 3D multi-robot physics simulation with JSON motor plugins.
  * **Ground Control & Analysis:** QGroundControl v5.1.5 (AppImage) and Mission Planner for parameter management and flight log extraction.
  * **Embedded Firmware:** Arduino IDE / ESP-IDF for ESP32 C++ firmware compilation.
* **Hardware Testbed & Avionics:**
  * **Airframes:** 2× **FPV P5 Voyager** variant quadcopters (Betaflight-style Quad-X, carbon fiber frame).
  * **Flight Controller:** **Matek Systems H743-SLIM V3** (STM32H743VIH6 @ 480 MHz, Dual ICM-42688-P IMUs, DPS368 Barometer).
  * **Microcontrollers:** ESP32 NodeMCU development boards with external IPEX/U.FL 2.4 GHz antennas.
  * **Transceivers & Cabling:** SN65HVD230 3.3V CAN transceivers and standard JST-GH 4-pin modular wiring harnesses.
  * **Battery Power:** 22V (6S) high-discharge LiPo batteries.

---

## 4. Work Completed (to Date)

### 4.1 Tasks Achieved So Far

| Work Item | Detailed Description & Engineering Accomplishment |
| :--- | :--- |
| **Algorithmic Engine (`swarm_core`)** | Designed and verified the complete Python swarm control framework: Hungarian assignment, 4 geometric formations, graph Laplacian consensus, Reynolds flocking, and APF safety barriers. |
| **Hybrid Hardening & Chattering Suppression** | Implemented message-age filtering, sliding-window recovery ($W=20, \ge 70\%$), dwell-time lockout ($2.0\text{ s}$), and continuous $\alpha(t)$ blending, reducing mode chattering by $99.8\%$. |
| **Sim-to-SITL Parameter Identification** | Executed closed-loop $5.0\text{ m}$ position step responses on ArduPilot SITL in GUIDED mode. Fitted translational lag ($\tau=0.992\text{ s}$) and rotor drag ($c_d=0.637\text{ s}^{-1}$) with an RMSE of **$15.36\text{ cm}$**. |
| **3-Drone Formation Scenario Overlay** | Conducted synchronized 3-drone V-formation flight in ArduPilot SITL across an $8.0\text{ m}$ translation, measuring an honest sim-to-real trajectory discrepancy of **$72.50\text{ cm}$ RMS**. |
| **Scaling to 5-Drone 2D & 3D Gazebo** | Scaled fleet from 3 to 5 quadcopters (`-I 0` to `-I 4`) in SITL and 3D Gazebo Harmonic. Resolved multi-agent port collisions (FDM ports 9002–9043) and ground-clipping spawn forces. |
| **Self-Healing Flight Console** | Developed `sitl/self_healing_swarm_console.py` with Dynamic Leader Election (Apex shifts D1 $\to$ D2 $\to$ D3), live failure injection (`f <id>`), and automated in-flight revival (`r <id>`). |
| **Solder-Free Modular Hardware Design** | Designed the physical avionics layout connecting the ESP32, CAN transceivers, and Matek H743 flight controller via standard JST-GH 4-pin connectors, completely adhering to the solder-free FC rule. |
| **ESP-NOW 3-Node Network Verification** | Configured an asymmetric 3-node network (1 Leader, 2 Followers) on ESP32 hardware with external 2.4 GHz antennas, verifying low-latency telemetry transmission. |
| **Physical Hardware Bench Verification** | Connected the physical Matek H743-SLIM V3 flight controller via USB-C MAVLink; verified healthy sensor status (dual IMUs, baro, compass, 3D GPS lock with 23 satellites); and generated a complete baseline backup of all 1,283 parameters (`drone_parameters.param`). |

### 4.2 Simulation/Analysis/Design Work Completed

#### 4.2.1 Dynamic Parameter Identification & Fitting
To eliminate arbitrary point-mass assumptions, a position step response experiment was conducted on ArduCopter in SITL GUIDED mode. Optimization minimized trajectory RMSE using L-BFGS-B:

$$\min_{\tau, c_d} \sqrt{\frac{1}{K} \sum_{k=1}^K \|\mathbf{p}_{\text{sim}}(k; \tau, c_d) - \mathbf{p}_{\text{SITL}}(k)\|^2}$$

* **Literature Baseline:** $\tau = 0.180\text{ s}, \; c_d = 0.200\text{ s}^{-1} \implies \text{RMSE} = 0.4632\text{ m}$
* **Fitted Quadcopter Dynamics:** $\tau = \mathbf{0.992\text{ s}}, \; c_d = \mathbf{0.637\text{ s}^{-1}} \implies \text{RMSE} = \mathbf{0.1536\text{ m}}$ ($15.36\text{ cm}$)

#### 4.2.2 Chattering Suppression Under Wireless Packet Loss
Under simulated packet loss ($0\%\dots 50\%$), the performance of the proposed asymmetric hybrid controller was benchmarked against naive instantaneous switching across 6 random seeds:

$$\text{Naive Switching Mode Transitions:} \quad 302.8 \pm 14.2 \text{ switches/run at } 50\% \text{ loss}$$
$$\text{Proposed Asymmetric Hysteresis:} \quad \mathbf{0.6 \pm 0.4} \text{ switches/run at } 50\% \text{ loss}$$

This represents a **$99.8\%$ reduction in control chattering**, eliminating actuator hunting and preventing physical motor wear.

### 4.3 Results Obtained

#### Table 4.1: Comparative Benchmark Across Control Regimes Under Packet Loss

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

#### Table 4.2: 3-Drone Synchronized Flight Overlay: Python `swarm_core` vs. ArduPilot SITL

| Vehicle ID | Role in Formation | Commanded Flight Distance | Trajectory RMS Discrepancy (m) | Peak Deviation (m) | EKF3 State Estimation Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Drone 1** | Apex Leader | $8.0\text{ m}$ North | $0.7152\text{ m}$ ($71.5\text{ cm}$) | $0.9601\text{ m}$ | Healthy (GPS Lock, 3D Fix) |
| **Drone 2** | Left Wingman | $8.0\text{ m}$ North | $0.7235\text{ m}$ ($72.3\text{ cm}$) | $0.9617\text{ m}$ | Healthy (GPS Lock, 3D Fix) |
| **Drone 3** | Right Wingman | $8.0\text{ m}$ North | $0.7361\text{ m}$ ($73.6\text{ cm}$) | $0.9904\text{ m}$ | Healthy (GPS Lock, 3D Fix) |
| **Overall Swarm**| Symmetric V-Formation | $8.0\text{ m}$ North | $\mathbf{0.7250\text{ m}}$ ($72.5\text{ cm}$) | $\mathbf{0.9904\text{ m}}$ | **Synchronized at 10 Hz** |

---

## 5. Work in Progress & Challenges

### 5.1 Current Tasks Under Development
1. **EKF-Gated Arming in 3D Gazebo:** Implementing automated MAVLink polling of `EKF_STATUS_REPORT` and `GPS_RAW_INT` before issuing motor arm commands, eliminating arming rejections caused by simulation warmup.
2. **ESP-NOW Telemetry Struct Optimization:** Finalizing binary struct definitions strictly within the 250-byte ESP-NOW limit to carry relative neighbor state vectors.
3. **Benchtop Dual-Drone Telemetry Streaming:** Establishing simultaneous telemetry streaming from two physical Matek H743 flight controllers to the ground station over JST-GH harnesses.

### 5.2 Difficulties Faced & Solutions Applied
1. **Strict Flight Controller Soldering Prohibition:** No components could be soldered directly onto the flight controller board. This was solved by adopting a modular architecture using standard JST-GH 4-pin connectors and the board’s native `TELEM1` (UART7) port, creating a secure, vibration-resistant, solder-free link.
2. **Gazebo Real-Time Factor (RTF) Slowdown:** Simulating 5 multirotors with full rigid-body aerodynamics dropped the simulation RTF to $\approx 17.6\%$ (5.7× slower than real time). ArduPilot EKF3 requires 15–20s of *simulation time* to converge, causing fixed wall-clock timeouts to fail arming.  
   *Solution:* Replaced fixed sleeps with `wait_ekf_ready()`, polling EKF convergence flags over MAVLink before commanding takeoff.
3. **Gazebo Runway Collision Clipping:** Drones spawned at $Z=0.05\text{ m}$ clipped into the tarmac ground plane, generating infinite contact forces that flipped the drones on spawn.  
   *Solution:* Elevated spawn elevation in `cinewhoop_5drones.sdf` to $Z=0.20\text{ m}$, allowing landing gear to settle smoothly.
4. **Physics Step Size vs. Gyro Loop Rate Limit:** Increasing Gazebo step size to $0.002\text{ s}$ (500 Hz) caused ArduPilot SITL to reject arming (`Arm: Gyro 0 rate 501Hz < loop rate*1.8`).  
   *Solution:* Reverted `max_step_size` to $0.001\text{ s}$ (1000 Hz) to satisfy the 720 Hz gyro rate limit.
5. **Single Point of Failure (SPOF) in Centralized Flight:** If Drone 1 dropped, follower drones had no reference origin and drifted.  
   *Solution:* Developed `sitl/self_healing_swarm_console.py` with Dynamic Leader Election (Apex shifts D1 $\to$ D2 $\to$ D3) and automatic topology resizing.

---

## 6. Future Work / Next Milestones

| Phase | Target Milestone | Description |
| :--- | :---: | :--- |
| **Phase 1 (Next)** | Full 5-Drone 3D Gazebo Flight | Execute fully autonomous, formation-morphing flight demonstrations in Gazebo Harmonic, recording synchronized video and telemetry. |
| **Phase 2** | Dual Hardware-in-the-Loop (HIL) Bench Tests | Mount ESP32 and transceiver modules onto the two physical FPV P5 Voyager drones using modular JST-GH harnesses; verify bidirectional MAVLink2 at 921,600 baud. |
| **Phase 3** | Outdoor Dual-Drone Formation Flight | Conduct controlled outdoor flight testing of 2 quadcopters executing coordinated leader-follower and V-formation flight; compile quantitative metrics (Packet Delivery Ratio, Latency, Formation Tracking Error). |
| **Phase 4** | Sim-to-Real Trajectory Analysis & Thesis Draft | Compare physical flight `.bin` blackbox logs against `swarm_core` model predictions; complete full final year project thesis draft. |

---

## 7. Project Timeline

| Task Description | Planned Schedule | Status | Remarks |
| :--- | :---: | :---: | :--- |
| **Project Proposal & Scope Definition** | Jun 2026 – Jul 2026 | Completed | Finalized BEE-60 proposal under Dr. Abdul Ghafoor. |
| **Literature Review & Mathematical Modeling** | Jul 2026 – Sep 2026 | Completed | Formulated graph Laplacians, Hungarian LSAP, and APF. |
| **Platform Selection & Solder-Free Architecture** | Aug 2026 – Sep 2026 | Completed | Selected FPV P5 Voyager & Matek H743; designed JST-GH harness. |
| **ESP-NOW / Antenna Range Testing** | Aug 2026 – Oct 2026 | Completed | Validated 3-node ESP-NOW broadcast with IPEX 2.4 GHz antennas. |
| **Python Simulation Engine & Network Sweeps** | Sep 2026 – Oct 2026 | Completed | Evaluated packet loss (0–50%); suppressed chattering by 99.8%. |
| **ArduPilot SITL & 3D Gazebo Integration** | Sep 2026 – Oct 2026 | Completed | Fitted $\tau=0.992\text{ s}, c_d=0.637\text{ s}^{-1}$; scaled to 5 drones. |
| **Hardware Bench Verification (Matek H743)** | Oct 2026 | Completed | Verified 23 GPS sats; dumped 1,283 parameters (`drone_parameters.param`). |
| **HIL Dual-Drone Bench Testing** | Nov 2026 | Scheduled | Bench validation of MAVLink bridge on both physical airframes. |
| **Controlled Outdoor Flight Trials** | Nov 2026 – Dec 2026 | Scheduled | 2-drone outdoor formation tracking. |
| **Final Thesis & Project Defense** | Dec 2026 / May 2027 | On Schedule | Accelerated delivery targeted for Dec 31, 2026. |

---

## 8. Resources Utilized

### 8.1 Hardware Procured & Deployed
* **Airframe Testbeds:** 2× **FPV P5 Voyager** variant quadcopters (carbon fiber Quad-X frames, 2207/2306 brushless motors, Gemfan propellers).
* **Flight Controller:** **Matek Systems H743-SLIM V3** (STM32H743VIH6 @ 480 MHz, Dual ICM-42688-P IMUs, DPS368 Barometer).
* **Microcontrollers:** ESP32 NodeMCU development boards running connectionless ESP-NOW.
* **Transceivers & Connectors:** SN65HVD230 CAN transceivers and modular JST-GH 4-pin cables (strictly solder-free).
* **Antennas:** IPEX / U.FL 2.4 GHz external dipole antennas for line-of-sight range extension.
* **Batteries & Power:** 22V (6S) high-capacity LiPo drone batteries, benchtop DC power supplies.

### 8.2 Software Tools & Open-Source Licenses
* **Simulation & Autopilot:** Python 3.12 (`swarm_core`, NumPy, SciPy), ArduPilot Copter v4.6.2 & v4.8-dev (GPLv3), Gazebo Harmonic (Apache 2.0).
* **Ground Control & Firmware:** QGroundControl v5.1.5 (GPLv3), Mission Planner, Arduino IDE / ESP-IDF (C++).
* **Total License Cost:** **\$0.00** (100% open-source software stack).

### 8.3 Laboratory Facilities & Academic Support
* **Facilities:** Department of Electrical Engineering Laboratories, Military College of Signals (MCS), National University of Sciences & Technology (NUST).
* **Supervision:** Technical guidance, academic direction, and project review provided by Prof. Dr. Abdul Ghafoor.

---

## 9. Conclusion

This Interim Progress Report – 1 documents the complete engineering progress for the Final Year Project **"Swarm Drones Coordinated Controlled Movement"**. By merging physical hardware constraints with advanced multi-agent control theory, the project has achieved:
1. A mathematically rigorous control engine featuring Hungarian optimal slot assignment, distributed Laplacian consensus, and an onboard Artificial Potential Field safety barrier.
2. An asymmetric hybrid supervisor that eliminates control chattering (reducing mode switches from $302.8$ to $0.6$ under $50\%$ packet loss) and ensures smooth $\alpha(t)$ continuous control blending.
3. Real quadcopter parameter identification directly from ArduPilot SITL, capturing physical translational lag ($\tau=0.992\text{ s}$) and rotor drag ($c_d=0.637\text{ s}^{-1}$) with an RMSE of $15.36\text{ cm}$.
4. Successful scaling to 5-drone SITL in both 2D and 3D Gazebo Harmonic physics with an interactive, thread-safe console featuring Dynamic Leader Election and live fault recovery.
5. Verification of the physical **FPV P5 Voyager / Matek H743-SLIM V3** hardware platform, utilizing modular, solder-free JST-GH harnesses and validating an ESP-NOW wireless telemetry link.

The project is on schedule for dual-drone hardware-in-the-loop bench integration and outdoor flight trials in the upcoming project phase.

---

## 10. References

1. S. N. Alkhamees, "Recent advances and a hybrid framework for cooperative UAV formation control," *Applied Sciences*, vol. 15, no. 17, p. 9761, 2025.
2. C. D. R. Pita-Romero, "Flocking behavior for dynamic and complex swarm structures," *arXiv preprint arXiv:2601.21772*, 2026.
3. T. Wang, S. Zhao, Y. Xia, Z. Pan, and H. Tian, "Consensus control of large-scale UAV swarm based on multi-layer graph," *Drones*, vol. 6, no. 12, p. 402, 2022.
4. M. Mubashir Iqbal, Z. Anwar Ali, R. Khan, and M. Shafiq, "Motion planning of UAV swarm: Recent challenges and approaches," in *Aeronautics – New Advances*, IntechOpen, 2022.
5. K. A. R. Grøntved, "Communication for UAV swarms: An open-source, low-cost solution based on ESP-NOW," Master's thesis, University of Southern Denmark, 2025.
6. R. Pasic, I. Kuzmanov, and K. Atanasovski, "ESP-NOW communication protocol with ESP32," *Izzivi prihodnosti*, vol. 6, no. 1, 2021.
7. R. Olfati-Saber, J. A. Fax, and R. M. Murray, "Consensus and cooperation in networked multi-agent systems," *Proceedings of the IEEE*, vol. 95, no. 1, pp. 215–233, 2007.
8. L. Schenato, B. Sinopoli, M. Franceschetti, K. Poolla, and S. S. Sastry, "Foundations of control and estimation over lossy networks," *Proceedings of the IEEE*, vol. 95, no. 1, pp. 163–187, 2007.
9. J. A. Preiss, W. Hönig, G. S. Sukhatme, and N. Ayanian, "Crazyswarm: A large nano-quadcopter swarm," in *Proc. IEEE International Conference on Robotics and Automation (ICRA)*, 2017, pp. 3299–3304.
10. C. W. Reynolds, "Flocks, herds and schools: A distributed behavioral model," in *Proc. 14th ACM SIGGRAPH*, 1987, pp. 25–34.
11. H. W. Kuhn, "The Hungarian method for the assignment problem," *Naval Research Logistics Quarterly*, vol. 2, no. 1–2, pp. 83–97, 1955.
12. O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," *The International Journal of Robotics Research*, vol. 5, no. 1, pp. 90–98, 1986.
13. ArduPilot Development Team, "ArduPilot Copter Architecture and GUIDED Mode Navigation," ArduPilot Documentation, 2026. [Online]. Available: https://ardupilot.org/copter/
14. MAVLink Protocol Association, "MAVLink 2.0 Message Specifications: SET_POSITION_TARGET_LOCAL_NED (#84)," 2026. [Online]. Available: https://mavlink.io/en/messages/common.html
