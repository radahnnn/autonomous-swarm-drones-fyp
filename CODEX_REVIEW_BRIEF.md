# SWARM DRONES FYP — COMPREHENSIVE TECHNICAL BRIEFING & ARCHITECTURE REVIEW
**Project:** Autonomous Swarm Drones Coordinated Movement Framework (Final Year Project BEE-60)  
**Target Hardware:** 5-inch FPV Quadcopters, Matek H743-SLIM V3 (STM32H743), 6S LiPo, ESP32 Wi-Fi Bridge, Plain GPS (u-blox M10Q)  
**Flight Controller Software:** ArduPilot Copter (V4.6 / V4.8-dev) in GUIDED mode  
**Review Target:** Cross-Architecture Audit & Expert Recommendations (for OpenAI Codex / Claude Code)  
**Date:** October 1, 2026  

---

## 1. Executive Summary & Problem Formulation

### 1.1 The Research Problem
Autonomous multi-vehicle drone swarms flying coordinated geometric formations (V-Shape, Line, Circle, Grid) face a fundamental trade-off:
- **Centralized Coordination (GCS / Ground Master)** achieves optimal global formation geometry and Hungarian minimum-displacement slot matching, but has a **single point of failure**: wireless packet dropouts, channel contention, and latency cause delayed or lost setpoints, risking swarm divergence or catastrophic collisions.
- **Decentralized Coordination (Local Inter-Drone Flocking)** relies on 1-hop neighbor peer broadcasts and Laplacian consensus ($\dot{\mathbf{v}} = -\mathbf{L}\mathbf{v}$) with Reynolds rules and Artificial Potential Field (APF) collision avoidance. While resilient to coordinator dropouts, it suffers from slower convergence, noise accumulation across the communication graph, and higher tracking errors ($5 - 15\text{ cm}$ vs $< 1\text{ cm}$).
- **The Core Solution: Resilient Hybrid Switching with Continuous Blending**:
  The swarm executes centralized trajectory tracking during healthy link conditions, automatically degrades to local flocking when coordinator heartbeats drop, locks into fallback during a dwell-time lockout to prevent rapid chattering, and smoothly blends back to centralized control ($\alpha(t) \in [0, 1]$) once the wireless channel recovers.

---

## 2. Mathematical & Algorithmic Architecture

### 2.1 Vehicle Kinematic & Dynamic Model (`swarm_core/drone.py`)
Each drone $i$ is modeled as a 3D dynamical agent with first-order closed-loop attitude/translation lag ($\tau$) and aerodynamic rotor drag ($c_d$):
$$\dot{\mathbf{p}}_i = \mathbf{v}_i$$
$$\dot{\mathbf{v}}_i = \mathbf{a}_i - c_d \mathbf{v}_i$$
$$\dot{\mathbf{a}}_i = \frac{1}{\tau} (\mathbf{a}_{\text{cmd}, i} - \mathbf{a}_i)$$
subject to velocity and acceleration saturation limits:
$$\|\mathbf{v}_i\| \le v_{\max} \quad (3.0\text{ m/s}), \qquad \|\mathbf{a}_{\text{cmd}, i}\| \le a_{\max} \quad (2.5\text{ m/s}^2)$$
- **Fitted Parameters from ArduPilot SITL**: $\tau = \mathbf{0.992\text{ s}}$, $c_d = \mathbf{0.637\text{ s}^{-1}}$ (identified via 5.0m step response optimization; residual RMSE $= \mathbf{15.36\text{ cm}}$).

---

### 2.2 Wireless Channel & Network Emulator (`swarm_core/network.py`)
Packets are transmitted through an emulated lossy, delayed wireless channel:
1. **Gilbert-Elliott Burst Loss Model**: 2-state discrete Markov chain ($G$: Good / zero loss, $B$: Bad / 100% loss). Transition probabilities $p_{GB}$ and $p_{BG}$ simulate bursty RF fading and multipath drops.
2. **Deterministic Outages**: Injects full communication blackouts of 1.0s, 2.0s, and 3.0s to stress-test fallback lockout and recovery.
3. **Latency Queue**: Time-delayed FIFO queue modeling propagation and transmission latency ($\tau_{\text{lat}} \in [10, 400]\text{ ms}$).

---

### 2.3 Control Modes (`swarm_core/controllers/`)

#### A. Centralized Controller (`centralized.py`)
- **Hungarian Optimal Slot Assignment**: Solves the Linear Sum Assignment problem $\min \sum_{i=1}^N \|\mathbf{p}_i - \mathbf{s}_{\sigma(i)}\|^2$ using `scipy.optimize.linear_sum_assignment` to prevent trajectory crossings during formation morphing.
- **Control Law with Velocity Feedforward**:
  $$\mathbf{a}_{\text{cmd}, i} = k_p (\mathbf{s}_i - \mathbf{p}_i) + k_d (\mathbf{v}_{\text{target}} - \mathbf{v}_i) + c_d \mathbf{v}_{\text{target}} + \mathbf{F}_{\text{rep}, i}$$
  - *Analytical Proof*: Without feedforward, steady-state lag is $e_{\text{steady}} = \frac{k_d + c_d}{k_p} \|\mathbf{v}_{\text{target}}\| = \frac{2.2 + 0.2}{1.8} \times 0.8544 = 1.1392\text{ m}$. Adding feedforward reduces steady error to $\mathbf{0.018\text{ m}}$ (a **98.4% reduction**).

#### B. Decentralized Controller (`decentralized.py`)
- **Laplacian Velocity Consensus**: $\mathbf{a}_{\text{consensus}, i} = -k_v \sum_{j \in \mathcal{N}_i} a_{ij} (\mathbf{v}_i - \mathbf{v}_j)$, where $\mathbf{L} = \mathbf{D} - \mathbf{A}$.
- **Reynolds Flocking & Spacing Cohesion**: Computes target slot offsets relative to 1-hop neighbor positions.
- **Onboard APF Safety Barrier**:
  $$\mathbf{F}_{\text{rep}, i} = \sum_{j \ne i, \|\mathbf{d}_{ij}\| < d_{\text{col}}} k_{\text{rep}} \left(\frac{1}{\|\mathbf{d}_{ij}\|} - \frac{1}{d_{\text{col}}}\right) \frac{\mathbf{d}_{ij}}{\|\mathbf{d}_{ij}\|^3}$$
  Always active directly on the vehicle regardless of network state.

#### C. Hybrid Controller with Hysteresis State Machine (`hybrid.py`)
To prevent dangerous control chattering on lossy links, the hybrid controller uses an **asymmetric hysteresis state machine**:
1. **Degrade Trigger**: If no coordinator packet arrives within $t_{\text{degrade}} = 0.50\text{ s}$, mode switches: $\text{CENTRALIZED} \longrightarrow \text{DECENTRALIZED}$.
2. **Dwell-Time Lockout**: Once in fallback, the drone is locked in decentralized mode for at least $t_{\text{dwell}} = 2.00\text{ s}$ to prevent rapid flapping.
3. **Sliding-Window Recovery**: Recovery requires a **delivery ratio $\ge 70\%$ across a 20-tick sliding window** ($2.0\text{ s}$ history).
4. **Continuous $\alpha(t)$ Blending**: Prevents step-discontinuities in acceleration setpoints during mode transitions:
   $$\mathbf{u}_{\text{hybrid}}(t) = \alpha(t) \mathbf{u}_{\text{centralized}}(t) + (1 - \alpha(t)) \mathbf{u}_{\text{decentralized}}(t)$$
   where $\alpha(t)$ slews continuously between $0.0$ and $1.0$ at a rate of $2.0\text{ s}^{-1}$ ($500\text{ ms}$ smooth ramp).

---

### 2.4 SITL & Hardware Integration Architecture (`sitl/`)

```
+-----------------------------------------------------------------------------------------+
|                                     GROUND LAPTOP                                       |
|                                                                                         |
|  [ Swarm Core Mission Planner ]                                                         |
|         |                                                                               |
|         v                                                                               |
|  [ WirelessChannel (0-50% loss, latency, outages) ]                                     |
|         |                                                                               |
|         v                                                                               |
|  [ MAVLinkSwarmAdapter ]                                                                |
|         |                                                                               |
|         +---> CommonCoordinateFrame (Global WGS84 Tangent Datum: error < 0.1mm)         |
+---------+-------------------------------------------------------------------------------+
          |
          | MAVLink2 over TCP (SITL) / UDP (5 GHz Wi-Fi to ESP32)
          v
+-----------------------------------------------------------------------------------------+
|                              DRONE ONBOARD AUTOPILOT                                    |
|                                                                                         |
|  ESP32 WiFi Bridge (UART7 / TELEM1, 921600 baud)                                        |
|         |                                                                               |
|         v                                                                               |
|  Matek H743-SLIM V3 (ArduCopter V4.6 GUIDED Mode)                                       |
|    - EKF3 Navigation Filter (GPS + Baro + Dual ICM42688P IMU)                           |
|    - Cascaded Position Controller (PSC_POSXY_P=1.0, PSC_VELXY_P=2.0)                    |
|    - DShot600 + BDShot ESC RPM Notch Filter (INS_HNTCH_MODE=3)                          |
+-----------------------------------------------------------------------------------------+
```

---

## 3. Reviewer Audit: The 8 Concerns & Their Solutions

| # | Concern Raised | Root Cause Identified | Engineering Solution Implemented | Verification Evidence |
| :- | :--- | :--- | :--- | :--- |
| **1** | Hybrid results looked too good at 50% loss | Test targets were static; lost packets cost nothing; 5 consecutive packets recovery had 3% probability | Added dynamic moving targets ($0.85\text{ m/s}$), in-flight morphing, and replaced consecutive count with **20-tick sliding window ($\ge 70\%$)** | Verified across 6 seeds with mean $\pm$ std in [`TASK_B_EXPERIMENTAL_REPORT.md`](file:///home/drone/.gemini/antigravity/scratch/swarm_drones_fyp/TASK_B_EXPERIMENTAL_REPORT.md) |
| **2** | Stale-age limit (150ms) conflicted with latency sweep (400ms) | Constant 150ms rejection threshold rejected packets purely due to channel delay | Decoupled age threshold relative to tested latency; swept loss and latency as separate experiments | Benchmarked cleanly up to 400ms latency |
| **3** | Dot model too ideal (2 mm error) | Point-mass kinematics ignored attitude lag and drag | Added lag $\tau$, drag $c_d$, and sensor noise $\sigma$. Scripted 5m step response in SITL; fitted $\tau = 0.992\text{s}$, $c_d = 0.637\text{s}^{-1}$ | Residual RMSE $= \mathbf{15.36\text{ cm}}$ in [`TASK_C_VALIDATION_REPORT.md`](file:///home/drone/.gemini/antigravity/scratch/swarm_drones_fyp/TASK_C_VALIDATION_REPORT.md) |
| **4** | Two separate codebases (sim vs SITL) | SITL script had separate leader-follower logic | Built `MAVLinkSwarmAdapter` running the exact `swarm_core` controllers on MAVLink. Co-simulated 3-drone scenario on common 10 Hz timebase | Overall Trajectory RMS difference $= \mathbf{72.50\text{ cm}}$ across 300 synchronized frames |
| **5** | "100% complete" claim was premature | Overclaiming when SITL had only 1 formation | Replaced all superlative claims with honest audited breakdown: Sim 95%, SITL 65%, Benchmarks 90%, Hardware 10% | Updated in [`FYP_RUNNING_LOG.md`](file:///home/drone/.gemini/antigravity/scratch/swarm_drones_fyp/FYP_RUNNING_LOG.md) |
| **6** | Gazebo visuals were gold-plating | Visual carbon textures earned zero thesis marks | Froze all Gazebo visuals. Standardized geometry (7.0m wingspan) and corrected wingman labels | Zero visual changes |
| **7** | Local NED frame offsets | Each SITL vehicle local NED starts at spawn; 5m spawn offset was carried into flight | Created `CommonCoordinateFrame` projecting all drones into a shared WGS84 tangent plane | Unit tested: round-trip error $< \mathbf{0.1\text{ mm}}$ within 100m radius |
| **8** | Defense risk on AI-assisted code | Student must understand and defend every line | Equipped every report and PR with plain-English mathematical derivations and examiner defense notes | Complete examiner Q&A sections added |

---

## 4. Key Empirical Benchmark Results

### 4.1 Task B: Baseline Comparison on Identical Random Seeds (6 Seeds)
Evaluated across seeds `[42, 59, 76, 93, 110, 127]` under dynamic moving targets:
- **Gilbert-Elliott Burst Loss**:
  - *Naive Hybrid*: **$107.5 \pm 6.8$ mode switches/run** (destructive chattering).
  - *Proposed Hybrid*: **$0.0 \pm 0.0$ mode switches/run** (**99.5%+ chattering suppression** due to $0.5\text{s}$ degrade filtering).
- **Deterministic Outages (1.0s, 2.0s, 3.0s)**:
  - *Proposed Hybrid*: Entered fallback exactly once ($1.0 \pm 0.0$), locked for $2.05\text{s}$ dwell, smoothly recovered in $0.34\text{s} - 1.65\text{s}$.
  - *Minimum Inter-Drone Clearance*: Maintained at $1.38 \pm 0.26\text{ m}$ (safety floor $= 0.70\text{ m}$).
- **GPS Noise Sweep ($\sigma \in \{0.04, 0.50, 1.50, 2.50\}\text{ m}$)**:
  - Evaluated with $60\%$ shared common-mode constellation error.
  - *Proposed Hybrid Steady Error*: $0.058\text{ m}$ ($\sigma=0.04$) $\to$ $0.084\text{ m}$ ($\sigma=2.50\text{ m}$). Common-mode error shifts the entire swarm together, preserving relative geometry!
  - *Decentralized Error*: Degrades to $\mathbf{3.054 \pm 1.222\text{ m}}$ due to noise propagation across the graph.

### 4.2 Task C: ArduPilot SITL Co-Simulation (3 Drones, 10 Hz Synchronized Timebase)
- **Vehicle 0 (Apex Leader)**: Trajectory RMS Difference $= \mathbf{0.7152\text{ m}}$ ($71.52\text{ cm}$), Max discrepancy $= 0.9601\text{ m}$.
- **Vehicle 1 (Left Wingman)**: Trajectory RMS Difference $= \mathbf{0.7235\text{ m}}$ ($72.35\text{ cm}$), Max discrepancy $= 0.9617\text{ m}$.
- **Vehicle 2 (Right Wingman)**: Trajectory RMS Difference $= \mathbf{0.7361\text{ m}}$ ($73.61\text{ cm}$), Max discrepancy $= 0.9904\text{ m}$.
- **Overall Swarm Trajectory RMS Difference**: **$0.7250\text{ m}$ ($72.50\text{ cm}$)**.

---

## 5. Hardware Specifications & Deployment Plan (Task E)

1. **Flight Controller**: Matek H743-SLIM V3 (STM32H743VIT6 @ 480 MHz, dual IMU ICM42688P + ICM42605, DPS310 baro).
2. **Serial Mapping**:
   - `SERIAL0`: USB-C (GCS).
   - `SERIAL1`: **UART7 (`TX7`/`RX7`)** $\to$ ESP32 Wi-Fi Telemetry Bridge (`SERIAL1_PROTOCOL = 2`, `SERIAL1_BAUD = 921`).
   - `SERIAL3`: USART2 (`TX2`/`RX2`) $\to$ Primary GPS + I2C Compass (`SERIAL3_PROTOCOL = 5`, `SERIAL3_BAUD = 38`).
   - `SERIAL7`: USART6 (`RX6`) $\to$ Radio Receiver (ELRS / CRSF).
3. **ESC Protocol**: DShot600 with Bi-directional DShot (`MatekH743-bdshot` firmware, `SERVO_BLH_BDMASK = 15`, `INS_HNTCH_MODE = 3`).
4. **Safety Failsafes**: `FS_GCS_ENABLE = 7` (**BRAKE or LAND**), `FS_GCS_TIMEOUT = 5.0` s, `FS_OPTIONS = 4` (Continue Guided on RC failsafe).
5. **Network Topology**: Ground Laptop connected via UDP socket to a dedicated **5 GHz Wi-Fi Travel Router** (e.g. GL.iNet GL-MT3000). ESP32 modules on each drone connect as Stations with static IPs (`192.168.1.101-105:14550`). 5 GHz eliminates interference with 2.4 GHz ELRS and 5.8 GHz FPV video.

---

## 6. Questions & Prompt for Codex Review

When prompting Codex with this document, use the following prompt:

```text
You are reviewing the autonomous drone swarm framework detailed in CODEX_REVIEW_BRIEF.md.
Please provide a critical peer review focusing on:
1. Control & Stability: Is the asymmetric hysteresis state machine with continuous alpha(t) blending provably stable under Lyapunov or switched systems theory?
2. Sim-to-Real Mismatch: Given the 72.5 cm trajectory RMS difference between swarm_core and ArduPilot SITL, what physical effects (aerodynamic downwash ground effect, ESC deadbands, clock drift) should we anticipate on physical 5-inch 6S hardware?
3. Communication & Network: Are there edge cases in UDP socket broadcasting across 5-10 ESP32 nodes where packet loss bursts could synchronize (e.g. Wi-Fi beacon collision), and how can we mitigate this?
4. Final Defense Recommendations: What 2-3 novel academic plots or analyses should be included in the thesis to impress the examination committee?
```
