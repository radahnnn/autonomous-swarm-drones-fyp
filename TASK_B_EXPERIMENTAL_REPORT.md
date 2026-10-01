# Final Year Project BEE-60: Task B Experimental Report & Defense Guide
**Autonomous Swarm Drones Coordinated Movement Framework**  
**Date:** 1 October 2026 | **Author:** FYP Swarm Team | **Branch:** `master`

---

## 1. Executive Summary & Audit Overview

In accordance with academic review recommendations, **Task B (Fix the Experiments)** has been completed. All simulations were upgraded from ideal kinematic models to physics-lagged, drag-aware multirotor dynamics with realistic network impairments, dual-component sensor noise, and feedforward control.

### Core Upgrades Implemented
1. **Wireless Outages & Correlated Burst Loss**: Upgraded `WirelessChannel` with a Gilbert-Elliott 2-state Markov model ($p_{g \to b} = 0.05, p_{b \to g} = 0.20$) and deterministic complete RF outages ($1.0\text{s}, 2.0\text{s}, 3.0\text{s}$). Measured fallback entry count, dwell duration, and link recovery time.
2. **Dual-Component GPS Noise Model**: Horizontal positioning error modeled as $\mathbf{w}_{\text{GPS}, i} = \mathbf{w}_{\text{common}} + \mathbf{w}_{\text{indep}, i}$, where $60\%$ of variance is common-mode (shared constellation/atmospheric bias) and $40\%$ is independent receiver noise. Swept across $\sigma \in \{0.04, 0.5, 1.5, 2.5\}\text{ m}$.
3. **Four Baselines Evaluated on Identical Seeds**:
   - *Pure Centralized (Hold-Last-Command)*: Holds last received command when packets drop.
   - *Pure Decentralized*: Local Laplacian consensus + Artificial Potential Field (APF).
   - *Naive Hybrid*: Instant degrade (1 drop) and instant recovery (1 packet), zero dwell time.
   - *Proposed Hybrid*: Asymmetric hysteresis, sliding-window delivery ratio ($\ge 70\%$), dwell time ($2.0\text{s}$), and smooth continuous blending $\alpha(t)$.
4. **Target Velocity Feedforward & Error Decomposition**: Split tracking error into transient (during mid-flight morphing, $t \in [5.0, 7.5\text{s}]$) and steady-state ($t \in [8.5, 12.0\text{s}]$). Proved analytically and validated empirically why steady-state error was $\approx 1.14\text{ m}$ without feedforward, and demonstrated reduction to $0.018\text{ m}$ with feedforward enabled.
5. **Parameter Provenance Registry**: Centralized all physical, sensor, control, and network parameters in [`swarm_core/config.py`](file:///home/drone/.gemini/antigravity/scratch/swarm_drones_fyp/swarm_core/config.py) with explicit `"assumed"` labels.

---

## 2. Parameter Provenance Table (`swarm_core/config.py`)

All parameters are categorized with explicit provenance to ensure academic honesty:

| Parameter Name | Value | Unit | Provenance | Engineering Rationale / Hardware Basis |
| :--- | :---: | :---: | :---: | :--- |
| `attitude_tau` | 0.18 | $\text{s}$ | **Assumed** | Typical first-order closed-loop tilt response for Betaflight/ArduPilot 5-inch quad. Pending frequency-sweep identification. |
| `drag_coeff` ($c_d$) | 0.20 | $1/\text{s}$ | **Assumed** | Linear aerodynamic rotor drag coefficient. Pending flight coast-down fitting. |
| `drone_radius` | 0.35 | $\text{m}$ | **Assumed** | Physical span: 230mm wheelbase + 5-inch prop radius + frame margin ($0.28\text{m} \to 0.35\text{m}$). |
| `collision_threshold` | 0.70 | $\text{m}$ | **Assumed** | Double physical radius ($2 \times 0.35\text{m}$): physical prop-strike boundary. |
| `apf_safe_radius` | 2.50 | $\text{m}$ | **Assumed** | Repulsive APF activation distance; matches planned nominal formation spacing. |
| `gps_noise_plain` | 1.50 | $\text{m}$ | **Assumed** | Typical CEP accuracy of U-Blox M8N/M9N plain GPS without RTK corrections. |
| `gps_noise_rtk` | 0.04 | $\text{m}$ | **Assumed** | Centimeter-level accuracy achievable only with RTK base station / NTRIP link. |
| `gps_common_fraction` | 0.60 | ratio | **Assumed** | $60\%$ shared constellation geometry & atmospheric delay across co-located quads. |
| `degrade_timeout` | 0.50 | $\text{s}$ | **Assumed** | Duration of link silence before decentralized fallback ($10$ consecutive missed ticks at 20 Hz). |
| `min_dwell_time` | 2.00 | $\text{s}$ | **Assumed** | Asymmetric hysteresis lockout preventing chattering across transient link edges. |
| `recovery_window_size` | 20 | ticks | **Assumed** | 1.0s observation window at 20 Hz. |
| `recovery_ratio_threshold` | 0.70 | ratio | **Assumed** | Requires $\ge 70\%$ packet reception within sliding window to exit fallback. |
| `ramp_duration` | 0.80 | $\text{s}$ | **Assumed** | Continuous blending time $\tau_{\text{ramp}}$ for $\alpha(t) \in [0.0, 1.0]$. |

---

## 3. Experiment 1: Deterministic Outages & Burst Loss Across 4 Baselines

### Setup
- **Swarm Size**: 5 drones.
- **Seeds**: 6 identical seeds (`[42, 59, 76, 93, 110, 127]`).
- **Trajectory**: Continuous moving centroid ($\mathbf{v}_{\text{target}} = [0.8, 0.3]\text{ m/s}$, $\|\mathbf{v}\| = 0.854\text{ m/s}$).
- **Mid-Flight Morph**: V-Shape to Line at $t = 6.0\text{s}$.
- **Outage Windows**: Deterministic complete blackout from $t = 3.0\text{s}$ for durations $\Delta t \in \{0.0, 1.0, 2.0, 3.0\}\text{s}$.
- **Burst Loss**: Gilbert-Elliott 2-state Markov channel ($p_{g \to b} = 0.05$, $p_{b \to g} = 0.20$, bad loss rate $= 100\%$).

### Empirical Results (Raw Mean $\pm$ Standard Deviation)

| Baseline | Outage / Impairment | Fallback Entries / Drone | Time in Fallback ($\text{s}$) | Recovery Time ($\text{s}$) | Mode Switches per Run | Steady-State Error ($\text{m}$) | Min Recorded Distance ($\text{m}$) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Centralized (Hold)** | $0.0\text{s}$ | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.078 \pm 0.000$ | $1.33 \pm 0.30$ |
| **Decentralized** | $0.0\text{s}$ | $0.0 \pm 0.0$ | $12.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.575 \pm 0.031$ | $1.42 \pm 0.23$ |
| **Naive Hybrid** | $0.0\text{s}$ | $0.5 \pm 0.3$ | $0.03 \pm 0.02$ | $0.05 \pm 0.01$ | $5.3 \pm 3.2$ | $0.060 \pm 0.003$ | $1.38 \pm 0.26$ |
| **Proposed Hybrid** | $0.0\text{s}$ | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.059 \pm 0.001$ | $1.38 \pm 0.26$ |
| **Centralized (Hold)** | $1.0\text{s}$ | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.078 \pm 0.001$ | $1.33 \pm 0.30$ |
| **Decentralized** | $1.0\text{s}$ | $0.0 \pm 0.0$ | $12.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.611 \pm 0.027$ | $1.42 \pm 0.23$ |
| **Naive Hybrid** | $1.0\text{s}$ | $1.5 \pm 0.2$ | $0.98 \pm 0.01$ | $0.10 \pm 0.00$ | $14.7 \pm 1.9$ | $0.060 \pm 0.001$ | $1.38 \pm 0.26$ |
| **Proposed Hybrid** | $1.0\text{s}$ | $1.0 \pm 0.0$ | $2.05 \pm 0.00$ | $1.65 \pm 0.00$ | $10.0 \pm 0.0$ | $0.083 \pm 0.001$ | $1.38 \pm 0.26$ |
| **Centralized (Hold)** | $2.0\text{s}$ | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.077 \pm 0.000$ | $1.33 \pm 0.30$ |
| **Decentralized** | $2.0\text{s}$ | $0.0 \pm 0.0$ | $12.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.618 \pm 0.028$ | $1.42 \pm 0.23$ |
| **Naive Hybrid** | $2.0\text{s}$ | $1.5 \pm 0.2$ | $1.98 \pm 0.01$ | $0.10 \pm 0.00$ | $14.7 \pm 1.9$ | $0.059 \pm 0.001$ | $1.38 \pm 0.26$ |
| **Proposed Hybrid** | $2.0\text{s}$ | $1.0 \pm 0.0$ | $2.05 \pm 0.00$ | $0.65 \pm 0.00$ | $10.0 \pm 0.0$ | $0.084 \pm 0.001$ | $1.38 \pm 0.26$ |
| **Centralized (Hold)** | $3.0\text{s}$ | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.079 \pm 0.003$ | $1.33 \pm 0.30$ |
| **Decentralized** | $3.0\text{s}$ | $0.0 \pm 0.0$ | $12.00 \pm 0.00$ | $0.00 \pm 0.00$ | $0.0 \pm 0.0$ | $0.601 \pm 0.029$ | $1.42 \pm 0.23$ |
| **Naive Hybrid** | $3.0\text{s}$ | $1.5 \pm 0.2$ | $2.97 \pm 0.01$ | $0.10 \pm 0.00$ | $14.7 \pm 1.9$ | $0.081 \pm 0.003$ | $1.38 \pm 0.26$ |
| **Proposed Hybrid** | $3.0\text{s}$ | $1.0 \pm 0.0$ | $2.71 \pm 0.01$ | $0.34 \pm 0.03$ | $10.0 \pm 0.0$ | $0.130 \pm 0.002$ | $1.38 \pm 0.26$ |
| **Centralized (Hold)** | **GE-BURST** | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | — | $0.0 \pm 0.0$ | $0.079 \pm 0.001$ | $1.34 \pm 0.28$ |
| **Decentralized** | **GE-BURST** | $0.0 \pm 0.0$ | $12.00 \pm 0.00$ | — | $0.0 \pm 0.0$ | $0.511 \pm 0.067$ | $1.43 \pm 0.23$ |
| **Naive Hybrid** | **GE-BURST** | $10.8 \pm 0.6$ | $0.88 \pm 0.10$ | — | **$107.5 \pm 6.8$** | $0.061 \pm 0.004$ | $1.37 \pm 0.30$ |
| **Proposed Hybrid** | **GE-BURST** | $0.0 \pm 0.0$ | $0.00 \pm 0.00$ | — | **$0.0 \pm 0.0$** | $0.065 \pm 0.002$ | $1.38 \pm 0.26$ |

### Key Physical & Algorithmic Observations
1. **Chattering Suppression**: Under Gilbert-Elliott burst loss, the Naive Hybrid exhibited violent chattering with **$107.5 \pm 6.8$ mode switches per run** ($10.8$ fallback entries per drone). The Proposed Hybrid eliminated this chattering entirely (**$0.0 \pm 0.0$ switches**), filtering out short burst dropouts ($< 0.5\text{s}$) through its degrade timer.
2. **Deterministic Outage Handling**: During a $1.0\text{s}$ outage, the Proposed Hybrid entered fallback exactly once per drone ($1.0 \pm 0.0$), locked in decentralized mode for the minimum dwell time ($2.05\text{s}$), and returned to centralized operation with exactly $10.0 \pm 0.0$ total transitions ($5 \text{ drones} \times 2 \text{ transitions}$).
3. **Safety Barrier Integrity**: Minimum inter-drone distance remained at $1.38\text{ m} \pm 0.26\text{ m}$, well above the $0.70\text{ m}$ collision threshold across all outage durations.

---

## 4. Experiment 2: GPS Noise Sweep ($\sigma \in \{0.04, 0.5, 1.5, 2.5\}\text{ m}$)

### Setup
- Evaluated dual-component error: $\mathbf{w}_{\text{GPS}, i} = \mathbf{w}_{\text{common}} + \mathbf{w}_{\text{indep}, i}$ with $\sigma_{\text{common}}^2 / \sigma^2 = 0.60$.
- Tested against $\sigma \in \{0.04, 0.50, 1.50, 2.50\}\text{ m}$.
- 6 seeds (`[100, 123, 146, 169, 192, 215]`), 5 drones, 12s simulation duration.

### Empirical Results (Raw Mean $\pm$ Standard Deviation)

| Controller Mode | GPS Noise $\sigma$ ($\text{m}$) | Steady-State Error ($\text{m}$) | Transient Morph Error ($\text{m}$) | Min Inter-Drone Distance ($\text{m}$) | Physical Collisions |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Proposed Hybrid** | $0.04$ | $0.058 \pm 0.000$ | $1.166 \pm 0.003$ | $1.670 \pm 0.227$ | $0/6$ |
| **Pure Centralized** | $0.04$ | $0.077 \pm 0.000$ | $1.171 \pm 0.003$ | $1.675 \pm 0.222$ | $0/6$ |
| **Pure Decentralized** | $0.04$ | $0.561 \pm 0.033$ | $1.314 \pm 0.057$ | $1.635 \pm 0.210$ | $0/6$ |
| **Proposed Hybrid** | $0.50$ | $0.058 \pm 0.001$ | $1.166 \pm 0.003$ | $1.675 \pm 0.226$ | $0/6$ |
| **Pure Centralized** | $0.50$ | $0.077 \pm 0.000$ | $1.171 \pm 0.003$ | $1.675 \pm 0.222$ | $0/6$ |
| **Pure Decentralized** | $0.50$ | $0.878 \pm 0.218$ | $1.553 \pm 0.184$ | $1.651 \pm 0.220$ | $0/6$ |
| **Proposed Hybrid** | $1.50$ | $0.089 \pm 0.010$ | $1.201 \pm 0.016$ | $1.680 \pm 0.227$ | $0/6$ |
| **Pure Centralized** | $1.50$ | $0.077 \pm 0.000$ | $1.171 \pm 0.003$ | $1.675 \pm 0.222$ | $0/6$ |
| **Pure Decentralized** | $1.50$ | $1.979 \pm 0.751$ | $2.545 \pm 0.679$ | $1.668 \pm 0.226$ | $0/6$ |
| **Proposed Hybrid** | $2.50$ | $0.084 \pm 0.007$ | $1.211 \pm 0.007$ | $1.670 \pm 0.224$ | $0/6$ |
| **Pure Centralized** | $2.50$ | $0.077 \pm 0.000$ | $1.171 \pm 0.003$ | $1.675 \pm 0.222$ | $0/6$ |
| **Pure Decentralized** | $2.50$ | **$3.054 \pm 1.222$** | **$3.241 \pm 1.061$** | $1.636 \pm 0.193$ | $0/6$ |

### Physical Defense Rationale
- **Decentralized Error Accumulation**: In Pure Decentralized mode, errors grow linearly with GPS noise ($\approx 0.56\text{ m}$ at $\sigma=0.04\text{ m} \to 3.05\text{ m}$ at $\sigma=2.5\text{ m}$) because peer consensus broadcasts noisy relative positions across the network graph without a global anchoring reference.
- **Common-Mode Rejection**: In Centralized and Hybrid modes, the $60\%$ common-mode component shifts the entire formation synchronously in global coordinates, preserving internal formation geometry. Steady-state error remains under $0.09\text{ m}$ even at $\sigma = 2.50\text{ m}$.
- **Collision Safety**: Despite up to $2.5\text{ m}$ GPS noise, zero collisions occurred in any trial ($0/6$) because local Artificial Potential Field repulsive forces maintain a minimum inter-drone clearance of $> 1.63\text{ m}$ (safety boundary $= 0.70\text{ m}$).

---

## 5. Experiment 3: Target Velocity Feedforward & Mathematical Derivation

### Mathematical Proof: Why Steady-State Error was $\approx 1.14\text{ m}$ at 0% Loss

In the baseline implementation, the hybrid controller computed velocity damping assuming a static target:
$$v_{\text{err}} = -v_{\text{drone}}$$
The reference trajectory moves at constant ground velocity $\mathbf{v}_{\text{target}} = [0.8, 0.3]^T\text{ m/s}$, with magnitude:
$$\|\mathbf{v}_{\text{target}}\| = \sqrt{0.8^2 + 0.3^2} = \sqrt{0.73} \approx 0.8544\text{ m/s}$$
The multirotor physical dynamics include aerodynamic rotor drag opposing velocity:
$$a_{\text{drag}} = -c_d \mathbf{v}_{\text{drone}}$$
Where $c_d = 0.20\text{ s}^{-1}$. Commanded acceleration from the PD tracking law was:
$$u_{\text{cmd}} = k_p (\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{drone}}) + k_d (-\mathbf{v}_{\text{drone}})$$
In steady state, the drone cruises at constant velocity $\mathbf{v}_{\text{drone}} = \mathbf{v}_{\text{target}}$, meaning actual acceleration $\dot{\mathbf{v}} = 0$:
$$\dot{\mathbf{v}} = u_{\text{cmd}} + a_{\text{drag}} = 0$$
$$k_p \mathbf{e}_{\text{steady}} - k_d \mathbf{v}_{\text{target}} - c_d \mathbf{v}_{\text{target}} = 0$$
$$k_p \mathbf{e}_{\text{steady}} = (k_d + c_d) \mathbf{v}_{\text{target}}$$
Solving for the steady-state tracking error magnitude:
$$\|\mathbf{e}_{\text{steady}}\| = \frac{k_d + c_d}{k_p} \|\mathbf{v}_{\text{target}}\|$$
Substituting the controller gains ($k_p = 1.8\text{ s}^{-2}, k_d = 2.2\text{ s}^{-1}$) and physical drag ($c_d = 0.20\text{ s}^{-1}$):
$$\|\mathbf{e}_{\text{steady}}\| = \frac{2.2 + 0.20}{1.8} \times 0.8544 = \frac{2.4}{1.8} \times 0.8544 = \frac{4}{3} \times 0.8544 = 1.1392\text{ m} \approx 1.14\text{ m}$$

### The Solution: Velocity Feedforward & Drag Compensation
By introducing target velocity feedforward and rotor drag compensation:
$$u_{\text{cmd}} = k_p (\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{drone}}) + k_d (\mathbf{v}_{\text{target}} - \mathbf{v}_{\text{drone}}) + c_d \mathbf{v}_{\text{target}}$$
When $\mathbf{v}_{\text{drone}} = \mathbf{v}_{\text{target}}$:
$$\dot{\mathbf{v}} = k_p \mathbf{e}_{\text{steady}} + 0 + c_d \mathbf{v}_{\text{target}} - c_d \mathbf{v}_{\text{target}} = k_p \mathbf{e}_{\text{steady}} = 0 \implies \mathbf{e}_{\text{steady}} = 0$$

### Empirical Validation (0% Loss, 6 Seeds)

| Mode | Feedforward State | Transient Morph Error ($t \in [5, 7.5\text{s}]$) | Steady-State Error ($t \in [8.5, 12\text{s}]$) | Final Formation Error |
| :--- | :---: | :---: | :---: | :---: |
| **Hybrid** | **Without Feedforward** | $1.849 \pm 0.000\text{ m}$ | **$1.140 \pm 0.000\text{ m}$** | $1.139 \pm 0.000\text{ m}$ |
| **Hybrid** | **With Feedforward** | **$1.329 \pm 0.000\text{ m}$** | **$0.018 \pm 0.000\text{ m}$** | $0.001 \pm 0.000\text{ m}$ |
| **Centralized** | **Without Feedforward** | $1.809 \pm 0.000\text{ m}$ | **$1.098 \pm 0.000\text{ m}$** | $1.097 \pm 0.000\text{ m}$ |
| **Centralized** | **With Feedforward** | **$1.303 \pm 0.001\text{ m}$** | **$0.049 \pm 0.000\text{ m}$** | $0.043 \pm 0.000\text{ m}$ |

**Conclusion**: The measured steady-state error of $1.140\text{ m}$ matches the analytical prediction ($1.1392\text{ m}$) to within $0.07\%$. Adding feedforward eliminates this lag entirely ($0.018\text{ m}$ steady-state error) and reduces transient morph error by $28.1\%$.

---

## 6. Oral Defense Guide: Questions & Model Answers

### Q1: "Why did your hybrid controller report 0 switches at 20-30% packet loss in earlier tests? Was the logic flawed?"
**Model Answer:**  
> *"In earlier tests, packet drops were modeled as independent Bernoulli trials at 20 Hz. At 20 Hz, losing 10 consecutive packets to exceed the 0.5-second degrade timeout has an infinitesimal probability of $0.2^{10} \approx 10^{-7}$. Real RF channels experience correlated fading and burst drops rather than purely independent trials. When we implemented a Gilbert-Elliott 2-state Markov chain and deterministic 1–3 second RF blackouts in Task B, the hybrid entered fallback reliably, held for the 2.0-second dwell time, and demonstrated clean sliding-window recovery at $\ge 70\%$ packet reception."*

### Q2: "Why was steady-state tracking error 1.14 meters even at 0% packet loss?"
**Model Answer:**  
> *"This was not a simulation bug, but an exact manifestation of classical control dynamics. Without velocity feedforward, the velocity damping term $k_d$ treats the moving target as stationary, resisting forward motion. In addition, the physics model accounts for aerodynamic rotor drag $c_d = 0.20\text{ s}^{-1}$. In steady state at $\|\mathbf{v}\| = 0.854\text{ m/s}$, the position error must generate sufficient proportional force to overcome both damping and drag: $e_{\text{steady}} = \frac{k_d + c_d}{k_p} \|\mathbf{v}\| = \frac{2.2 + 0.2}{1.8} \times 0.8544 = 1.1392\text{ m}$. When we add target velocity feedforward and drag compensation, steady-state error drops to $0.018\text{ m}$."*

### Q3: "You claim your hybrid reduces chattering. Against what baseline?"
**Model Answer:**  
> *"The chattering reduction is evaluated specifically against a Naive Hybrid controller that lacks hysteresis (instant degrade on 1 missed packet, instant recovery on 1 good packet, 0 dwell time). Under Gilbert-Elliott burst loss, the naive hybrid exhibited $107.5 \pm 6.8$ switches per run. Our proposed controller exhibited $0.0 \pm 0.0$ switches under burst loss and exactly 1 transition per drone during a full link blackout. We do not claim chattering reduction against pure centralized or pure decentralized controllers, as those baselines never switch modes."*

### Q4: "How does your swarm maintain safety with plain GPS having 1.5 to 2.5 meters of error?"
**Model Answer:**  
> *"We modeled GPS noise with two components: $60\%$ shared common-mode error and $40\%$ independent error per drone. Because all drones in a close formation observe the same satellite constellation, common-mode error shifts the entire formation together in world space, preserving internal relative spacing. Furthermore, our Artificial Potential Field safety barrier operates onboard at 100% gain at all times, preventing drones from violating the $0.70\text{ m}$ collision boundary even when independent GPS noise causes transient position deviations."*

---

## 7. Artifacts & Generated Scientific Plots
The following high-resolution comparison plots are available in `experiments/results/` and the system artifact gallery:
1. `feedforward_ablation_comparison.png`: Transient vs steady-state tracking error with and without feedforward.
2. `outage_burst_comparison.png`: Four-panel evaluation of fallback dwell time, recovery duration, mode switches, and steady error across 1s, 2s, 3s outages and Gilbert-Elliott burst loss.
3. `gps_noise_sweep_comparison.png`: Steady-state tracking error and minimum inter-drone separation across $\sigma \in \{0.04, 0.5, 1.5, 2.5\}\text{ m}$.
4. `network_loss_comparison.png`: Three-panel Bernoulli packet loss sweep ($0\%$ to $50\%$) comparing all four baselines on identical seeds.
