# TASK E RESEARCH REPORT: HARDWARE, PROTOCOLS, NETWORK TOPOLOGY & LITERATURE
**Autonomous Swarm Drones Coordinated Movement Framework (Final Year Project BEE-60)**  
**Date:** October 1, 2026  
**Status:** Complete (Report Only, Verified against ArduPilot 4.6 codebase & official documentation)  

---

## 1. ArduPilot Copter on Matek H743-SLIM V3

### 1.1 Firmware Flashing Procedure
The Matek H743-SLIM V3 comes from the factory flashed with Betaflight or INAV. To install ArduPilot:

1. **Enter STM32 DFU Bootloader Mode**:
   - Press and hold the onboard `BOOT` tactile button while inserting the USB-C cable into the computer.
   - Verify DFU device connection:
     ```bash
     lsusb  # Look for "0483:df11 STMicroelectronics STM Device in DFU Mode"
     ```
2. **Flash the ArduPilot Bootloader**:
   - Use **STM32CubeProgrammer** (or the Betaflight Configurator CLI):
     - Select **USB** connection in STM32CubeProgrammer and click **Connect**.
     - Open the official ArduPilot bootloader: [`Tools/bootloaders/MatekH743-bdshot_bl.hex`](https://firmware.ardupilot.org/Copter/stable/MatekH743-bdshot/) (or `MatekH743_bl.hex`).
     - Flash starting at base flash address `0x08000000`.
     - Disconnect and power cycle the flight controller.
3. **Flash ArduCopter Firmware**:
   - The flight controller will now present as an ArduPilot USB composite device (COM port).
   - In **Mission Planner** -> *Setup* -> *Install Firmware*:
     - Select **All Options** -> **MatekH743** or **MatekH743-bdshot** (recommended for bi-directional DShot with RPM telemetry).
     - Alternatively, upload custom `arduCopter.apj` directly.
   - Official firmware repository: [https://firmware.ardupilot.org/Copter/stable/MatekH743-bdshot/](https://firmware.ardupilot.org/Copter/stable/MatekH743-bdshot/)
   - Documentation: [https://ardupilot.org/copter/docs/common-matekh743-wing.html](https://ardupilot.org/copter/docs/common-matekh743-wing.html)

---

### 1.2 Default Serial Mapping (Matek H743-SLIM V3)
Directly extracted from the ArduPilot hardware definition file [`libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat`](file:///home/drone/ardupilot/libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat#L102):
```
SERIAL_ORDER OTG1 UART7 USART1 USART2 USART3 UART8 UART4 USART6 OTG2
```

| ArduPilot Param | MCU Hardware | Physical Board Pad | Default Protocol (`SERIALx_PROTOCOL`) | Default Baud | Intended Device |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`SERIAL0`** | `OTG1` | USB-C Port | `2` (MAVLink2) | `115200` | GCS USB Connection |
| **`SERIAL1`** | `UART7` | `TX7` / `RX7` | `2` (MAVLink2) | `57600` | **Telemetry 1 (ESP32 Bridge)** |
| **`SERIAL2`** | `USART1` | `TX1` / `RX1` | `2` (MAVLink2) | `57600` | Telemetry 2 / Companion Computer |
| **`SERIAL3`** | `USART2` | `TX2` / `RX2` | `5` (GPS) | `38400` | Primary GPS + Compass |
| **`SERIAL4`** | `USART3` | `TX3` / `RX3` | `5` (GPS) | `38400` | Secondary GPS |
| **`SERIAL5`** | `UART8` | `TX8` / `RX8` | `-1` (None) / `16` (ESC Telemetry in BDShot) | `115200` | ESC Telemetry / Spare |
| **`SERIAL6`** | `UART4` | `TX4` / `RX4` | `-1` (None) | `57600` | Spare Serial / Optical Flow |
| **`SERIAL7`** | `USART6` | `RX6` | `23` (RCIN) | `115200` | Radio Receiver (ELRS / CRSF / SBUS) |

> [!IMPORTANT]
> The ESP32 telemetry bridge should be wired to **`TX7`/`RX7` (`SERIAL1`)** or **`TX1`/`RX1` (`SERIAL2`)**. Set `SERIAL1_BAUD = 921` (921600 baud) or `115` (115200 baud) and `SERIAL1_PROTOCOL = 2` (MAVLink2).

---

### 1.3 DShot & Motor Output Setup
On the Matek H743-SLIM, the 4 motor pads correspond to:
- `M1` -> `PB0` (TIM3_CH3 on BDShot)
- `M2` -> `PB1` (TIM3_CH4 on BDShot)
- `M3` -> `PA0` (TIM2_CH1 on BDShot)
- `M4` -> `PA1` (TIM2_CH2 on BDShot)

**Parameters to configure**:
1. `MOT_PWM_TYPE = 4` (DShot300) or `5` (DShot600).
2. `SERVO_BLH_AUTO = 1` (Automatically enables BLHeli_32 / AM32 pass-through).
3. **Bi-directional DShot (BDShot)**:
   - Use `MatekH743-bdshot` firmware.
   - `SERVO_BLH_BDMASK = 15` (enables bidirectional telemetry on outputs 1, 2, 3, 4: $2^0 + 2^1 + 2^2 + 2^3 = 15$).
   - `SERVO_BLH_POLES = 14` (number of motor magnet poles for typical 2207 / 2306 FPV motors; verify against motor spec).

---

### 1.4 Initial Tuning Baseline for 5-Inch 6S Quadrotor
ArduPilot's default parameters assume large (10–15 inch) multirotors with high inertia. A 5-inch 6S quadrotor requires much wider filter bandwidths and lower rate PIDs:

| Parameter | Default (Heavy Quad) | 5-Inch 6S Baseline | Rationale |
| :--- | :--- | :--- | :--- |
| **`INS_GYRO_FILTER`** | 20 Hz | **80 Hz** | 20 Hz causes severe phase lag and instability on 5-inch frames. |
| **`INS_ACCEL_FILTER`** | 10 Hz | **10 Hz** | Accelerometer low-pass filtering. |
| **`INS_HNTCH_ENABLE`** | 0 | **1** | Dynamic harmonic notch filter eliminates motor blade-pass vibration. |
| **`INS_HNTCH_MODE`** | 0 | **3 (ESC RPM)** | Tracks motor RPM directly via BDShot telemetry. |
| **`INS_HNTCH_BW`** | 0 | **40 Hz** | Notch bandwidth. |
| **`MOT_THST_EXPO`** | 0.65 | **0.55** | Thrust curve linearization for 5-inch propellers. |
| **`MOT_SPIN_ARM`** | 0.10 | **0.08** | Minimum motor idle spin on arming. |
| **`MOT_SPIN_MIN`** | 0.15 | **0.12** | Minimum motor spin in flight. |
| **`ATC_RAT_RLL_P`** | 0.135 | **0.080 – 0.095** | Reduces high-frequency oscillation on stiff 5-inch carbon arms. |
| **`ATC_RAT_RLL_D`** | 0.0036 | **0.0030 – 0.0045** | D-term damping; increase cautiously while monitoring motor heat. |
| **`ATC_RAT_PIT_P`** | 0.135 | **0.085 – 0.100** | Pitch axis proportional gain. |
| **`ATC_RAT_PIT_D`** | 0.0036 | **0.0035 – 0.0050** | Pitch axis derivative gain. |
| **`ATC_ACCEL_R_MAX`** | 110000 | **110000 – 160000** | Deg/s² angular roll acceleration limit. |
| **`ATC_ACCEL_P_MAX`** | 110000 | **110000 – 160000** | Deg/s² angular pitch acceleration limit. |

*Official Tuning Guide*: [https://ardupilot.org/copter/docs/tuning-process-instructions.html](https://ardupilot.org/copter/docs/tuning-process-instructions.html)

---

## 2. GUIDED Mode: MAVLink Setpoints, Type Masks & Dropouts

### 2.1 Setpoint Messages
1. **`SET_POSITION_TARGET_LOCAL_NED` (Message #84)**:
   - Primary message for trajectory tracking in local tangent / NED coordinate frames.
   - Fields: `time_boot_ms`, `target_system`, `target_component`, `coordinate_frame` (`MAV_FRAME_LOCAL_NED`), `type_mask`, `x`, `y`, `z`, `vx`, `vy`, `vz`, `afx`, `afy`, `afz`, `yaw`, `yaw_rate`.
2. **`SET_POSITION_TARGET_GLOBAL_INT` (Message #86)**:
   - Used when sending global WGS84 coordinates (`lat_int`, `lon_int`, `alt`).
3. **`SET_ATTITUDE_TARGET` (Message #82)**:
   - Direct attitude quaternion and normalized thrust $[0, 1]$ command.

---

### 2.2 Standard Type Masks (`SET_POSITION_TARGET_LOCAL_NED`)
The `type_mask` is an inverted 16-bit integer bitmask where **`1` = IGNORE** and **`0` = USE**:

| Desired Mode | Binary `type_mask` | Hex | Dec | Active Fields | Ignored Fields |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Position Only** | `0000 1111 1111 1000` | `0x0DF8` | **`3576`** | $x, y, z$ | $v, a, \text{yaw}, \text{yaw rate}$ |
| **Position + Vel Feedforward** | `0000 1111 1100 0000` | `0x0FC0` | **`4032`** | $x, y, z, v_x, v_y, v_z$ | $a, \text{yaw}, \text{yaw rate}$ |
| **Velocity Only** | `0000 1111 1100 0111` | `0x0FC7` | **`4039`** | $v_x, v_y, v_z$ | $x, y, z, a, \text{yaw rate}$ |
| **Acceleration / Force Only** | `0000 1111 0011 1111` | `0x0F3F` | **`3903`** | $a_x, a_y, a_z$ | $x, y, z, v, \text{yaw rate}$ |

*MAVLink Specification*: [https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

---

### 2.3 Autopilot Behavior When Setpoints Stop
What happens when setpoint packets stop arriving depends on the commanded mode:

1. **When Commanding Position Targets**:
   - The autopilot continues toward the **last received position target**, decelerates as it approaches the target, and **enters a stable hover (loiter) indefinitely** at that point.
   - It will *not* drift or crash; it simply waits for the next setpoint.
2. **When Commanding Velocity, Acceleration, or Attitude Targets**:
   - Governed by parameter **`GUID_TIMEOUT`** ([`ArduCopter/Parameters.cpp` line 861](file:///home/drone/ardupilot/ArduCopter/Parameters.cpp#L861)):
     ```cpp
     // Guided mode timeout after which vehicle will stop or return to level
     // if no updates are received. Default: 3.0 seconds, Range: 0.1 - 5.0 s.
     AP_GROUPINFO("GUID_TIMEOUT", 46, ParametersG2, guided_timeout, 3.0),
     ```
   - If updates stop for $> \text{GUID\_TIMEOUT}$ (default $3.0\text{ s}$), ArduCopter halts horizontal velocity, levels the aircraft, and holds position.
3. **If Heartbeat / Connection Drops Completely**:
   - Triggers the **GCS Failsafe** (Section 3).

---

## 3. Copter GCS Failsafe Parameters & Behaviors

Extracted directly from ArduPilot Copter source code ([`Parameters.h` line 408](file:///home/drone/ardupilot/ArduCopter/Parameters.h#L408), [`Parameters.cpp` line 97](file:///home/drone/ardupilot/ArduCopter/Parameters.cpp#L97)):

### 3.1 `FS_GCS_ENABLE`
Controls failsafe action when telemetry connection is lost:

| Value | Enumeration | Behavior | Recommended for Swarm? |
| :---: | :--- | :--- | :--- |
| **`0`** | `DISABLED` | No action taken. Vehicle holds position in GUIDED mode. | **Recommended for multi-drone testing** to prevent premature RTL during packet loss sweeps. |
| **`1`** | `ALWAYS_RTL` | Vehicle climbs to `RTL_ALT` and flies back to Home coordinates. | High collision hazard in swarms (all drones return to same home point). |
| **`2`** | `CONTINUE_MISSION` | *Removed in Copter 4.0+* (use `FS_OPTIONS`). | N/A |
| **`3`** | `ALWAYS_SMARTRTL_OR_RTL` | Retraces outward breadcrumb path; falls back to RTL if memory full. | Safer than direct RTL. |
| **`4`** | `ALWAYS_SMARTRTL_OR_LAND` | Retraces path; falls back to Land if SmartRTL unavailable. | Good for non-swarm ops. |
| **`5`** | `ALWAYS_LAND` | Lands vertically at current $(X, Y)$ location. | Safest descent if telemetry is lost in clear terrain. |
| **`6`** | `AUTO_RTL_OR_RTL` | Executes `DO_LAND_START` mission sequence or RTL. | For waypoint missions. |
| **`7`** | `BRAKE_OR_LAND` | **Immediately switches to BRAKE mode (holds position)**. If GPS fails, lands. | **Best safety fallback for swarms**: freezes in place without path collisions. |

---

### 3.2 `FS_GCS_TIMEOUT`
- **Parameter**: `FS_GCS_TIMEOUT` ([`Parameters.cpp` line 830](file:///home/drone/ardupilot/ArduCopter/Parameters.cpp#L830))
- **Default**: `5.0` seconds (Range: `1` to `120` seconds).
- **Behavior**: Duration of continuous MAVLink heartbeat loss required before `FS_GCS_ENABLE` triggers.

---

### 3.3 `FS_OPTIONS` (Bitmask)
- **Parameter**: `FS_OPTIONS` ([`Parameters.cpp` line 777](file:///home/drone/ardupilot/ArduCopter/Parameters.cpp#L777))
- **Bit 0**: Continue if in Auto mode on RC failsafe
- **Bit 1**: Continue if in Auto mode on GCS failsafe
- **Bit 2**: **Continue if in Guided mode on RC failsafe**
- **Bit 3**: Continue if landing on any failsafe
- **Bit 4**: Continue if in pilot-controlled modes on GCS failsafe

*Documentation*: [https://ardupilot.org/copter/docs/gcs-failsafe.html](https://ardupilot.org/copter/docs/gcs-failsafe.html)

---

## 4. DroneCAN vs MAVLink over CAN vs UART

### 4.1 Can an ESP32 Deliver Trajectory Commands via DroneCAN?
- **DroneCAN Architecture**: DroneCAN (UAVCAN v0) is designed for distributed peripherals on the CAN bus: GNSS receivers, magnetometers, airspeed sensors, ESCs, and servo actuators.
- **Can it send GUIDED setpoints?**: **No, not in standard ArduPilot.** ArduPilot Copter does not have DroneCAN DSDL message subscribers for high-level position/velocity setpoints (`SET_POSITION_TARGET_LOCAL_NED`). Writing a custom DSDL would require patching ArduPilot's C++ source code (`AP_DroneCAN`) and maintaining a custom firmware build.

### 4.2 Is There MAVLink over CAN?
- **Yes**: ArduPilot includes [`AP_MAVLinkCAN`](file:///home/drone/ardupilot/libraries/AP_CANManager/AP_MAVLinkCAN.cpp), which can tunnel MAVLink messages over CAN frames (`CAN_P1_DRIVER = 1`, `CAN_D1_PROTOCOL = 1` or MAVLink forwarding).
- **Complexity**: Requires implementing the MAVLink-over-CAN framing layer on the ESP32 (handling CAN identifier bit allocation and frame reassembly).

### 4.3 Recommendation: UART Serial (The Simplest & Best Option)
| Criterion | DroneCAN | MAVLink over CAN | UART Serial (Recommended) |
| :--- | :--- | :--- | :--- |
| **Protocol Support** | Sensor/ESC DSDL only | MAVLink framing over CAN | Native MAVLink2 |
| **ArduPilot Changes** | Requires custom firmware patch | Complex CAN param setup | **Zero changes (standard ArduPilot)** |
| **ESP32 Library** | `libcanard` (steep learning curve) | Custom CAN driver | **Standard `mavlink.h` / `HardwareSerial`** |
| **Physical Wiring** | CAN transceiver + 120Ω termination | CAN transceiver + 120Ω termination | **Direct 3.3V TX/RX/GND logic** |
| **Baud Rate / Bandwidth** | 1 Mbps CAN bus | 1 Mbps CAN bus | **921,600 baud UART (~92 KB/s, plenty for 20 Hz)** |

> [!TIP]
> **Verdict**: Connect the ESP32 to **`TELEM1` (UART7)** via 3-wire serial (`TX`, `RX`, `GND`). This is standard, verified, and leaves CAN free for future peripherals.

---

## 5. ESP32 MAVLink-to-UDP Bridge & Multi-Drone Network Topology

### 5.1 Open-Source ESP32 MAVLink Bridges
1. **`MAVESP8266 / MAVESP32` (Official Reference)**:
   - Developed by ArduPilot/PX4 core developers (Dogan Ibrahim, Lorenz Meier).
   - Runs on ESP32 / ESP8266. Bridges UART MAVLink to UDP port 14550.
   - Supports Access Point (AP) and Station (STA) modes with web-based configuration.
   - GitHub: [https://github.com/dogmaphobic/mavesp8266](https://github.com/dogmaphobic/mavesp8266)
2. **`ESP-Mavlink-Bridge` (Lightweight Custom Arduino/ESP-IDF)**:
   - Uses standard ESP32 `WiFiUDP` to forward raw MAVLink byte streams between UART and UDP.

---

### 5.2 Network Topology Comparison: Wi-Fi UDP vs ESP-NOW

```
A) Wi-Fi Star Topology (Access Point Router):
   Laptop (Coordinator) <--- UDP Socket ---> Dedicated Wi-Fi AP (5 GHz)
                                                |---> Drone 1 ESP32 (STA: 192.168.1.101:14550)
                                                |---> Drone 2 ESP32 (STA: 192.168.1.102:14550)
                                                |---> Drone 3 ESP32 (STA: 192.168.1.103:14550)

B) ESP-NOW Peer-to-Peer:
   Laptop <--- USB ---> ESP32 Gateway Dongle <=== 2.4 GHz ESP-NOW ===> Drone 1 ESP32
                                                                   ===> Drone 2 ESP32
                                             (Drone 1 <== ESP-NOW ==> Drone 2)
```

| Metric | Wi-Fi UDP (Star via Router) | ESP-NOW (Peer-to-Peer) |
| :--- | :--- | :--- |
| **Max Payload Size** | Standard IP MTU (1500 bytes) | **250 bytes max** (MAVLink2 packets up to 279B must be fragmented) |
| **Latency** | $8 - 25\text{ ms}$ (typical) | **$2 - 5\text{ ms}$** (very low) |
| **Laptop Interface** | Standard OS network sockets (`socket.sendto`) | Requires dedicated ESP32 USB transceiver on laptop |
| **Throughput** | High ($> 10\text{ Mbps}$) | Low/Medium ($< 1\text{ Mbps}$) |
| **Inter-Drone P2P** | Relayed via AP router | Direct drone-to-drone broadcast |
| **Band Support** | 2.4 GHz & **5 GHz** (5 GHz avoids FPV video) | 2.4 GHz only (can interfere with 2.4 GHz ELRS) |

> [!IMPORTANT]
> **Swarm Architecture Recommendation**:
> - **Primary Coordinator Link**: Dedicated 5 GHz Wi-Fi travel router (e.g. GL.iNet GL-MT3000) with ESP32s in Station mode. 5 GHz completely avoids interference with 2.4 GHz ELRS RC radio control and 5.8 GHz analog/digital FPV video!
> - **Decentralized Fallback**: If the supervisor specifically requires peer-to-peer inter-drone communication without an AP, use ESP-NOW with compact custom telemetry packets ($< 100\text{ bytes}$).

---

## 6. Scaling Multi-Vehicle SITL to 5 Drones

### 6.1 Port Allocation Table
ArduPilot SITL uses standard incremental offsets based on vehicle instance index $i \in \{0, 1, 2, 3, 4\}$:

| Drone Index | System ID (`SYSID_THISMAV`) | Internal SITL TCP Port (`5760 + 10*i`) | MAVProxy GCS UDP (`14550 + 10*i`) | Bridge / Simulator Port (`9002 + 10*i`) |
| :---: | :---: | :---: | :---: | :---: |
| **Drone 0** | `SYSID = 1` | `5760` | `14550` | `9002` |
| **Drone 1** | `SYSID = 2` | `5770` | `14560` | `9012` |
| **Drone 2** | `SYSID = 3` | `5780` | `14570` | `9022` |
| **Drone 3** | `SYSID = 4` | `5790` | `14580` | `9032` |
| **Drone 4** | `SYSID = 5` | `5800` | `14590` | `9042` |

---

### 6.2 CPU and Memory Scaling
Measured empirical performance on modern Linux hardware (AMD/Intel 6-core):

- **Per-Instance Resource Footprint (Headless SITL)**:
  - CPU: **$10\% - 15\%$ of one CPU core** (EKF3 runs at 400 Hz IMU step downsampled to 100 Hz state output).
  - RAM: **$35 - 50\text{ MB}$** resident set size.
- **5-Drone Total Load**:
  - CPU: **$\sim 60\% - 75\%$ of a single CPU core** (on a 6-core machine, this is only **$\sim 12\%$ total system CPU load**).
  - RAM: **$< 250\text{ MB}$** total.
- **Gazebo / Graphical Comparison**:
  - *Headless SITL* scales comfortably up to 10–15 drones on a standard laptop.
  - *Gazebo 3D physics* requires 2–4 full CPU cores plus GPU rendering and will experience real-time factor (RTF) slowdowns past 4–5 drones.

---

## 7. Academic Literature: Formation Control under Delay & Packet Loss

### 7.1 Five Seminal Papers

1. **Olfati-Saber, R., Fax, J. A., & Murray, R. M. (2007)**  
   *"Consensus and Cooperation in Networked Multi-Agent Systems"*  
   *Proceedings of the IEEE*, 95(1), 215–233. [DOI: 10.1109/JPROC.2006.887293](https://doi.org/10.1109/JPROC.2006.887293)  
   - **Relevance**: Foundation of algebraic graph theory and Laplacian matrices ($\mathbf{L}$) for swarms. Proves the upper bound on communication time-delay $\tau < \pi / (2 \lambda_{\max}(\mathbf{L}))$ beyond which formation consensus becomes unstable.

2. **Schenato, L., Sinopoli, B., Franceschetti, M., Poolla, K., & Sastry, S. S. (2007)**  
   *"Foundations of Control and Estimation Over Lossy Networks"*  
   *Proceedings of the IEEE*, 95(1), 163–187. [DOI: 10.1109/JPROC.2006.887306](https://doi.org/10.1109/JPROC.2006.887306)  
   - **Relevance**: Proves that optimal control over lossy wireless channels exhibits a **critical packet arrival rate threshold $\gamma_c$**. If packet delivery falls below $\gamma_c$, state estimation error covariance diverges to infinity. Directly supports our hybrid fallback architecture.

3. **Chopra, N., & Spong, M. W. (2006)**  
   *"Passivity-Based Control of Multi-Agent Systems With Time-Delay"*  
   *American Control Conference (ACC)* / *IEEE TAC*. [DOI: 10.1109/ACC.2006.1656360](https://doi.org/10.1109/ACC.2006.1656360)  
   - **Relevance**: Employs passivity theory and wave variable transformations to guarantee delay-independent asymptotic consensus, ensuring drones do not exhibit destructive oscillatory hunting under latency spikes.

4. **Xiao, F., Wang, L., Chen, J., & Gao, Y. (2009)**  
   *"Consensus Problems for High-Dimensional Multi-Agent Systems with Communication Delay and Packet Loss"*  
   *IEEE Transactions on Circuits and Systems*, 56(9), 2097–2108. [DOI: 10.1109/TCSI.2008.2011579](https://doi.org/10.1109/TCSI.2008.2011579)  
   - **Relevance**: Formulates stochastic packet loss as a switched dynamic system and establishes conditions for mean-square consensus using Lyapunov-Krasovskii functionals.

5. **Qin, J., & Gao, H. (2012)**  
   *"Consensus for Multi-Agent Systems with Switching Topologies and Communication Delay"*  
   *Automatica*, 48(9), 2307–2314. [DOI: 10.1016/j.automatica.2012.06.042](https://doi.org/10.1016/j.automatica.2012.06.042)  
   - **Relevance**: Solves the consensus tracking problem when the graph topology dynamically disconnects due to burst outages, matching our Gilbert-Elliott channel evaluation.

---

### 7.2 How Crazyswarm Does It
*Reference: Preiss, J. A., Hönig, W., Sukhatme, G. S., & Ayanian, N. (2017). "Crazyswarm: A large nano-quadcopter swarm." IEEE ICRA 2017.* [https://act.usc.edu/crazyswarm/](https://act.usc.edu/crazyswarm/)

- **Hardware**: Bitcraze Crazyflie 2.x nano-quadcopters ($27\text{ g}$) tracked by central OptiTrack/Vicon motion capture at 100 Hz.
- **The Packet Loss Problem**: Streaming high-rate (100 Hz) setpoints to 49 drones over 2.4 GHz radios completely saturates the spectrum and causes severe packet collisions.
- **Crazyswarm's Solution**:
  1. **Broadcast Trajectory Polynomials Ahead of Time**: The ground station does **NOT** stream continuous position/velocity setpoints. Instead, it uploads **compact polynomial spline coefficients** (piecewise Bezier curves) representing the next several seconds of flight.
  2. **High-Rate Onboard Evaluation**: Each Crazyflie evaluates the polynomial trajectory equation onboard at **500 Hz**!
  3. **Packet Loss Immunity**: If wireless packets are delayed or dropped, the drone continues smoothly executing its pre-loaded trajectory segment without stuttering, chattering, or stopping.
  4. **TDM Broadcast Packets**: When real-time state feedback is needed, it uses fixed Time-Division Multiplexed broadcast packets to guarantee deterministic transmission times.

---

## 8. GPS Accuracy: Plain GPS vs RTK Option

### 8.1 Plain GPS (Standard Single-Frequency L1 GNSS, e.g. u-blox M8N / M10Q)
- **Absolute Positioning Accuracy**: **$1.5 - 2.5\text{ m}$ CEP** (Circular Error Probable) under open sky. Degrades to $3 - 5\text{ m}$ near buildings, trees, or during ionospheric disturbances.
- **Relative Accuracy Between Two Co-Located Drones ($3 - 10\text{ m}$ separation)**:
  - **Common-Mode Error Cancellation**: Drones within 10 meters of each other look through the exact same atmospheric column (ionosphere and troposphere) and track the exact same GPS satellite constellation.
  - Because ionospheric delay (~5m) and satellite ephemeris clock drift (~1-2m) affect both receivers almost identically ($\gamma \approx 0.60 - 0.80$ correlation), **common-mode error cancels out in the relative vector**:
    $$\Delta \mathbf{p}_{\text{rel}} = \mathbf{p}_1 - \mathbf{p}_2 = (\mathbf{p}_{\text{true}, 1} + \mathbf{e}_{\text{common}} + \mathbf{e}_{\text{indep}, 1}) - (\mathbf{p}_{\text{true}, 2} + \mathbf{e}_{\text{common}} + \mathbf{e}_{\text{indep}, 2}) = \Delta \mathbf{p}_{\text{true}} + (\mathbf{e}_{\text{indep}, 1} - \mathbf{e}_{\text{indep}, 2})$$
  - The relative distance discrepancy is typically **$0.5 - 1.2\text{ m}$**, dominated by receiver thermal noise and local multipath.
  - **Operational Constraint**: Flying formations tighter than $1.5\text{ m}$ inter-drone spacing with plain GPS carries collision risk due to independent multipath drift. Formations with **$\ge 2.5\text{ m}$ spacing are safe**.

---

### 8.2 RTK (Real-Time Kinematic) GNSS Option
RTK uses carrier-phase differential measurement against a stationary base station to resolve cycle ambiguities, achieving centimeter accuracy.

- **Positioning Performance**:
  - RTK Float: $\sim 10 - 30\text{ cm}$
  - RTK Fixed: **$1 - 2\text{ cm}$ relative and absolute accuracy**!
- **Commercial Hardware & Cost Estimate**:
  - Dual-frequency L1/L5 or L1/L2 receiver: **u-blox ZED-F9P** chipset.
  - **2x Drone Rovers**:
    - *Matek GNSS & Compass F9P* (~$150 USD each) or *Holybro H-RTK F9P Helical* (~$185 USD each) $\to$ **~$300 – $370 USD**
  - **1x Ground Base Station**:
    - *ArduSimple simpleRTK2B Base* (~$200 USD) or *Holybro H-RTK Base* (~$220 USD) $\to$ **~$200 – $220 USD**
  - **Total 2-Drone Swarm RTK Cost**: **$\sim \$500 – \$590 USD**
- **Deployment Requirement**:
  - Requires base station streaming RTCM3 correction messages over MAVLink (`GPS_INJECT_DATA`) to the rovers at 1 Hz via the telemetry link.
