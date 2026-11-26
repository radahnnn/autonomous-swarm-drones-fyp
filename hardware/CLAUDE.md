# Hardware Integration Instructions for Claude Code (`hw/integration`)

This file is for the **hardware bench workstation (Windows + Claude Code)** working on the BEE-60 swarm-drone FYP.

## 1. Branch & Folder Ownership (Zero-Conflict Two-PC Rule)

- **Active hardware branch:** `hw/integration`
- **Your write scope:** **ONLY files inside `hardware/`** (`hardware/esp32/`, `hardware/params/`, `hardware/scripts/`, `hardware/reports/`).
- **Never edit** `sitl/`, `swarm_core/`, `simulator/`, `tests/`, root `CLAUDE.md`, `docs/PROJECT_STATE.md`, or `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`. Those belong to the Ubuntu software/simulation branch (`wip/decentralized-sitl-handoff-20261029`).
- **Do read for context:** `docs/PROJECT_STATE.md`, `docs/HARDWARE_READINESS_CHECKLIST.md`, `docs/PHYSICAL_NETWORK_VALIDATION_PLAN.md`, and `sitl/independent_agent_node.py`.
- **Never delete** existing files in `hardware/`, `hardware/.git`, logs, parameter backups, or branches.

## 2. Locked Hardware Architecture & Constraints

- **Flight Controller:** Matek H743-SLIM V3 running **ArduCopter 4.6.2** (`SYSID_THISMAV=1` on Drone 1, `2` on Drone 2).
- **Companion Node:** **ESP32-WROOM-32U** (CP2102 USB-to-UART bridge, `10C4:EA60`, 3.3 V GPIO) + **`MCU-2551` (MCP2551, 5 V)** CAN transceiver with the groupmate's **resistor voltage divider on `CRX` → ESP32 RX**, plugged into the Matek's **4-pin JST-GH `CAN1` port** (`5V / CAN-H / CAN-L / G`).
- **Hard physical constraints:**
  - **NO soldering** on the Matek H743-SLIM V3.
  - **NO `SN65HVD230`** 3.3 V CAN transceiver available locally.
- **Why the 2026-11-23 USB-only CAN test saw 0 packets:**
  - On the Matek H743-SLIM V3, the CAN transceiver and `CAN1` 5 V pin are powered **ONLY from `VBAT` (LiPo battery via the onboard 5 V BEC), NOT from USB VBUS**.
  - USB-only power runs the STM32H743 MCU and USB MAVLink (`COM5`), **not** the CAN bus. Testing CAN requires `VBAT` connected (with propellers physically removed!).
- **Breadboard-only `CTX` backup (only if `LISTEN_ONLY=0` TX test shows `tx_err > 0`):**
  - The `MCU-2551` input threshold is `0.7 × VDD` (~3.5 V at `VCC = 5.0 V`). At room temperature 3.3 V often works as-is; test her existing wiring first.
  - If TX fails (`tx_err` climbs while RX works cleanly), add one of these breadboard-only fixes between ESP32 TX and `MCU-2551 CTX` (no Matek soldering):
    1. **`74HCT08`** (tie both inputs to ESP32 TX) or **`74HCT125`** DIP IC from the MCS DLD lab (`VCC=5V`, triggers at `2.0 V`, outputs `5.0 V`), OR
    2. A standard **4-channel `BSS138` 3.3V↔5V bi-directional level shifter** module, OR
    3. A **`1N4007` diode** in series with `MCU-2551 VCC` (`~4.5 V` supply → `0.7 × VDD ≈ 3.15 V < 3.3 V`).

## 3. Non-Negotiable Safety Rules

1. **REMOVE ALL 4 PROPELLERS PHYSICALLY** with a prop wrench before plugging in the LiPo battery (`VBAT`) for any bench test.
2. **Never send arm or motor-test commands** until Gate 1 in `docs/HARDWARE_READINESS_CHECKLIST.md` is completed and explicitly approved.
3. **Back up existing ESP32 flash before overwriting** the `"Waiting for CAN packets..."` firmware:
   `esptool --port COM3 read_flash 0x0 0x400000 esp32_backup_4mb.bin` (keep backups local, do not commit `.bin` files to Git).
4. **Always keep `ARMING_CHECK`, failsafes, and fence review intact.** From `hardware/params/matek_h743_quad_20261023.param`, these still need fixing before Gate 1 flight sign-off:
   - `ARMING_CHECK = 112` → must be `1` (All)
   - `FENCE_ENABLE = 0` → must be `1`
   - `BATT_FS_LOW_ACT = 0`, `BATT_FS_CRT_ACT = 0` → must be configured for actual LiPo cell count
   - `FS_GCS_ENABLE = 0` → must be enabled (Land/RTL on companion heartbeat loss)

## 4. Ordered Bench Milestones

1. **Step 1 — Prove CAN RX & TX alive (Props OFF, `VBAT` connected, disarmed):**
   - Record the actual ESP32 GPIO pins wired to `MCU-2551` `CTX` / `CRX` and confirm common `GND` + `120 ohm` termination.
   - Flash `hardware/esp32/twai_diagnostic/twai_diagnostic.ino`:
     - First with `LISTEN_ONLY = 1`: confirm `DroneCAN msg type 341 from node 10 = NodeStatus` arrives every 1 s with `rx_err=0`, `bus_err=0`.
     - Next with `LISTEN_ONLY = 0` (`SEND_TEST_NODESTATUS = 1`): confirm `state=RUNNING`, `tx_ok` increments, and `tx_err=0`.
2. **Step 2 — MAVLink2 over DroneCAN ↔ Wi-Fi UDP (Props OFF):**
   - In QGroundControl over Matek USB, set the 5 DroneCAN serial-tunnel parameters (`CAN_D1_UC_SER_EN=1`, reboot, then `CAN_D1_UC_S1_NOD=50`, `CAN_D1_UC_S1_IDX=0`, `CAN_D1_UC_S1_PRO=2`, `CAN_D1_UC_S1_BD=115200`; keep `CAN_P1_BITRATE=250000` initially).
   - Flash `hardware/esp32/dronecan_mavlink_bridge/dronecan_mavlink_bridge.ino` with your Wi-Fi SSID/password and laptop IP.
   - Verify QGroundControl and `sitl/independent_agent_node.py --endpoint udpin:0.0.0.0:14551 --mavlink-system-id 1` receive live `HEARTBEAT` and telemetry over Wi-Fi UDP.
   - Optionally raise `CAN_P1_BITRATE` on Matek and `CAN_BITRATE_KBPS` on ESP32 together from `250` → `500` or `1000` kbit/s once stable.
3. **Step 3 — Gate 1 Pre-Flight Bench (Props OFF):**
   - Connect and bind RC receiver (`SERIAL7`, `Tx6/Rx6/4V5/G` — do NOT disturb that wiring), calibrate 3D accel + compass in QGroundControl, insert FAT32 8–32 GB MicroSD card on the underside of the Matek, configure failsafes, verify motor order/direction with props OFF, and export updated `.param` files to `hardware/params/`.
4. **Step 4 — Flight Progression & Drone 2:**
   - Drone 1 manual hover (collect 5 min `.BIN` log to measure real GNSS error) → Drone 1 single-aircraft `GUIDED` hover → replicate exact configuration on **Drone 2** (`SYSID_THISMAV=2`, UDP `14552`).

## 5. Git & Reporting Rules

- **Evidence labels (always honest):**
  - `UNIT`: host logic/codec test only (e.g., `dronecan_tunnel_codec.h` verified against `libcanard`).
  - `PHYSICAL`: real bench observation on real hardware (state exact conditions: props on/off, battery connected/disconnected, port, baud, raw numbers).
  - Never mark an `.ino` sketch as compiled/tested unless it actually compiled and ran on real hardware.
- **After every bench session:**
  - Write a dated report in `hardware/reports/BENCH_RESULTS_YYYY-MM-DD.md`.
  - Stage and commit **only** under `hardware/`:
    ```bash
    git status
    git add hardware/
    git commit -m "hardware: <short summary>"
    git push origin hw/integration
    ```
  - Never force-push (`--force`), never `git reset --hard`, never commit binary `.bin` flash dumps.
  - To pull software updates from Ubuntu: `git fetch origin && git merge origin/wip/decentralized-sitl-handoff-20261029`.
