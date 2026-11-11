# Physical Drone Hardware Integration (Matek H743-SLIM V3)

This directory contains configuration, parameter backups, diagnostic scripts, and documentation for the physical swarm quadcopter hardware connected and verified via USB-C MAVLink.

---

## 1. Hardware Overview & Identification

* **Flight Controller:** **Matek Systems H743-SLIM V3**
  * **Processor:** STM32H743VIH6 @ 480 MHz
  * **IMU:** Dual InvenSense ICM-42688-P
  * **Barometer:** Infineon DPS368
  * **Hardware UUID / Serial:** `180041000F51333531333639`
  * **USB Vendor/Product ID:** VID `0x1209` / PID `0x5740`
* **Autopilot Firmware:** **ArduCopter v4.6.2** (Git hash: `31656264`, ArduPilot on ChibiOS)
* **Airframe:** **Quadcopter** (`MAV_TYPE_QUADROTOR`)
  * Layout: Betaflight-style X frame (`FRAME_CLASS = 1`, `FRAME_TYPE = 12`)
  * ESC Protocol: **DShot600** digital motor signaling (`MOT_PWM_TYPE = 6`)
  * Battery Monitoring: Analog current & voltage sensing (`BATT_MONITOR = 4`)

---

## 2. Diagnostics & Bench Verification Results (Oct 6, 2026)

* **Arming State:** `DISARMED` (powered safely via USB bus power, 0.00 V main battery reading).
* **Sensors Status:**
  * 3D Gyroscope: **HEALTHY**
  * 3D Accelerometer: **HEALTHY**
  * 3D Magnetometer / Compass: **HEALTHY**
  * Barometer (Pressure): **HEALTHY**
  * GPS Module: **HEALTHY** (Active **3D Fix** with **23 satellites** in view, altitude 467.1 m)
* **Pre-Arm Status:** Reported expected bench notices:
  * `PreArm: RC not found` (Transmitter unpowered)
  * `PreArm: 3D Accel calibration needed`

---

## 3. MicroSD Card & Blackbox Logging Setup

* **Socket Location:** On the **Matek H743-SLIM V3**, the push-push MicroSD slot is soldered to the **UNDERSIDE (bottom)** of the blue flight controller board, inside the ~3–4 mm gap between the flight controller and the 4-in-1 ESC board.
* **Why QGroundControl Reported "List directory failed":**
  * Parameter `LOG_DISARMED = 0` means ArduPilot will **only create `/APM/LOGS` and record logs after the drone is armed**.
  * If the card slot is empty or the drone has never been armed, no `/APM/LOGS` directory exists on the SD card.
* **To Enable Flight Logging:**
  1. Insert a **FAT32-formatted** MicroSD card (8 GB to 32 GB, Class 10/U1) into the underside socket with the **gold contacts facing UPWARDS** toward the blue board.
  2. Once armed in flight, ArduPilot automatically creates `/APM/LOGS` and saves `.bin` flight logs.

---

## 4. Directory Structure

```text
hardware/
├── README.md                          # This hardware documentation
├── Matek_H743_Parameter_Configuration_Summary.docx # Parameter summary (generated Oct 14)
├── reports/
│   └── PHYSICAL_ESP32_CAN_BENCH_PLAN_2026-10-29.md # Evidence, interface decision and safe bench plan
├── params/
│   ├── matek_h743_quad_20261006.param # Clean backup of all 1,283 flight parameters (Oct 6)
│   └── matek_h743_quad_20261023.param # Later export (Oct 23): CAN port enabled (CAN_P1_DRIVER=1, DroneCAN node 10)
└── scripts/
    ├── read_drone_info.py             # Live MAVLink telemetry & diagnostic reader
    ├── export_parameters.py           # Tool to download and dump parameters from FC
    └── launch_qgroundcontrol.sh       # Wayland/X11 launcher for QGroundControl v5.1.5
```

---

## 5. Usage Instructions

### Reading Live Telemetry & Health Checks
Connect the flight controller via USB-C (creates `/dev/ttyACM0`) and run:
```bash
python3 hardware/scripts/read_drone_info.py
```

### Backing Up / Exporting Parameters
To pull the latest parameters from the flight controller into a `.param` file:
```bash
python3 hardware/scripts/export_parameters.py
```

### Launching QGroundControl GUI
```bash
bash hardware/scripts/launch_qgroundcontrol.sh
```

---

## 6. ESP32/CAN Bench Status (Oct 29, 2026)

**PHYSICAL, passive USB evidence:** the team's ESP32 board enumerated on Ubuntu as a Silicon Labs
CP2102 USB-to-UART bridge (`10c4:ea60`) at `/dev/ttyUSB0`, with a stable `/dev/serial/by-id/...`
name. No process held the port. Linux exposed no SocketCAN interface or USB-to-CAN adapter.

The serial port was deliberately not opened because the airframe still had propellers installed and
the flight battery was connected. No CAN frame, ESP32 firmware behavior, Matek node discovery, or
command path was verified. CAN remains **configuration only**: the Oct 23 export records DroneCAN on
CAN port 1 at 250 kbit/s with autopilot node 10, but contains no traffic evidence.

The recommended first command path is connectorized **UART/MAVLink** if an approved no-solder UART
harness is available. CAN should be validated separately as an experimental DroneCAN link with a
defined ESP32 protocol. See
[`reports/PHYSICAL_ESP32_CAN_BENCH_PLAN_2026-10-29.md`](reports/PHYSICAL_ESP32_CAN_BENCH_PLAN_2026-10-29.md).
