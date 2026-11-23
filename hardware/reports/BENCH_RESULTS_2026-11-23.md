# Bench results - ESP32 + Matek on the Windows laptop (2026-11-23)

Conditions: flight battery **disconnected**; Matek and ESP32 powered by USB only (propellers fitted but the motors have no power);
aircraft disarmed. All checks were passive or read-only: no arm, no motor, no CAN transmit, no parameter write, no firmware flash.
Tools: Windows built-in device queries and .NET serial port (reset lines DTR/RTS held off); a hand-built MAVLink2 `PARAM_REQUEST_READ`.

## Evidence (label: PHYSICAL)
| # | Observation | Result |
|---|---|---|
| 1 | ESP32 USB-UART bridge CP2102 `10C4:EA60` serial `0001` | Enumerates. Windows first reported driver error 28 (no driver); after installing the Silicon Labs CP210x driver (11.6.0.420, signed) it is **COM3**, status OK. Stable for 182 s (31 samples, no disconnect). |
| 2 | ESP32 firmware output on COM3 @115200 (921600 gives garbage) | Runs. Over 15 s the **only** line text was `Waiting for CAN packets...` (6,998 lines, 1 distinct text). It prints no bitrate, pins, counters or error states. |
| 3 | Matek USB `1209:5740`, serial `180041000F51333531333639` (same as the Oct 6 README) | Enumerates as a composite device: **COM5** (MAVLink) and COM6 (not used). |
| 4 | Matek MAVLink heartbeat on COM5 (passive, 8 s) | sys 1 / comp 1, type 2 (quadrotor), autopilot 3 (ArduPilot), **disarmed**, system status standby, mode 0, EKF3 active. Pre-arm notices unchanged: `RC not found`, `3D Accel calibration needed`. |
| 5 | Matek live parameters (read-only request) | `CAN_P1_DRIVER 1`, `CAN_P1_BITRATE 250000`, `CAN_D1_PROTOCOL 1`, `CAN_D1_UC_NODE 10`, `SYSID_THISMAV 1`, `ARMING_CHECK 112`, `FS_GCS_ENABLE 0`. Identical to the Oct 23 export. The board has no answer for `CAN_P2_DRIVER` / `CAN_D2_PROTOCOL` (one CAN port). |

## Conclusion
- USB links: both devices enumerate and respond. Stability only observed over minutes.
- **CAN link: NOT working / not verified.** The Matek is configured to run DroneCAN on CAN port 1 at 250 kbit/s, yet the ESP32 received **zero frames**.
  The cause is not determined. Candidates, cheapest first: wiring (CAN-H/CAN-L, ground), transceiver supply (MCP2551 is a 5 V part) and logic levels to the
  ESP32, the ESP32 TWAI pins / bitrate / mode in the sketch, termination, or the Matek not actually transmitting.
- The current ESP32 sketch cannot distinguish these (it prints no TWAI status or error counters).
- Safety settings unchanged (`ARMING_CHECK 112`, `FS_GCS_ENABLE 0`, no battery failsafe, fence off). Nothing here shows flight readiness.

## Needed to resolve CAN (open questions)
ESP32 board model; GPIOs wired to the transceiver TX/RX; transceiver module and its supply voltage; how the Matek CAN pads/connector are wired (H, L, GND, 5 V);
whether either end is terminated; the ESP32 sketch source. A diagnostic sketch that prints TWAI state, TX/RX error counters, bus-error and RX-missed counts every
second would show whether the bus is electrically alive (flashing it needs the team's approval; the Oct 29 plan states no firmware has been flashed).

## Recommendation
Centralized control over MAVLink via a connectorized UART does not depend on CAN (see `HARDWARE_SOFTWARE_MAPPING.md`). Treat CAN as an optional experiment and do
not block the UART/MAVLink path on it.
