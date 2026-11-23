# Bench session check - ESP32 / CAN link (prepared 2026-11-23, NOT YET RUN)

## Status
No measurement was taken. The Ubuntu laptop was off and unreachable when this was prepared, so **nothing in this document is a
result**. It records the decision on how to test safely and gives a ready-to-run read-only procedure.
Evidence label for everything here: none yet (procedure only).

## Safety decision: propellers could not be removed
- Active tests (opening the ESP32 serial port, any CAN or MAVLink traffic) stay **off the table while propellers are fitted and the
  flight battery is connected**. The Oct 29 plan already stopped for this reason (a CP2102 serial open can toggle DTR/RTS and reset an
  ESP32 whose firmware behaviour is unknown).
- Propellers can stay on **only if the flight battery is disconnected**: the motors are powered from the battery, not from USB, so
  with USB-only power the Matek and ESP32 run but the motors cannot spin. This is the "logic-only USB test" (step 2 of the Oct 29 plan).
- Do not connect the flight battery with propellers fitted for this check. Keep the aircraft disarmed; nobody near the motors.

## Procedure (read-only; no serial port opened; no arm/motor/command message)
1. Flight battery disconnected. Plug the ESP32 USB cable into the Ubuntu laptop, then the Matek USB-C (separate steps, note the time).
2. On the laptop: `hardware/scripts/passive_usb_check.sh 10` (10 minutes). It lists USB serial devices and CAN interfaces every 5 s,
   counts disappear/reappear events and prints PASS/FAIL. Log goes to `./bench_logs/` (git-ignored).
3. Matek over USB, read-only: `python3 hardware/scripts/read_drone_info.py` (requests firmware info and a telemetry stream only) and
   `python3 hardware/scripts/export_parameters.py` (downloads parameters). Compare `CAN_P1_DRIVER`, `CAN_P1_BITRATE`,
   `CAN_D1_PROTOCOL`, `CAN_D1_UC_NODE` against `hardware/params/matek_h743_quad_20261023.param`.
4. Save the parameter export, the log and a photo of the wiring with the Git commit used.

## What each outcome allows you to claim
| Observation | Allowed claim | Not allowed |
|---|---|---|
| Both devices enumerate, stable 10 min, no disconnects | PHYSICAL USB enumeration, stable | ESP32 firmware running, CAN works, command path |
| Matek parameters read over USB match the Oct 23 export | Matek CAN configuration confirmed | any CAN traffic |
| A CAN capture or ESP32/ArduPilot counters show valid frames at 250 kbit/s from an identified node, acceptable error state | PHYSICAL CAN traffic | command path, flight readiness |

## Limits of this check (be honest in the report)
- The laptop has no CAN interface. CAN is between the ESP32 (TWAI + transceiver) and the Matek, so this procedure **cannot by itself show CAN
  traffic**. To verify it you need (a) the ESP32 firmware printing bus state, TX/RX counts and error counters (needs a serial monitor, so
  only with the safe conditions above), or (b) a DroneCAN view on the Matek side, e.g. QGroundControl > Vehicle Setup > DroneCAN, or
  (c) a USB-CAN adapter on the bus. Until one of these exists the CAN link stays "configuration only".
- Which ESP32 firmware is flashed, its CAN pins, transceiver supply and logic levels (MCP2551 is a 5 V part) are still unrecorded.
- Matek side: `SERIAL1` (Telemetry 1) is 57,600 baud; the agent examples use 115,200. Match `SERIALx_PROTOCOL`/`SERIALx_BAUD` to the UART used.
- Open safety settings from the Oct 6/23 exports are unchanged here: `ARMING_CHECK 112`, `FS_GCS_ENABLE 0`, no battery failsafe,
  `FENCE_ENABLE 0`. Not part of this check; needed before any flight (readiness checklist gate 1.3).

## Next session
Run the procedure above, then report PASS/FAIL per row with the log. If the CAN question must be answered, bring the ESP32 firmware
source/hash and decide between (a), (b) and (c) before connecting anything else.
