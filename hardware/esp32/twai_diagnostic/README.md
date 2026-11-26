# TWAI (CAN) diagnostic sketch

Goal: show whether the ESP32 <-> MCP2551 (`MCU-2551`) <-> Matek CAN bus is electrically alive in **both** directions at 250 kbit/s and whether the Matek's DroneCAN node 10 is heard. Not a command bridge. Not compiled or tested on physical hardware yet; fix any compile error before use.

## Critical Hardware Note (Matek H743-SLIM V3)
- On the Matek H743-SLIM V3, the CAN1 5 V rail and onboard CAN transceiver are powered from the **5 V BEC (LiPo `VBAT`), NOT from USB**.
- On USB-only power, Matek CAN1 is silent even when configured properly.
- **Remove all 4 propellers physically** before connecting the LiPo battery (`VBAT`) to run this diagnostic.

## Two-Step Check
1. **RX check (`LISTEN_ONLY = 1`, default):** verifies ESP32 receives Matek `NodeStatus` (node 10) at 250 kbit/s.
2. **TX check (`LISTEN_ONLY = 0`, `SEND_TEST_NODESTATUS = 1`):** verifies ESP32 ACKs frames and transmits `NodeStatus` as node 50 (`tx_ok` climbs, `tx_err=0`).

## Reading the output (115200 baud)
| Output | Meaning | Next check |
|---|---|---|
| `frames_total` rises, `DroneCAN ... NodeStatus ... from node 10`, `rx_err=0` | RX alive, bitrate right, Matek transmitting | Switch to `LISTEN_ONLY 0` to verify TX |
| `LISTEN_ONLY 0`: `tx_ok` climbs, `tx_err=0`, `state=RUNNING` | **Two-way CAN works!** | Proceed to `hardware/esp32/dronecan_mavlink_bridge/` |
| RX passes, `LISTEN_ONLY 0` has `tx_err` climbing / `BUS_OFF` | `MCU-2551 CTX` is rejecting 3.3 V HIGH (`0.7 * VDD` spec) | Add a breadboard `74HCT08` / `74HCT125` / `BSS138` level shifter (or `1N4007` diode on `MCU-2551 VCC`) |
| `frames_total` = 0, `rx_err` and `bus_err` = 0 | nothing on the bus | Verify `VBAT` (LiPo) is plugged in (props off!), wiring (`CANH/CANL/GND`), pin numbers |
| `bus_err` or `rx_err` climbing, no frames | signal present but unreadable | bitrate mismatch, `CANH/CANL` swapped, missing `120 ohm` termination, common `GND` |
| `state=BUS_OFF` | persistent errors | unplug battery and recheck wiring |

## Before flashing
- Flight battery disconnected. Flashing replaces the current firmware (`"Waiting for CAN packets..."`): save a flash dump with `esptool` first if you want it back.
- Check `CAN_TX_PIN` and `CAN_RX_PIN` at the top of `twai_diagnostic.ino` against your wiring and verify the resistor voltage divider on `CRX`.
