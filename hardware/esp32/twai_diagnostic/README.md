# TWAI (CAN) diagnostic sketch

Goal: show whether the ESP32 <-> MCP2551 <-> Matek CAN bus is electrically alive at 250 kbit/s and whether the Matek's DroneCAN
node 10 is heard. Not a command bridge. Not tested on hardware yet (written 2026-11-23).

## Reading the output (115200 baud)
| Output | Meaning | Next check |
|---|---|---|
| `frames_total` rises, `DroneCAN ... NodeStatus ... from node 10` lines | bus alive, bitrate right, Matek transmitting | go on to node discovery / DroneCAN node |
| `frames_total` = 0, `rx_err` and `bus_err` = 0 | nothing on the bus | wiring (CANH/CANL/GND), module power, Matek CAN port/connector, Matek not transmitting |
| `bus_err` or `rx_err` climbing, no frames | signal present but unreadable | bitrate mismatch, CANH/CANL swapped, missing termination, ground |
| `state=BUS_OFF` | persistent errors | same as above; unplug and recheck wiring |
| frames but `NodeStatus` never appears | other traffic present | check data-type decoding, node id |

## Before flashing
- Flight battery disconnected. Flashing replaces the current firmware ("Waiting for CAN packets..."): save the old source or a flash dump first if you want it back.
- Check the pin constants at the top of the sketch against your wiring and the level divider on CRX (see the header comment).
