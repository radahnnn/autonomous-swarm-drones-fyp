# DroneCAN Serial-Tunnel <-> Wi-Fi UDP MAVLink2 Bridge

Zero-solder MAVLink2 bridge between the **Matek H743-SLIM V3** (`CAN1` 4-pin JST-GH) and the ground station / `IndependentAgentNode` over Wi-Fi UDP.

> **Evidence status:** `dronecan_tunnel_codec.h` is **UNIT-tested** on the host against ArduPilot's `libcanard` (`uavcan.tunnel.Targetted` and `uavcan.protocol.NodeStatus`). `dronecan_mavlink_bridge.ino` is **NOT YET COMPILED OR TESTED ON PHYSICAL HARDWARE** — complete `hardware/esp32/twai_diagnostic/` (both RX and TX) first before flashing this bridge.

---

## 1. Matek H743-SLIM V3 Parameters (Set Over USB in QGroundControl)

1. Connect the Matek over USB (props OFF, battery disconnected) and set:
   - `CAN_P1_DRIVER = 1`
   - `CAN_P1_BITRATE = 250000` *(match `CAN_BITRATE_KBPS` in the `.ino`; once verified at `250`, you can optionally bump both sides together to `500000` or `1000000`)*
   - `CAN_D1_PROTOCOL = 1`
   - `CAN_D1_UC_NODE = 10`
   - `CAN_D1_UC_SER_EN = 1`
2. **Write & reboot the Matek**, then refresh parameters and set:
   - `CAN_D1_UC_S1_NOD = 50` *(matches `ESP32_CAN_NODE_ID`)*
   - `CAN_D1_UC_S1_IDX = 0` *(matches `SERIAL_ID`)*
   - `CAN_D1_UC_S1_PRO = 2` *(MAVLink2)*
   - `CAN_D1_UC_S1_BD = 115200`

---

## 2. Drone 1 vs. Drone 2 Settings

| Aircraft | Matek `SYSID_THISMAV` | ESP32 `GCS_UDP_PORT` | Agent Command (`--endpoint`) |
|---|---|---|---|
| **Drone 1** | `1` | `14551` | `python3 -m sitl.independent_agent_node ... --endpoint udpin:0.0.0.0:14551 --mavlink-system-id 1` |
| **Drone 2** | `2` | `14552` | `python3 -m sitl.independent_agent_node ... --endpoint udpin:0.0.0.0:14552 --mavlink-system-id 2` |

---

## 3. Flashing & Testing (Props OFF!)

1. Edit the `USER CONFIGURATION` block at the top of `dronecan_mavlink_bridge.ino` (`WIFI_SSID`, `WIFI_PASS`, `GCS_IP`, `GCS_UDP_PORT`, `CAN_TX_PIN`, `CAN_RX_PIN`).
2. Keep the flight battery disconnected while flashing. In Arduino IDE, select board **`ESP32 Dev Module`** (no external libraries required — uses built-in `WiFi`, `WiFiUdp`, and ESP-IDF `driver/twai.h`) and upload.
3. With **all 4 propellers removed**, connect the LiPo (`VBAT` powers Matek `CAN1`) and open Serial Monitor at `115200` baud:
   - Look for `wifi=OK`, `can=RUNNING`, and `rx_b` / `udp_tx` climbing while `crc_err=0`, `rx_err=0`, `tx_err=0`, `bus_err=0`.
   - Open QGroundControl (UDP link on `14551`) or run `python3 -m pymavlink.tools.mavproxy` / `IndependentAgentNode` against `udpin:0.0.0.0:14551` and verify live `HEARTBEAT` + telemetry and bidirectional parameter reads.
