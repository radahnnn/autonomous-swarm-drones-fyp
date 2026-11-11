# Physical ESP32/CAN bench readiness and validation plan — 2026-10-29

## Scope and evidence language

This report separates observations made with real hardware from configuration records and proposed
tests. `PHYSICAL` below means that a real device was connected to the laptop. It does not imply that
the CAN bus, MAVLink command path, motors, or flight were tested.

No firmware was flashed, no flight-controller parameter was changed, no serial port was opened, and
no arm or motor command was sent in this session.

## PHYSICAL evidence collected

On 2026-10-29 the team's ESP32 development board was connected to the Ubuntu laptop by USB:

- Linux enumerated a Silicon Labs CP2102 USB-to-UART bridge (`10c4:ea60`).
- The serial device was `/dev/ttyUSB0`; its persistent name was
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
- No process owned the serial device at inspection time.
- Linux exposed no SocketCAN interface (`can0`, `slcan0`, or equivalent) and no USB-to-CAN adapter.
- The Matek was not connected to the laptop by USB during this observation.

This proves only that the ESP32 board's USB-UART interface and cable enumerated. It does **not**
prove that ESP32 application firmware is running, that TWAI is configured, that the MCP2551 is
electrically compatible, that CAN frames are present, or that ArduPilot accepts commands from it.

The active CAN/serial test was stopped because the airframe still had propellers installed and the
flight battery was connected. Opening a CP2102 serial port can change DTR/RTS and reset an ESP32;
the behavior of the firmware after reset is currently unknown. The next active test requires
propellers removed and the aircraft disarmed.

## What the saved Matek configuration establishes

The 2026-10-23 export records:

| Parameter | Saved value | Meaning for this project |
| --- | ---: | --- |
| `CAN_P1_DRIVER` | `1` | Physical CAN port 1 is assigned to CAN driver 1 after reboot. |
| `CAN_D1_PROTOCOL` | `1` | Driver 1 uses DroneCAN. |
| `CAN_P1_BITRATE` | `250000` | Any attached CAN node must use 250 kbit/s for this saved configuration. |
| `CAN_D1_UC_NODE` | `10` | Node ID used by the autopilot on this DroneCAN network; it is not proof of an ESP32 node. |
| `CAN_D1_UC_SER_EN` | `0` | DroneCAN serial passthrough is disabled. |
| `CAN_D1_UC_ESC_BM` | `0` | No DroneCAN ESC output channels are selected. |
| `CAN_D1_UC_SRV_BM` | `0` | No DroneCAN servo output channels are selected. |
| `CAN_LOGLEVEL` | `0` | No extra CAN diagnostic logging is requested by this setting. |

These are configuration facts only. The export contains no receive count, node-discovery result,
bus-error count, or captured CAN frame.

ArduPilot separates the physical CAN interface, its selected protocol driver, and the application
that consumes protocol messages. Consequently, arbitrary CAN frames do not become MAVLink GUIDED
commands. See the official [ArduPilot CAN setup](https://ardupilot.org/copter/docs/common-canbus-setup-advanced.html)
and [DroneCAN serial passthrough](https://docs.ardupilot.org/copter/docs/common-dronecan-serial.html)
documentation.

## Recommended command path decision

**First command-path milestone: connectorized UART carrying MAVLink between the ESP32 and Matek.**

Reasons:

1. The current swarm integration already uses MAVLink semantics for heartbeat, telemetry, system
   identity, modes, acknowledgements, and bounded GUIDED setpoints.
2. UART preserves that interface with less protocol translation than implementing a DroneCAN node.
3. The saved Matek CAN configuration selects DroneCAN, not a transparent MAVLink command channel.
4. The current Linux/Python agent will not run unchanged on the ESP32; ESP32 firmware must have an
   explicitly scoped role such as a MAVLink-to-network bridge or a deliberately ported onboard
   agent.

This recommendation is conditional on the team identifying an unused UART that is available through
an approved plug-in harness. The no-solder rule remains binding. Record the Matek connector name,
pin order, voltage level, ArduPilot `SERIALx` mapping, protocol, baud rate, and power source before
connection.

**CAN milestone: validate it as a separate experimental DroneCAN link.** CAN may still be useful if
the project deliberately implements a compatible DroneCAN node or an approved adapter-node/serial
passthrough design. That requires a defined application protocol and cannot be inferred from
`CAN_P1_DRIVER=1`.

## Electrical questions that must be answered before CAN traffic testing

- Exact ESP32 GPIO pins used for TWAI TX and RX.
- Exact transceiver/module part and schematic; the available board has been described as MCP2551.
- Transceiver supply voltage and verified logic-level compatibility with 3.3 V ESP32 GPIO.
- CAN-H, CAN-L, ground, and power connector pinout at both ends.
- Termination arrangement and unpowered resistance across CAN-H/CAN-L.
- Whether the Matek and transceiver logic are powered from USB with the flight battery absent.
- ESP32 TWAI bitrate; it must match the saved Matek value of 250 kbit/s for this test.
- ESP32 firmware source, build environment, binary hash, expected node ID, startup behavior, and
  printed diagnostics.

The MCP2551 is a transceiver, while the ESP32 TWAI peripheral supplies the CAN controller. The
firmware should report controller state, TX/RX counts, transmit and receive error counters, bus
errors, failed transmissions, error-passive transitions, and bus-off/recovery events. Espressif
documents these states and counters in its
[TWAI driver guide](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32/api-reference/peripherals/twai.html).

## Propellers-off one-aircraft bench sequence

All active steps require propellers removed, aircraft disarmed, a responsible person present, and
the test area cleared.

1. **Photograph and inventory:** record both board faces, connectors, wire colours, transceiver
   markings, termination components, power wiring, ESP32 GPIO mapping, and cable pinout.
2. **Logic-only USB test:** flight battery absent. Connect ESP32 USB and Matek USB-C. Confirm stable
   enumeration and persistent device names. Do not transmit commands.
3. **ESP32 firmware provenance:** obtain the exact source and binary/build hash. Read startup output
   at its documented baud rate and confirm that opening the monitor cannot initiate flight commands.
4. **Electrical CAN test:** with all power removed, measure CAN-H to CAN-L resistance and verify no
   short to power or ground. Check termination against the documented wiring design.
5. **Passive CAN observation:** power logic safely with props off. Capture bus state, bitrate,
   discovered node IDs, RX/TX counts, and error counters. First pass requires traffic without
   persistent bus errors or bus-off.
6. **Read-only application test:** demonstrate a version/health/heartbeat exchange and show which
   protocol message produced it. A changing byte stream alone is not a pass.
7. **MAVLink command-path test:** use UART/MAVLink unless a reviewed CAN bridge has been implemented.
   Verify correct system ID, heartbeat, telemetry freshness, command acknowledgement, and that no
   other vehicle can be targeted.
8. **Bounded disarmed command test:** send only a harmless command approved by the supervisor and
   record the exact request and acknowledgement. Do not arm or run motors in this milestone.
9. **Fault tests:** unplug the companion and network independently. Verify timeout reporting, no
   automatic re-arm, and no stale command replay after reconnection.
10. **Preserve evidence:** save parameter export, firmware hashes, wiring photos, serial/CAN logs,
    timestamps, failures, and the Git commit used for the test.

## Department fleet inventory and progression

The department reportedly owns 10 drones and the team currently has 2. This count is from team
notes and has not been physically inventoried in this session. Assign a stable project asset ID to
each airframe and record:

| Field | Required record |
| --- | --- |
| Asset identity | Project asset ID; department label; custody/availability |
| Airframe | Frame/propeller/motor/ESC models and physical condition |
| Flight controller | Model/revision, USB ID, firmware version/hash, parameter-export hash |
| Navigation and pilot links | GPS/compass, RC receiver/transmitter, telemetry radio |
| Companion hardware | ESP32/other companion model, firmware hash, power source, mounting |
| FC interface | UART/CAN connector, harness, pinout, baud/bitrate, protocol, node/system ID |
| Network identity | Peer ID, radio MAC/serial, IP/port if applicable; keep private identifiers local |
| Evidence status | Inventory only, USB bench, command-path bench, supervised flight |

Progress one team airframe through the complete propellers-off bench gate, repeat independently on
the second team airframe, then perform a two-node radio/peer test. Do not extrapolate those results
to the other eight department aircraft until each is inventoried and bench-checked.

## Pass criteria for the next session

The next session may claim **PHYSICAL USB enumeration** when real devices appear with stable paths.
It may claim **PHYSICAL CAN traffic** only when a real CAN capture or ESP32/ArduPilot counters show
valid frames, matching bitrate, identified nodes, and acceptable error state. It may claim a
**PHYSICAL command path** only when a documented request traverses the intended interface and a real
Matek returns the expected acknowledgement. None of those claims imply flight readiness.
