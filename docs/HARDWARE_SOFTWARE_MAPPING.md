# Mapping the decentralized software to the drone hardware

## Status and scope

This document describes a future integration boundary. The software has been exercised with
ArduPilot SITL, including five independent node processes and direct peer datagrams. One physical
airframe is assembled, none has been flight-tested, and the software has not been connected to that
airframe. No flight-controller parameters or hardware were changed for this work.

## Roles on a physical system

```text
                    mission setup / monitoring / emergency request
               ┌────────────────────────────────────────────────────┐
               │              Operator laptop / GCS                 │
               └───────────────────┬────────────────────────────────┘
                                   radio/network
                 shared mission intent + status, not setpoints
                   ┌───────────────┴────────────────┐
                   │                                │
       Drone 1 peer agent  ⇄ direct peer network ⇄ Drone 2 peer agent …
                   │                                │
                 MAVLink UART                    MAVLink UART
                   │                                │
             Matek + ArduPilot                 Matek + ArduPilot
                   │                                │
             sensors / ESCs / motors          sensors / ESCs / motors
```

Each aircraft needs its own agent process or embedded agent. It reads that aircraft's telemetry,
uses the shared mission intent and fresh neighbor states, calculates its own target, and sends
setpoints only to its own flight controller. ArduPilot remains responsible for stabilization,
motor mixing, and configured vehicle failsafes. The laptop may start a mission, display health, and
request an emergency action; it must not calculate and relay every drone's movement for the swarm
to qualify as decentralized.

## Software components and physical equivalents

| Current software boundary | Physical equivalent | Current readiness |
| --- | --- | --- |
| `independent_agent_node` | Per-aircraft companion running one agent | Process boundary and SITL behavior tested; not deployed onboard |
| MAVLink TCP connection to SITL | MAVLink UART/telemetry port to that aircraft's Matek | Agent now accepts serial device paths and configurable baud/system ID; only TCP SITL has been exercised |
| Direct peer UDP datagrams | Wi-Fi/radio network between aircraft agents | Roster-based IP/UDP interface and impairment tests exist; no radio or two-computer test verified |
| Mission-intent UDP | Operator link that sends the same goal/lease to each agent | Protocol and same-host operator-loss behavior tested; physical link not tested |
| Fixed agent ID, formation, and peer roster | Provisioned aircraft identity and addresses | Configuration is static and suitable only for a controlled test; no discovery or join protocol |
| Simulated LAND request and GCS heartbeat | ArduPilot Guided command and real configured failsafe | SITL-tested only; real aircraft parameters and failsafe actions remain a separate safety review |

## First physical interface to investigate

The simplest first data path is **MAVLink over UART between the Matek and one companion**, with the
peer network handled separately. ArduPilot documents this companion-computer pattern: the companion
reads MAVLink telemetry and can make higher-level decisions. The Matek board photo and available
harnesses must be checked to determine whether an unused UART is accessible through a plug-in cable.
If it is available only as bare pads, the team's no-solder rule means this interface cannot be used
until an approved connectorized solution is found.

The independent agent accepts either its existing SITL TCP endpoint or a serial device path. For a
serial link, the process can be configured with a device path such as
`--endpoint /dev/serial/by-id/usb-flight-controller` and `--baudrate 115200`. Use a unique expected
MAVLink system ID per aircraft with `--mavlink-system-id 1`; the default mapping (agent ID + 1) is
convenient for SITL, not proof that
the real flight controllers are configured that way. The actual device path, baud rate, system ID,
UART protocol setting, and power budget must be confirmed on the bench before connection.

This code change only adds an interface option. It does not establish electrical compatibility,
make a cable, change ArduPilot parameters, or prove a physical connection.

## Where ESP32 and CAN fit

An ESP32 could be the companion on each aircraft: it would exchange peer state over a radio link and
send that aircraft's MAVLink commands to its Matek. The present swarm agent is Python and uses
NumPy and pymavlink; it will not run unchanged on a typical ESP32 firmware. A real ESP32 deployment
would need a deliberate port of the agent logic and MAVLink/protocol code to an embedded runtime,
with equivalent tests. A small Linux companion would preserve more of the Python implementation but
adds cost, mass, power draw, and integration work.

For the current software baseline, I recommend a Linux companion per aircraft as the reference
implementation because it can run the Python node and connect to that aircraft's ArduPilot over
MAVLink. An ESP32 can still be useful as a separate radio/network interface, or as the main agent
after a deliberate firmware port and behavior-parity tests. Decentralized behavior itself does not
require a mini-PC if a smaller agent is implemented directly on a microcontroller or inside the
autopilot; a mini-PC is needed to run this current Linux/Python design unchanged.

### Would adding an ESP32 to the simulation make it more realistic?

A physical ESP32 in the SITL loop would improve realism for the parts it actually exercises:
firmware execution, packet encoding and decoding, the serial or radio interface, and measured timing
between the board and the simulated vehicle. It would not make SITL simulate the real motors,
sensors, airframe, or flight performance. It also would not validate a five-drone swarm unless
each simulated aircraft had its own agent endpoint (and, for an ESP32-agent design, its own ESP32).

The current peer transport already injects packet loss, delay, duplication, reordering, and link cuts
in software. A physical ESP32 is therefore not required to make those algorithm and network-fault
tests stronger. The practical progression is to keep the reproducible SITL tests, then run one agent
against one embedded endpoint connected to one SITL vehicle, then five independent endpoints. If
ESP32 is selected as the agent computer, port the Python agent behind the same mission/peer protocol
and require behavioral-parity tests against the Python implementation. Treat that as future
hardware-in-the-loop work, not as something established by localhost SITL.

CAN is an alternative physical network, not a plug-in replacement for the current UDP and MAVLink
software. A CAN design needs a controller at each endpoint, CAN transceivers, bus wiring/termination,
and a defined application protocol. The MCP2551 is a transceiver only. Its datasheet specifies a
4.5–5.5 V supply and an RXD high level of at least 0.7 × VDD; therefore its module must not be
assumed electrically safe for direct ESP32 GPIO connection without checking its exact circuit and
level compatibility. ESP32 TWAI supplies a CAN controller, but a transceiver is still required.

The saved Matek parameter file reported `CAN_P1_DRIVER=0`, which means that saved configuration
does not enable that CAN port. ArduPilot's CAN configuration also selects a driver and protocol;
arbitrary swarm messages are not automatically accepted as flight commands. The exact board CAN
interface, firmware support, bus wiring, selected protocol, and a safe command bridge all need
bench validation. CAN does not remove the need for a connectorized physical connection under the
no-solder constraint.

## Recommended integration sequence

1. Keep the current five-agent SITL evidence as the software baseline.
2. Confirm the exact Matek revision, exposed UART/CAN connectors, cable pinout, logic levels, and
   permitted plug-in harness with the hardware team. Do not connect based only on pad labels or
   assumed voltage levels.
3. On a disarmed bench setup, connect one agent to one flight controller and verify telemetry
   reception, correct system ID, heartbeat behavior, and read-only status first.
4. In a separate controlled test, verify a bounded Guided setpoint and the configured loss-of-agent
   failsafe. Props removed and the vehicle restrained/otherwise made safe according to the team's
   test procedure.
5. Add a second aircraft and test peer messages, stale-data rejection, loss of operator, link loss,
   and recovery without automatic re-arming or takeoff.
6. Expand to five only after two-aircraft evidence passes. Record raw logs, versions, parameters,
   network settings, and unsuccessful runs.

Passing SITL, Gazebo, or the serial-interface unit test is not evidence that any real aircraft is
ready to fly. Physical identification, parameter review, bench checks, and supervised flight tests
remain future gates.

## References

- [ArduPilot companion computers](https://ardupilot.org/dev/docs/companion-computers.html)
- [ArduPilot CAN setup](https://ardupilot.org/copter/docs/common-canbus-setup-advanced.html)
- [ESP-IDF TWAI/CAN controller](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/can.html)
- [Microchip MCP2551 datasheet](https://ww1.microchip.com/downloads/en/devicedoc/21667e.pdf)
- [Independent SITL protocol and limitations](AGENT_PROTOCOL.md)
- [External agent roster setup](PEER_NETWORK_SETUP.md)
