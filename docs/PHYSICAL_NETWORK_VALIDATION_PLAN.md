# Physical Network and Flight Validation Plan

## What current evidence does and does not show

The independent-agent work has been exercised with five ArduPilot SITL vehicles and separately
running agent processes on one computer. The latest external-agent run uses loopback addresses and
injected packet loss/delay. It demonstrates software behavior under those simulated conditions. It
does not demonstrate a second computer, the installed radios, companion-computer wiring, or flight.

Keep these as separate claims and evidence gates:

1. **Network test:** peer traffic traverses the actual radio network.
2. **Distributed test:** every drone's agent runs independently and makes decisions from its own
   telemetry plus received peer state.
3. **Aircraft test:** the companion computer communicates with ArduPilot on the aircraft and the
   aircraft follows a safe, supervised test.

## Gate 0: identify the actual communication equipment

Before changing the software transport or preparing a flight, record for each vehicle:

- Flight-controller model, ArduPilot version, and available UART connectors.
- Companion-computer model and how it will be powered and mounted.
- Exact radio model, interface, supported topology, and whether it carries IP/UDP or only serial
  bytes.
- Which link is the pilot's RC control link, which carries MAVLink telemetry, and which carries
  peer-to-peer agent data.
- Radio configuration, permitted frequency/power for the test location, and a safe manual takeover
  method.

Do not assume that an RC receiver or a serial MAVLink telemetry pair is an IP network. The current
peer transport uses UDP. If the selected radios do not carry routed IP/UDP, implement and test an
explicit bridge or a transport adapter before claiming that the current peer path works over them.
ArduPilot's companion-computer model is a separate computer exchanging MAVLink with the flight
controller, commonly over UART; radio transport between companions is a distinct link to validate.
See the [ArduPilot companion-computer guide](https://docs.ardupilot.org/dev/docs/companion-computers.html)
and [Raspberry Pi MAVLink UART guide](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html).

## Gate 1: radio-only bench test

Use at least two separate computers and the actual radios. Initially run the peer agents without
arming or connecting motor outputs. Collect timestamped transmit/receive records and report:

- packet delivery and loss by peer and direction;
- median and high-percentile message age, jitter, and longest gap;
- peer watchdog detection and recovery time;
- whether duplicate, stale, unauthenticated, or wrong-peer messages are rejected;
- behavior when one radio is powered off and then restored.

Choose pass thresholds from the configured heartbeat, lease, and stale-state timeouts before the
run. A pass requires the observed message-age distribution to remain inside those limits during the
nominal run, and the deliberate outage to trigger the designed safe behavior without stale state
being accepted. Preserve the radio settings, logs, software revision, and test conditions with the
results. Repeat with all five nodes only after the two-node path is stable.

## Gate 2: flight-controller bench integration, propellers removed

Connect each companion to its own flight controller using a supported MAVLink interface. Confirm
that it receives the expected system ID and fresh telemetry, and that commands target only its own
vehicle. Test disconnecting the companion, the radio, and the operator independently. Verify that
the agent reports the fault and that no stale command resumes after reconnection without passing
the required preflight again.

Check the actual aircraft parameters and failsafe configuration with the supervisor. Do not bypass
pre-arm checks to make this gate pass. ArduPilot warns against flying with unresolved pre-arm errors
or disabled arming checks; use its
[pre-arm checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html) and
[pre-flight checklist](https://ardupilot.org/copter/docs/checklist.html).

## Gate 3: supervised single-aircraft flight

Only after the bench gates pass, have an experienced pilot/supervisor approve the aircraft and test
area. Keep a pilot with immediate manual control. Verify one aircraft at a time: basic controlled
flight, then a small guided movement, then the intended autonomy and loss-of-link response. Record
flight-controller logs, companion logs, radio metrics, pilot interventions, and battery state. Stop
and investigate any unexpected mode change, stale command, loss of control, or failsafe behavior.

## Gate 4: multi-aircraft progression

Do not jump directly from one aircraft to a five-aircraft flight. First prove two aircraft can
exchange peer state over their actual radios while maintaining conservative spacing and independent
pilot takeover. Then expand the same test plan to additional aircraft. Each increase requires a
review of radio airtime, packet age, identity/authentication, separation, operator visibility, and
the behavior when one aircraft or link fails. Keep ground and flight evidence separate.

## Current blockers and next evidence

- The exact radio model and its data-link capability have not been established in this software
  checkout.
- No actual radio link, second-computer LAN run, or physical flight is recorded here.
- Current shared-key authentication does not establish unique cryptographic identity for each
  aircraft; do not describe it as per-drone identity.
- The next software-preparation milestone is a two-computer LAN run of the five independent SITL
  agents. The next hardware-integration milestone is a two-node, propeller-off radio bench test
  after the radios and connector path are identified.
