# Direct peer and operator network setup

Independent SITL agents can now use a static IPv4 roster for direct peer state, receive operator
mission/command messages at configured addresses, and send status back to the operator. HMAC-SHA256
can authenticate both channels using the configured shared key. Localhost remains the default.

The example roster is
[`examples/peer_roster.multihost.example.json`](examples/peer_roster.multihost.example.json).
Each row describes:

- `id`: stable zero-based agent ID.
- `bind_host`, `port`: local peer UDP bind endpoint.
- `host`: advertised peer address other agents can reach.
- `command_bind_host`, `command_port`: local endpoint for incoming operator messages.
- `command_host`: address the operator uses to reach that agent.
- `status_host`, `status_port`: operator address and unique port for status reports.

Use stable private-network addresses. On one host, agents need distinct bind address/port pairs;
across hosts they can reuse a port. All hosts use the same roster. A remote agent should bind on its
available interface (often `0.0.0.0`) while advertising its reachable private IP. The operator
status ports must be unique. `--peer-auth-key-file` reads a shared secret of at least 32 bytes;
keep it outside Git and restrict access to it. HMAC provides integrity/shared-key authentication,
not encryption or per-agent identity. Any holder of the group key can impersonate another agent.
Peer-state reception also checks that each datagram's source IPv4 address and UDP port match the
claimed sender's advertised roster endpoint. This rejects a valid group signature arriving from an
unrostered endpoint; it is not a substitute for per-agent keys or network access controls. The
static roster assumes directly reachable UDP endpoints and does not support source-address-changing
NAT.

The direct UDP impairment scheduler has bounded pending-delivery queues: at most 64 datagrams per
peer and 512 total per agent process. When a limit is reached, it removes the oldest enqueued
datagram first, then retains the new one. `dropped_queue` reports these evictions separately from
configured random loss. This prevents long simulated delays or duplication bursts from building an
unbounded backlog; it is a software test-transport policy, not a claim about physical radio queues.

For independent agents, `--peer-latency-ms` sets mean injected one-hop delay and
`--peer-jitter-ms` sets its standard deviation. Each delay is sampled from a Gaussian distribution
and clipped at zero; use a different `--peer-seed` per agent for repeatable but distinct samples. If
jitter is omitted, the compatibility default is 5 ms when mean delay is nonzero and 0 ms otherwise.
For example, `--peer-latency-ms 25 --peer-jitter-ms 8 --peer-seed 120`. These are software
impairments for scenario testing, not a model of a specific radio until measured radio data is used
to choose the values. Agent status includes the number of scheduled delay samples and their observed
mean, standard deviation, minimum, and maximum.

## Verified so far

On 2026-10-25, a five-agent SITL mission passed across the Ubuntu desktop and
an Ubuntu Server VM connected by a host-only TAP network. Two agents and all
five SITL vehicles ran on the host; three agents ran in the VM. The first
attempt safely failed preflight because small clock-offset estimation errors
made one peer link appear stale. After a bounded timestamp-tolerance fix, all
five passed preflight, flew a 2 m north mission, landed, and disarmed. See the
[host/VM report](reports/VM_SPLIT_HOST_SITL_2026-10-25.md) and its raw logs.
This tests an actual virtual network boundary on one physical PC. It does not
test a separate laptop, radios, or real aircraft.

A subsequent [ten-agent host/VM SITL run](reports/VM_SPLIT_TEN_AGENT_SITL_2026-10-25.md)
split five agents onto each OS instance. All ten passed peer-link preflight,
completed a 2 m north mission, and landed/disarmed. Its minimum sampled
horizontal separation was 4.094 m. This is one clean run without injected
network faults; it does not qualify a physical network or aircraft.

Five agents completed a partition mission using five distinct loopback addresses, 2% injected peer
loss, 25 ms mean delay, and HMAC. Both isolated endpoints landed and all five disarmed. A wrong-key
test was rejected. This validated address selection and HMAC between processes on one computer; it
did not validate routing between computers.

The operator-channel transport has a five-endpoint loopback test that sends one signed command to
each configured agent address and receives authenticated status from all five. A separate external-
console integration test runs the actual operator class against five fake agent endpoints: all
report ready, receive mission and launch/LAND commands, and report landed status. It confirms the
console starts no local SITL or agent children and waits for landing confirmation. The fake agents
also use wall clocks deliberately offset from the operator by up to six seconds; the five-probe
startup exchange recovered those offsets, and all five agents acknowledged the clock map. These are
same-host loopback tests with simulated timestamps; they do not use ArduPilot agents or a second
computer. The operator marks status older than three seconds as stale.

In live seed 152, five independent ArduCopter SITL agents acknowledged launch and LAND, moved 1.531 m
toward a 2 m north goal, recovered from a directed peer-link cut/restore, and all landed/disarmed.
The run used 2% peer loss, 25 ms mean delay, and 5 ms jitter on the same host. Both commands were
acknowledged on their first transmission; same-sequence retry and duplicate suppression are verified
by unit tests, not by an injected acknowledgment-loss SITL run. Raw evidence is in
`runs/independent/external-sitl-20261021T182636.809231Z/`. This does not close the two-computer LAN
test. See the [independent SITL report](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md).

## External-agent mode

The operator console can run without launching local SITL or agent processes:

```bash
python3 -m sitl.swarm_console --drones 5 --agent-runtime independent \
  --control-mode decentralized --external-agents --operator-bind-host 0.0.0.0 \
  --peer-roster docs/examples/peer_roster.multihost.example.json \
  --peer-auth-key-file /path/to/private/operator-key
```

On each agent host, start that host's ArduPilot SITL instance, then run its agent with the local
MAVLink endpoint. For a SITL instance, for example, agent ID 0:

```bash
python3 -m sitl.independent_agent_node --id 0 --count 5 \
  --endpoint tcp:127.0.0.1:5760 --formation v \
  --peer-roster docs/examples/peer_roster.multihost.example.json \
  --peer-auth-key-file /path/to/private/operator-key
```

For a future companion connected to an autopilot UART, replace the TCP endpoint with the actual
serial device and baud rate, for example:

```bash
python3 -m sitl.independent_agent_node --id 0 --count 5 \
  --endpoint /dev/serial/by-id/usb-flight-controller --baudrate 921600 \
  --mavlink-system-id 1 --formation v \
  --peer-roster docs/examples/peer_roster.multihost.example.json \
  --peer-auth-key-file /path/to/private/operator-key
```

The device path, baud rate, and unique `SYSID_THISMAV` value must be verified for each autopilot.
The serial path has a fake-device software test only; no Matek connection has been tested. See
[`HARDWARE_SOFTWARE_MAPPING.md`](HARDWARE_SOFTWARE_MAPPING.md).

Repeat with each agent ID and that host's local flight-controller endpoint. Replace all sample IPs
with addresses on the test network. Set `status_host` to an operator address reachable from each
agent. The example ports are peer UDP 20000, command UDP 21000, and status UDP 22000–22004; permit
those ports only between participating hosts, or update both the roster and firewall rules. The same
key and roster must be available on every host. At startup, the operator measures each agent's clock
offset with five NTP-style UDP probes, accepts the lowest-delay sample only when its round trip is
at most 250 ms, sends the offset map, and blocks mission operation until all agents acknowledge it.
That map is rebroadcast every five seconds for nodes that restart, and offsets are remeasured every
60 seconds while the console is active. Use NTP or another host clock service as well. If a refresh
fails, the last acknowledged map remains active and the console retries after one minute. Commands
expire after five seconds, and the console discards replayed or stale status sequence numbers.
Launch and LAND commands are retransmitted with the same sequence for up to four seconds until all
agents acknowledge receipt. An agent re-acknowledges a duplicate sequence without executing the
action again. The acknowledgment proves command delivery only; check the agent's fresh status to
confirm it became airborne or landed. A missing acknowledgment is reported, and the command may
still have reached that agent if its reply was lost.

An operator restart creates a new clock-sync epoch. Agents accept the new epoch, reset only the
epoch-scoped mission/command/fault counters, and ignore delayed messages from the retired epoch. A
replacement console waits for fresh status and restores the common active goal reported by the
agents; it refuses takeover if their active goals, formations, or altitudes disagree. This protects
against silently resetting a live swarm to the console's default home target. Seed 154 exercised a
single same-host SITL handover while airborne. It remains unverified across two computers and under
repeated fault conditions.

At the external operator prompt, `preflight` reports each agent's fresh status, accepted mission
lease, synchronized clock, and directly observed peer heartbeats. `launch` repeats this swarm-wide
check and sends no launch command if any required peer link is missing, preventing a partial takeoff.
`status` shows configured peers and currently live peers separately. These checks confirm software
connectivity and SITL readiness; they do not qualify a radio or aircraft for flight.

External mode does not manage remote process lifecycles. `fail` and emergency `rescue` commands are
unavailable; stop an agent on its own host to test agent loss. The operator has no durable command
queue or crash-proof mission journal. The replacement-console handover described above preserves a
mission only when all responding agents report the same currently active intent. The whole path still
needs a two-computer SITL test, including actual UDP routing, asymmetric delay/loss, clock drift,
coordinator loss, and recovery. This is not a real-aircraft deployment configuration.

## Two-computer LAN SITL topology

The existing external-agent interface can be tested across two computers without installing
ArduPilot on the agent computer. Put five standalone SITL flight controllers, agents D1/D2, and the
external operator on **Computer A**. Put agents D3/D4/D5 on **Computer B**. The V topology has
cross-computer direct-peer edges D1↔D3 and D2↔D4, so this layout sends actual peer UDP between the
computers as well as operator commands/status and remote TCP MAVLink. It does not test radio
hardware.

The current ArduPilot SITL build's default SERIAL0 TCP listener binds all IPv4 interfaces and uses
the fleet ports `5760 + 10 * ID` (confirmed in the local ArduPilot SITL source). The included launcher
starts that fleet and stops only the children it created. On Computer A, replace the example IP with
its stable LAN address:

```bash
python3 experiments/launch_multihost_sitl.py --drones 5 --advertise-ip 192.168.1.10
```

On both computers, use the same roster. The template
[`examples/peer_roster.two_host_sitl.example.json`](examples/peer_roster.two_host_sitl.example.json)
uses `192.168.1.10` for A and `192.168.1.20` for B. Generate one private 32-byte HMAC key and copy it
to both computers through a trusted channel. Run one agent process for each assigned ID, changing
`ID` to 0 and 1 on A and to 2, 3, and 4 on B. The local agents on A connect to loopback SITL; agents
on B connect to A's LAN IP:

```bash
ID=2
SITL_HOST=192.168.1.10
if [ "$ID" -lt 2 ]; then SITL_HOST=127.0.0.1; fi
PORT=$((5760 + 10 * ID))
python3 -m sitl.independent_agent_node --id "$ID" --count 5 \
  --endpoint "tcp:${SITL_HOST}:${PORT}" --formation v \
  --peer-roster docs/examples/peer_roster.two_host_sitl.example.json \
  --peer-auth-key-file /path/to/private/operator-key
```

Run the operator on A, binding its status listeners to the LAN interface:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone \
  --control-mode decentralized --agent-runtime independent --external-agents \
  --operator-bind-host 0.0.0.0 \
  --peer-roster docs/examples/peer_roster.two_host_sitl.example.json \
  --peer-auth-key-file /path/to/private/operator-key
```

Permit B to reach A's TCP ports 5780, 5790, and 5800. Permit A and B to exchange UDP peer ports
20000–20004, operator command ports 21000–21004, and status ports 22000–22004. Keep the TCP MAVLink
listeners on a trusted private network: this
prototype does not authenticate or encrypt those TCP streams. At the console, require `preflight`
to pass before `launch`; then check agent status, movement toward a small goal, a directed
cut/restore, LAND confirmation, and process cleanup. Preserve both hosts' agent and simulator logs
with the run. Do not connect or arm physical aircraft during this topology test.

## Actual-agent external-operator SITL validation

`experiments/validate_external_agent_sitl.py` starts five standalone ArduPilot SITL vehicles,
five independent agent processes, and the external operator as separate process groups. The
agents used distinct `127.0.0.x` loopback addresses, a generated HMAC key, 2% application-level
peer loss, and 25 ms mean delay. In seed 122 the operator sent a shared 2 m north goal, the leader
moved 1.51 m toward it, a D1-to-D2 link cut was counted at the sender and restored, all agents
remained nominal after restoration, and all five vehicles were confirmed landed and disarmed. No
authentication, invalid-packet, or replay rejections were recorded. Raw logs and the summary are
under `runs/independent/external-sitl-20261021T173901.719377Z/`.

This closes a same-host external-operator/real-agent SITL gap. The addresses are still loopback;
it does not prove LAN routing, separate-host clocks, physical radio behavior, or flight safety.
An earlier exploratory run, `runs/independent/external-sitl-20261021T173636.189692Z/`, verified
process startup and landing but reached only 0.49 m of the 2 m goal before landing. It is preserved
and is not counted as the full mission-motion validation.

The SITL-only host launcher was smoke-tested with five ArduCopter processes. It opened and
connected to the SERIAL0 endpoints through the alternate loopback address `127.0.0.2` on ports
5760, 5770, 5780, 5790, and 5800, then stopped all five child processes on its configured timeout.
This verifies launcher setup and the all-interface listener path locally; it is not the missing
two-computer run. The retained run manifest and per-instance logs are in
`runs/independent/multihost-sitl-20261021T184308.547509Z/`.

The external-agent harness also supports `--topology split-loopback`, assigning D1/D2 to the `.1`
address and D3/D4/D5 to `.2`, matching the two-computer ID split while keeping every process on one
computer. Seed 155 passed the five-agent flight, 2 m goal, directed peer cut/restore, and landing
checks; status showed every configured direct neighbor live. This rehearses the actual roster and
agent-address grouping but remains Linux loopback traffic, not a LAN packet path. Evidence:
`runs/independent/external-sitl-20261021T184527.241236Z/`.

Seed 156 repeated that topology with a time-aligned separation metric enabled as a validation gate.
The harness matched all five airborne status streams within 250 ms for 22 samples and measured a
minimum horizontal separation of 4.217 m. All five vehicles landed/disarmed and the run recorded no
surviving SITL or agent processes. This is still one-computer loopback evidence. Summary:
`runs/independent/external-sitl-20261021T184822.843234Z/external_agent_sitl_summary.json`.

Seed 157 tested a missing cross-group edge before takeoff by cutting D1→D3. The operator blocked
launch, all five SITL vehicles stayed grounded/disarmed, and restoring the edge made preflight pass.
The mission then ran, the later D1→D2 cut/restore passed, and all five landed/disarmed. The run also
recorded 21 aligned airborne samples and 4.153 m minimum horizontal separation. This verifies the
negative launch gate against the planned cross-group link mapping on loopback, not across a physical
LAN. Summary: `runs/independent/external-sitl-20261021T185051.169880Z/external_agent_sitl_summary.json`.

Seed 134 repeated the scenario with explicit 8 ms jitter. The agents recorded 395–795 scheduled
delay samples each; measured means ranged from 24.32 to 25.15 ms and standard deviations from 7.55
to 8.21 ms. The leader moved 1.54 m toward the goal, the directed cut/restore passed, and all five
landed and disarmed. Evidence:
`runs/independent/external-sitl-20261021T174902.634712Z/`. These are sampled software delays, not
measurements of a radio.

The same external-agent harness now supports `--operator-loss`: after a shared goal is accepted and
the link-fault check completes, it kills only the operator process and listens directly for
authenticated agent status. Seed 135 passed with 2% packet loss, 25 ms mean delay, and 8 ms jitter.
All five agents reported `landing:mission_expired`, then landed and disarmed within 42.99 seconds;
no operator LAND fallback was sent. Evidence:
`runs/independent/external-sitl-20261021T175401.693794Z/`. This is still five agents and the
operator on one computer using loopback; it does not establish coordinator-loss behavior across
computers or radios.

The swarm-wide preflight gate was then verified in seed 136 with five live ArduPilot SITL agents.
Before launch, every agent reported all configured neighbors as live over direct peer heartbeats
(D1: D2/D3, D2: D1/D4, D3: D1/D5, D4: D2, D5: D3); the operator logged `launch_preflight: passed`.
The leader moved 1.52 m toward the 2 m goal, the directed peer cut/restore passed, and all five
landed/disarmed. Evidence is in
`runs/independent/external-sitl-20261021T180020.779637Z/`. This verifies the preflight logic and
direct-heartbeat reporting in same-host loopback SITL, not on a LAN or radio.

Seed 137 tested the negative case before the mission: the directed D1→D2 peer link was cut before
launch, D2 stopped listing D1 as live, and the operator's launch preflight failed. The summary
records `launch_blocked: true` and `all_vehicles_stayed_grounded: true`; after restoring the link,
preflight passed, the 2 m mission and later cut/restore check completed, and all five SITL vehicles
landed and disarmed. Evidence is in
`runs/independent/external-sitl-20261021T180346.638316Z/`. This proves the software gate in a
same-host fault-injected SITL run; it does not prove behavior across separate computers, actual
radios, or physical aircraft.

Seed 139 repeated the full external-agent scenario after peer source-endpoint validation was added.
With 2% injected peer loss, 25 ms mean delay, and 5 ms jitter, the pre-launch D1→D2 cut blocked
launch and kept all five grounded; restoring it passed preflight. The leader moved 1.54 m toward the
2 m goal, the later directed cut/restore completed, and all five landed/disarmed. No peer packet
from an unexpected source was observed (`rejected_source: 0`); a separate unit regression sends a
valid shared-key packet from an unrostered UDP source and verifies rejection. Evidence:
`runs/independent/external-sitl-20261021T181141.293642Z/`. Still one-host loopback SITL, not a LAN
or radio test.

Seed 141 repeated the external-agent path with operator-loss validation after source-endpoint
checking was enabled. Five agents accepted the 2 m goal, the leader moved 1.564 m, and the directed
peer cut/restore completed with all agents nominal after restoration. The operator process was then
killed. Every agent reported `landing:mission_expired`, and all five landed/disarmed within 42.93
seconds. The run used 2% injected peer loss, 25 ms mean delay, and 5 ms jitter; no unexpected-source
rejections were recorded. Evidence is in
`runs/independent/external-sitl-20261021T181541.466947Z/`. This validates coordinator-process loss
in five same-host loopback SITL agents; it does not prove LAN, radio, or physical behavior.

Seed 143 separately repeated the airborne agent-restart case after source-endpoint validation was
added. Drone 3's restarted process recognized that its aircraft was armed and airborne, commanded
LAND, and announced the landing state to peers. The other four agents entered their peer-landing
failsafe, and all five were confirmed landed/disarmed. No unexpected-source packets were reported.
Evidence is in `runs/independent/20261021T181849.530309Z/`. This is a same-host ArduPilot SITL
repeat, not a radio or aircraft test.
