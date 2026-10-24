# Experimental controller process and peer protocols

Run the five-drone console with `--control-mode decentralized --agent-runtime process`. Each
controller runs in its own Python process. The default runtime remains `local`.

The parent/agent control channel is newline-delimited JSON over private pipes. Its envelope remains
version 1 and carries one agent's own telemetry, one shared mission intent, its provisioned slot,
the current one-hop roster, and the peer states received by the child. The console still owns all
MAVLink connections and supplies each agent's own telemetry. Process isolation is not onboard
deployment or a separate flight-controller connection per agent.

Transport: newline-delimited JSON on private parent/child pipes, one request and response in flight
per agent. Version is `1`, frame is `shared_local_north_east_metres`, drone IDs are zero-based,
sequence numbers strictly increase. Position/goal/velocity use north/east metres and metres/second;
`sample_time` and `now` are seconds in the same host wall-clock domain. `dt` is seconds.
Each request contains only the recipient's own telemetry, its assigned goal, relative neighbor
slot offsets and received one-hop peer messages; it never includes the whole fleet state.
Peer entries carry ID, position, velocity, age and confidence. Altitude remains managed by the
console, and hybrid altitude/slots freeze with the last received mission during coordinator loss.

A request with an unsupported version/frame, wrong recipient, replayed sequence, own telemetry
older than 1 s or in the future, nonfinite positions or stale peer state is rejected. Shared
mission intent has its own version, monotonic sequence, issue/expiry times, common north/east goal,
and coordinate frame. Each agent combines that common goal with its local slot to derive its own
target. Expired or replayed intent is rejected. Missing or invalid replies cause an agent fault and
a LAND request for that simulated aircraft. Recovery from a failed controller process requires a
session restart. Response deadline is 2 s; this is a prototype with sequential request/response
handling and no real-time scheduling guarantee.

In the legacy `--agent-runtime process` mode, agents publish versioned state datagrams directly to
their configured one-hop peers over UDP loopback (ports `20000 + zero-based-drone-id`). The parent
does not relay peer-state packets in this mode. The harness still provisions peer rosters and slots.
Optional `--peer-loss`, `--peer-latency-ms`, `--peer-seed`, and `--peer-burst-loss` settings apply
per sender in the agent
transport; this is application-level impairment over loopback, not a radio channel model. The
separate local runtime continues to use `PeerStateBus` for deterministic delay/loss/link blocking.
Its pending-delivery queue is bounded to 64 datagrams per peer and 512 total; reaching a cap evicts
the oldest enqueued datagram and increments `dropped_queue`, preserving newer state. This is a
simulation-side memory bound, not a physical radio queue model.
This process-worker mode does not configure network authentication or encryption and is intended for
local development, not exposed network links.

The process-worker runtime does not own independent MAVLink links. Neither runtime implements peer
discovery, encryption, or radio framing. The separate independent runtime described below supports
static rostered UDP endpoints, shared-key HMAC authentication, source-endpoint checks, and clock
offset estimation; these are prototype network controls, not a radio-security design. Do not expose
the process-worker development ports beyond the local test machine.

## Independent per-drone SITL runtime

The experimental `--agent-runtime independent` path is separate from the controller worker above:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone \
  --control-mode decentralized --agent-runtime independent \
  --peer-loss 0.1 --peer-latency-ms 50 --peer-seed 44 --peer-block 1:2
```

Each `sitl.independent_agent_node` process opens one vehicle's MAVLink endpoint, reads its own
telemetry, owns local guidance and setpoint generation, and sends peer state directly to its
provisioned one-hop neighbors over UDP (`20000 + zero-based ID`). The SITL console uses each
vehicle's TCP MAVLink endpoint. For future companion integration, the node also accepts a serial
device path with `--baudrate`; its expected autopilot system ID is configurable with
`--mavlink-system-id` (defaults to zero-based agent ID + 1). Serial connection support is unit-tested
with a fake device and is not physical hardware evidence. The operator sends the same
versioned mission intent directly to each node (`21000 + zero-based ID`) and receives status on
`22000 + zero-based ID`. Mission sequence, frame, issue time, expiry, goal and target altitude are
validated. Nodes cache an accepted mission between operator refreshes. The console currently
refreshes that 30-second mission lease once per second; expired intent requests LAND. The peer
roster and formation slot remain fixed for the run. By default, endpoints bind to localhost and
traffic is unauthenticated. For a configured network, `--peer-roster` selects static peer,
operator-command, and status endpoints, while `--peer-auth-key-file` enables shared-key HMAC for
peer, command, and status traffic. The group key gives every holder the ability to impersonate any
participant; source-endpoint checks are an additional routing check, not unique drone identity.
Launch and LAND commands are retried with the same sequence until agents acknowledge receipt or
the four-second retry window ends; agents acknowledge a duplicate without repeating its action.
That receipt acknowledgment is distinct from status confirmation that takeoff or landing completed.
These prototype controls do not provide encryption or radio security. See
[`PEER_NETWORK_SETUP.md`](PEER_NETWORK_SETUP.md) before using external-agent mode.

Each operator process starts a new random clock-sync epoch. Mission, command, and peer-fault
messages are accepted only for the currently synchronized epoch; agents reset those message
sequence counters on a new epoch and reject messages from retired epochs. On startup, a replacement
external operator waits for fresh status, reads the agents' accepted mission intent, and resumes
that target if all agents report the same active goal, formation, and altitude. It refuses takeover
when active missions conflict. The next mission refresh uses a sequence above the recovered sequence
and the new epoch. This handover was exercised once in five-agent same-host SITL; it is not a
multi-host or reliability result.

Peer datagrams use protocol version 3 and include a random per-process session ID alongside the
per-session state and heartbeat sequence numbers. State packets also carry a boolean `landing` flag.
A receiver accepts a fresh, previously unseen session for a configured peer, resets that peer's
sequence tracking, and retires its old session; delayed packets from a retired session are rejected.
This lets the peer transport recognize a restarted sender instead of rejecting reset sequence
numbers as replays. A restarted agent that finds its flight controller armed and its vehicle airborne
does not attempt to reconstruct its lost mission/controller history: it commands LAND, publishes
`landing: true`, and enters `landing_after_process_restart`. Airborne peers that receive that state
also command LAND. After touchdown/disarm the restarted node reports
`landed_after_process_restart`; relaunch requires a later explicit operator command and normal
preflight. Unit tests and one five-drone standalone SITL run cover restart detection, peer LAND
propagation, and all-five landing/disarm; evidence is in
`runs/independent/20261021T165833.817892Z/`. The local independent console exposes `restart <id>`
for this fault test. This is a conservative recovery policy, not automatic mission resumption, and
the single live run is not a reliability estimate. Random session IDs are transport identifiers
rather than cryptographic per-drone identity.

Before missions are enabled, the operator performs five four-timestamp clock probes per agent,
selects the lowest-delay valid sample (maximum round trip 250 ms), and reports half the selected
round trip as the offset uncertainty bound. Startup fails if any agent has no valid sample or does
not acknowledge the resulting clock map. The measured host offsets are used to validate operator
mission/command expiry, normalize status timestamps, and translate peer sample times into each
receiver's clock domain. The map is periodically rebroadcast so a restarted agent can load it, and
the operator remeasures offsets every 60 seconds while active. A five-endpoint integration test used
synthetic agent clocks skewed from the operator by up to six seconds, then changed their offsets
after startup; the console recovered and distributed the updated map. This is loopback evidence, not
a two-computer network test. Failed refreshes leave the last acknowledged map active and are retried
after a minute, so hosts should still run a clock-sync service; multi-host validation remains open.
After accepting an intent, each node converts its remaining lease duration to a local monotonic
deadline so later local wall-clock adjustments cannot extend or prematurely expire that lease.

`--peer-block 1:2` drops only peer datagrams sent from drone 1 to drone 2 (console IDs are
1-indexed). `dropped_blocked` is counted separately from random-loss drops. One five-drone SITL
trial with this asymmetric link cut, 10% random loss, and 50 ms mean latency still moved and landed;
all vehicles were confirmed disarmed and grounded. Evidence:
`runs/independent/20261021T141447.196880Z/`.

On 2026-10-21 the independent runtime completed one live five-drone standalone SITL run: 10%
application-level peer loss, 50 ms mean latency, 2 m north mission, operator intent stream paused
for about 5 seconds and resumed, then all five landed. The fleet rose to about 5 m and moved as a
formation; minimum observed horizontal separation was 4.136 m. Raw events are in
`runs/independent/20261021T134539.157325Z/`. This tests mission-stream interruption, not termination
of the operator process. A corrected operator-process kill run also passed: agents continued the
accepted mission, then landed on intent expiry; all five explicitly reported landed and disarmed
within 42.98 seconds, without emergency fallback. Evidence:
`runs/independent/20261021T161122.758370Z/`. The earlier seed-50 record omitted armed state and is
not counted as disarm evidence. After lease expiry moved to a local monotonic deadline, seed 89
repeated the operator-loss scenario against the updated code: all five explicitly reported landed,
disarmed, and on the ground 42.99 seconds after operator loss, without fallback. Evidence:
`runs/independent/20261021T162235.986244Z/`. Five seeds (52, 89, 91, 92, 93) now pass this
same-host coordinator-loss scenario without emergency fallback. The harness requires each agent's
latest status to remain fresh and explicitly report landed, disarmed, and on ground; older near-ground
statuses that still showed armed are excluded. Seed 90's failed regression remains preserved.
Aggregate metrics and limitations are in
[`DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md`](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md)
and `runs/independent/operator_loss_batch_summary_20261021.json`.

The independent runtime now sends one MAVLink GCS heartbeat per second from each agent. This
allows an opt-in `--sitl-gcs-failsafe` profile to exercise ArduPilot's GCS failsafe in simulation:
`FS_GCS_ENABLE=5` (LAND), `FS_GCS_TIMEOUT=5`, and `FS_OPTIONS=0`. The profile is generated in the
run's evidence directory and does not edit `sitl/swarm_params.parm` or hardware parameter files.
One live agent-kill trial observed the simulated flight controller switch to LAND and disarm at
ground level. See the failsafe observation in
`runs/independent/20261021T142748.189010Z/failsafe_observation.json`.

For network fault injection, the console accepts `link-block <sender> <recipient>` and
`link-restore <sender> <recipient>`. Messages are test controls sent to the agent processes; each
sender modifies only its own directed egress edge. A two-way cut between drones 1 and 2 was
restored in one live five-drone SITL run, after which datagram sends resumed. This validates
transport toggling. The operator/test harness still chooses when to restore links. Evidence:
`runs/independent/20261021T143211.872887Z/`.

The independent agent sends a peer-liveness heartbeat every 0.5 seconds, including during blocking
EKF/arming/takeoff checks. This heartbeat is separate from position packets: only fresh position
packets are used for guidance, while the liveness message cannot refresh an old position estimate.
During flight, a peer is treated as stale after 1.5 seconds without a liveness heartbeat; if it is
still missing for a further 2-second grace period, the local agent commands LAND and reports
`peer_link_failsafe`, missing peer ID(s), and the reason in its status. Grounded agents record which
peers they have seen without starting a failsafe timer. Once landing starts, link recovery will not
trigger an automatic takeoff. The action is local to the affected agent, not a swarm-wide agreement:
other agents can continue their accepted mission.

Before arming, a node now requires both an unexpired accepted mission and fresh liveness from every
configured one-hop neighbor. On entering flight-time monitoring, the watchdog starts a grace timer
for every required neighbor that is missing, including peers never heard during preflight. A
preflight link failure therefore blocks launch; a link that disappears or remains absent after
takeoff reaches the same local LAND policy. Focused tests cover both never-seen peers and launch
preconditions. The first live regression (seed 90) exposed asynchronous takeoff timing and is
preserved as a failed run. After a five-second flight-monitoring startup grace was added, seed 91
completed the mission and operator-loss LAND scenario: all five landed and disarmed within 43.03
seconds after operator loss, with no emergency fallback and minimum time-aligned separation 4.199 m.
Evidence is under `runs/independent/20261021T163346.001631Z/` (failed seed 90) and
`runs/independent/20261021T163704.820929Z/` (successful seed 91). The complete 214-test software
suite subsequently passed. These results verify this regression path on localhost SITL, not a
reliability rate or real radio behavior.

The policy passed three five-drone standalone SITL trials (one interactive, two scripted seeds)
with a two-way D2↔D4 partition. In the scripted trials, the first failsafe status appeared 3.71 and
3.74 seconds after the link-cut events; endpoint touchdown took about 16.96 seconds. The other
three remained airborne until the operator's fleet LAND command, after which all five were
confirmed landed/disarmed. This is a small initial sample, not a reliability estimate. Evidence is
under `runs/independent/20261021T144908.936448Z/`,
`runs/independent/20261021T145947.252933Z/`, and
`runs/independent/20261021T150113.677325Z/`.

The independent runtime also supports seeded application-layer fault injection with
`--peer-duplicate-rate`, `--peer-reorder-rate`, and `--peer-reorder-delay-ms`. A duplicate retains
the original sequence number; receivers reject it as a replay. Reordering randomly adds variable
delay so a later sequence can arrive first; receivers reject the delayed old sequence instead of
replacing newer state. Per-agent event metrics count duplicates, delayed packets, and replay
rejections. These controls test freshness handling over localhost UDP; they are not a physical
radio or network-stack emulator. Seed 75 passed a five-drone D2↔D4 partition trial with 2% loss,
25 ms mean latency, 5% duplicate injection, and 25% reordering. Evidence and counters are in
`runs/independent/20261021T153950.467547Z/` and
`runs/independent/peer_partition_trials.json`.

`--peer-roster` configures peer, operator-command, and operator-status addresses. The
`--peer-auth-key-file` enables HMAC-SHA256 for peer datagrams, operator commands, and status
reports. Seed 76 passed with five distinct loopback addresses and zero authentication rejections;
this is same-host routing evidence only. `--external-agents` lets the operator run without starting
local processes, while each agent takes its own MAVLink endpoint. Command packets expire after five
seconds; status packets have per-agent session/sequence checks and the operator marks reports older
than three seconds stale. A five-agent fake-endpoint integration test verifies readiness, mission,
commands, status, LAND acknowledgement, and no local child processes. It is still a same-host
loopback test; the complete control path has not been verified across computers. See
[`PEER_NETWORK_SETUP.md`](PEER_NETWORK_SETUP.md). The UART boundary and physical limitations are
documented in [`HARDWARE_SOFTWARE_MAPPING.md`](HARDWARE_SOFTWARE_MAPPING.md).

Peer-state datagrams are also checked against the claimed sender's configured source address and
UDP port. A valid shared-key signature from a different endpoint is rejected and counted as
`rejected_source`. This guards against misrouted or casually injected packets; because the key is
shared, it does not establish unique cryptographic identity for each drone.
