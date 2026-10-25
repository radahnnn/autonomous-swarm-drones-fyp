# Five-agent SITL across the Ubuntu host and Ubuntu Server VM — 2026-10-25

## Setup and scope

The Ubuntu desktop ran five standalone ArduCopter SITL instances, the operator,
and agents D1–D2. The Ubuntu Server 24.04 VM at `192.168.100.2` ran agents
D3–D5. The host-only TAP interface was `192.168.100.1`. Remote agents reached
the host's SITL TCP ports 5780, 5790, and 5800; each agent exchanged the
project's HMAC-authenticated peer UDP messages using the shared roster in
[`evidence/VM_SPLIT_2026-10-25/roster.json`](evidence/VM_SPLIT_2026-10-25/roster.json).
The disposable test key is intentionally excluded from the repository.

This is a two-OS-instance test on **one physical PC**, not a radio or aircraft
test. It used five agents because the documented split-host V topology assigns
D1–D2 to the host and D3–D5 to the VM. Ten-drone same-host results are a
different test; ten across this boundary was tested later the same day. See the
[ten-agent report](VM_SPLIT_TEN_AGENT_SITL_2026-10-25.md).

## Results

An initial authenticated two-agent peer smoke test delivered 48 state packets
to the host and 47 to the VM in five seconds. Both sides identified a fresh
peer; authentication and source-address rejection counters were zero. Its
short-lived first attempt started the two ends at different times and received
no packets. Starting both ends together resolved that test setup issue.

The first five-agent attempt connected all five SITL endpoints and reported
all agents ready, but the operator's peer-link preflight failed. D5 frequently
reported its required D3 link missing, and the operator did not launch the
swarm. D5's final peer counter had 686 stale rejections versus 147 accepted
packets. Its estimated clock offset was about 12 ms greater than D3's, so the
existing peer receiver translated promptly received D3 packets slightly into
the future and rejected them under its exact `age >= 0` rule. The failure and
grounded statuses are retained in
[`first_preflight_events.jsonl`](evidence/VM_SPLIT_2026-10-25/first_preflight_events.jsonl).

The peer receiver now allows up to 50 ms of future timestamp error for state
and heartbeat packets, while retaining the 0.5 s state-age and 2.0 s heartbeat
age limits. A regression test accepts a 10 ms estimation error and rejects a
90 ms future error. This is a bounded tolerance for clock-estimation error;
it is not proof that the clocks are synchronized to 50 ms under every network.
The complete project suite passed after the fix (289 tests collected and run).

With the fix copied to both hosts, the second run passed the five-agent
preflight with every required link live. All five SITL aircraft took off to
5 m, accepted a 2 m north mission, moved north, and landed/disarmed after the
operator sent `q`. Final north positions were 1.875, -1.109, -1.143, -4.127,
and -4.105 m, versus V-slot targets of 2, -1, -1, -4, and -4 m. All five final
statuses were `landed`, `armed=false`, `airborne=false`, with required peers
still live. The D5 peer counter finished at 502 accepted packets, 59 stale,
and zero source or authentication rejections. The other four also reported
zero source and authentication rejections.

Time-aligning airborne status samples within 250 ms produced 34 full-fleet
samples, a **minimum sampled horizontal separation of 4.188 m**, and a
maximum alignment gap of 67 ms. The 2 m mission's all-sample horizontal
tracking RMSE was 1.011 m and p95 was 1.765 m because the samples include
the move itself; pairwise formation-offset RMSE was 0.026 m. These sampled
values do not bound separation between reports or measure physical flight.
The successful run's raw operator events, five agent logs, and five SITL logs
are in [`evidence/VM_SPLIT_2026-10-25/`](evidence/VM_SPLIT_2026-10-25/).

The temporary test agents and SITL listeners were stopped after all five
vehicles reported disarmed. The VM remained running. No physical drone was
connected or armed.

## Next test

Repeat the five- and ten-agent split-host runs across multiple seeds and with
controlled network delay, loss, clock skew, and directed link faults. Track
launch-preflight, mission, landing, spacing, and packet
rejection rates for every run. A separate physical computer and actual radio
link remain later evidence gates.
