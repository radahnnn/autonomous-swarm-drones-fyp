# Formation-transition protocol — design spec (DRAFT for Codex review)

Status: **state machines implemented and unit-tested** in `sitl/transition_protocol.py` (pure logic, `tests/test_transition_protocol.py`); **not yet wired** into the agent node or console, and no SITL evidence. Defaults taken for the open questions: abort = HOLD, and coordinator loss during a transition = HOLD (conservative). Written 2026-10-29 by Claude against commit `1b700b8`. Nothing here is validated; thresholds are starting values to be calibrated from measured latency and clock-sync uncertainty.

Accurate claim if implemented: *centralized transition planning, distributed synchronized execution, and distributed local supervision.* It is **not** decentralized formation negotiation.

## 1. Problem (what the code does today, with references)

Coordinator `sitl/independent_swarm_console.py` `_formation_command` (~L940–1040) plans a morph with `plan_formation_transition` (collision-screened paths), sets `formation_transition_started_at = now`, and `_publish_intent` (L565) broadcasts it once per second over UDP. Agents (`sitl/independent_agent_node.py`, mission handler ~L380–460 and slot interpolation ~L813–835) start from `intent.issued_at` and follow a quintic-smoothstep path. Gaps:

| # | Gap | Consequence |
|---|---|---|
| G1 | No acknowledgement of the plan | An agent that misses the intent (UDP loss, restart, link block) does not move while the others do; the clearance the planner screened no longer holds. |
| G2 | Start = "when the coordinator published" | A late receiver computes progress from `issued_at` and jumps ahead; there is no common start. |
| G3 | Only the coordinator supervises (`_formation_transition_abort_reason`: status age ≤ 2 s, airborne/armed, `safety_state`, live separation ≥ 2.8 m, 12 s overrun) | If the coordinator dies, nobody watches separation. Agents have no local abort. |
| G4 | The only abort action is LAND ALL | Overly drastic for a late ack or a transient stale reading; landing 5–10 drones in formation is itself a hazard. |
| G5 | Command gating is partial. **Verified 2026-10-29:** the REPL (`handle`) blocks most actions during a transition, but `_operator_command` (launch/land) has no transition check, and the dashboard blocks other actions only in browser JavaScript (buttons disabled). The dashboard's server-side `formation_transitioning` reads `session.morph_steps_remaining` (older console attribute), not the independent console's `formation_transition_deadline`, so it is likely always false there (links to the `session.count` vs `SwarmConsole.n` mismatch). | A conflicting HTTP/API command during a morph is not rejected server-side. |
| G6 | No defined behaviour for link loss, killed agents or membership change mid-transition | Undefined; the 4.6 GB log-spam run shows dropped links are not benign. |

## 2. Goals and non-goals

Goals: (a) no vehicle moves unless every vehicle has acknowledged the same plan; (b) all vehicles start at one agreed time; (c) any vehicle or the coordinator can stop the transition safely; (d) defined, tested behaviour for loss, restart and membership change; (e) stale/replayed/duplicate messages cannot change state (validate before mutate, as fixed in `1b700b8`).

Non-goals: decentralized planning; obstacle sensing; physical-radio guarantees; changing the existing mission/goal movement.

## 3. State machine

Per transition there is one `transition_id` (strictly increasing integer per clock-sync epoch). Coordinator states and agent states:

```
Coordinator: IDLE → PREPARING → COMMITTED → RUNNING → SETTLING → IDLE
                       │            │          │          │
                       └────────────┴──────────┴──────────┴──► ABORTING → HOLDING → IDLE
Agent:       IDLE → STAGED → RUNNING → ARRIVED → IDLE
                     │         │         │
                     └─────────┴─────────┴──► HOLD (local) → IDLE
```

- **PREPARING**: coordinator plans, chooses `start_at = now + lead_s` (operator clock), sends `transition_prepare` to every agent.
- **STAGED** (agent): plan validated and stored; **vehicle does not move**; agent replies `transition_ack`.
- **COMMITTED**: all N acks for this id received within `ack_timeout_s` → coordinator sends `transition_go` (carries the same `start_at`).
- **RUNNING**: from `start_at` each agent follows its path (existing smoothstep); progress α is computed from the common clock only.
- **SETTLING**: after `duration_s`, coordinator waits for fresh telemetry showing every agent within `arrival_tol_m` of its slot for `dwell_s` (generalizes the existing 0.75 m / 2 s check). Then IDLE and the resume-goal logic runs as today.
- **HOLD** (agent) / **ABORTING→HOLDING** (coordinator): every agent freezes its *slot* at the progress value α_freeze and keeps flying position-hold on the frozen slot. It does **not** LAND and does **not** continue. LAND is a separate, operator- or fault-triggered action (§7).

## 4. Messages

All messages travel on the existing authenticated operator↔node UDP channel (HMAC, epoch, endpoint checks). Every message carries `version`, `epoch` (added and checked by the transport layer) and `transition_id`; replay protection is by `transition_id` (strictly increasing, rejected once closed) plus idempotent handling, so no separate `sequence` field is needed. Receivers validate (epoch, id/sequence, field shapes, finiteness, spacing as in `MissionIntent.from_dict`) **before** changing any state.

| Type | Direction | Fields (beyond common) |
|---|---|---|
| `transition_prepare` | coordinator → agents | `formation`, `target_slots[N][3]`, `start_slots[N][3]`, `duration_s`, `start_at`, `plan_hash`, `abort_if_no_go_by` |
| `transition_ack` | agent → coordinator | `plan_hash`, `ok`, `reason` (NACK codes below), `own_position_error_m` |
| `transition_go` | coordinator → agents | `plan_hash`, `start_at` |
| `transition_abort` | either → all | `reason`, `freeze_at` (common-clock time) |
| `transition_done` | coordinator → agents | `plan_hash` (sent after the arrival dwell; an ARRIVED agent also self-closes after `arrived_close_s` = 7 s so a lost `done` cannot trap it) |
| `transition_status` | agent → coordinator (inside the existing 1 Hz status, plus on change) | `state`, `transition_id`, `progress`, `hold_reason` |

`plan_hash` = SHA-256 over canonical JSON of the plan fields; an ack or go for a different hash is ignored. Duplicates are idempotent (same id + same hash → resend the same reply, no state change). `transition_id` ≤ last completed/aborted id is rejected (replay).

**Agent NACK reasons** (checked at STAGED): `not_airborne_armed`, `stale_own_telemetry`, `stale_peer_links`, `already_in_transition`, `plan_invalid`, `start_too_soon` (cannot be staged before `start_at − min_lead_s`), `position_mismatch` (own position farther than `start_tol_m` from its reported start slot), `epoch_mismatch`.

## 5. Timing (initial values, to calibrate)

| Parameter | Start value | Reason |
|---|---|---|
| `lead_s` (prepare→start) | 4.0 s | Covers `ack_timeout_s` plus one retransmit plus clock-sync uncertainty; status heartbeat is 1 Hz. |
| `ack_timeout_s` | 1.5 s, prepare retransmitted at 0.5 s intervals | UDP loss tolerance without delaying start unduly. |
| `min_lead_s` (agent) | 1.0 s | An agent that receives `go` closer than this to `start_at` NACKs `start_too_soon`. |
| `go` late tolerance | 0.25 s | If an agent gets `go` after `start_at + 0.25 s` it does not start; it NACKs and the coordinator aborts. |
| `abort_if_no_go_by` | `start_at + 1.0 s` | A STAGED agent self-discards the plan if no `go` arrives (fail-safe to "do not move"). |
| `start_tol_m` / `arrival_tol_m` / `dwell_s` | 1.0 m / 0.75 m / 2.0 s | `arrival_tol_m`/`dwell_s` already used by the console. |
| Telemetry freshness | status ≤ 2 s, position ≤ 1 s | Same as existing checks. |
| Clock-sync uncertainty cap | refuse if > 0.15 s | From `sitl/clock_sync.py`; must be measured per run. |

## 6. Supervision

**Coordinator** (extends `_formation_transition_abort_reason`): every agent's status fresh; agent state matches the coordinator's state (a missing `RUNNING` after `start_at + 1 s` → abort); armed/airborne; `safety_state == nominal`; live pairwise separation ≥ `abort_separation_m` (2.8 m today); overrun > `duration_s + 12 s`. Any failure → `transition_abort` (HOLD), not LAND.

**Each agent, independently of the coordinator** (new): while STAGED/RUNNING it checks (a) own telemetry fresh and vehicle armed/airborne; (b) every plan participant that is a live peer reports a position within `peer_track_tol_m` of the position the plan predicts at the current α (peer-verified progress), and no peer's state is older than 1 s; (c) minimum live separation to any peer ≥ `local_hold_separation_m` (start: 2.6 m, below the 2.8 m coordinator abort and above the 2.5 m acceptance gate so local hold fires first only on genuine closure). On violation: enter HOLD locally, broadcast `transition_abort` to peers and coordinator. A peer receiving an abort freezes at `freeze_at`.

## 7. Failure semantics

| Event | Behaviour |
|---|---|
| Missing/NACK ack, bad plan | Abort before start: nothing moved, no HOLD needed. Report the reason in the structured command result. |
| Prepare/go lost to one agent | Retransmit within the ack window; if still missing, abort before start. After start, a missing `go` means the agent never started → others' coordinator check abort (HOLD) within 1 s of `start_at`. |
| Duplicate/reordered/stale message | Ignored without state change (idempotent or rejected by id/sequence). |
| Agent process killed or vehicle disarmed/crashed mid-transition | Coordinator and peers detect stale status/peer state → HOLD for the rest. No re-plan during the transition. Re-plan for N−1 is allowed only after HOLD, as a new transition. |
| Coordinator lost (no coordinator message for `coord_loss_s`, start 3 s) | Agents continue **only if** peer-verified progress holds (§6b) and separation is nominal; otherwise HOLD. On completing, agents hold the target slots and use the existing coordinator-loss logic (decentralized/hybrid mode). |
| One agent loses all peers | Agent HOLDs and invokes the existing peer failsafe (`_check_peer_failsafe`). |
| Membership change (adaptive membership) | Frozen during a transition; changes apply after IDLE. |
| HOLD persists > `hold_timeout_s` (start 20 s) | Coordinator reports `needs_operator`; **LAND** is never automatic from HOLD except on a vehicle fault/low battery already handled elsewhere. Operator may `land` at any time. |
| Operator `land` | Always allowed; supersedes any transition. |

## 8. Command gating

While coordinator state ≠ IDLE: reject `launch`, move/goal, formation, mode change, fault injection, recover/revive with a structured result `{ok:false, code:"transition_active", transition_id}`. Allowed: `status`, `abort` (HOLD), `land`. This must be enforced server-side in one shared command path used by the REPL (`handle`), `_operator_command` and every dashboard endpoint, not in browser JavaScript (see G5, verified).

## 9. Implementation plan

1. Pure state-machine module `sitl/transition_protocol.py` (no I/O, injectable clock) for coordinator and agent logic, with schemas and validators.
2. Wire agent side in `independent_agent_node.py` (message kinds, STAGED/RUNNING/HOLD, local supervision); wire coordinator side in `independent_swarm_console.py`; stop using `MissionIntent.formation_transition_*` for live morphs (keep fields for the grounded case; deprecate rather than break).
3. Structured command results + gating (backlog items 5 and 7).
4. Tests, in order of evidence: **UNIT** state-machine tables (every state × event, incl. duplicate, replay, late go, NACK each reason); **INTEGRATION** with a fake lossy/reordering/duplicating transport (property-style over seeds); then **SITL** 5 and 10 drones (same host), then **VM-NETWORK**, with kill-an-agent, drop-go, drop-ack and coordinator-kill runs. Only SITL/VM runs count as live evidence.

## 10. Open questions for Codex

1. Is HOLD (freeze slot at `freeze_at`) safe when paths are screened only for the *planned* time-parametrization? Staggered freezing could leave two drones at different α. Alternative: freeze at the locally observed α, with `local_hold_separation_m` margin. Which is safer?
2. Should `start_at` use the operator clock only, with agents refusing if clock-sync uncertainty exceeds the cap, or also a monotonic-clock countdown after `go` arrives?
3. Is continuing after coordinator loss (peer-verified) acceptable, or should coordinator loss always HOLD? The conservative choice is HOLD; the useful choice (hybrid-mode claim) is to continue.
4. Remove `formation_transition_*` from `MissionIntent` now or deprecate for one cycle (tests depend on them)?
5. Threshold review: `lead_s`, `local_hold_separation_m`, `peer_track_tol_m` need measured numbers — which run data should calibrate them?
