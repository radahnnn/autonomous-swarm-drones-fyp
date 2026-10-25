# Ten-drone reliability validation — 2026-10-25

## Result

The software reliability campaign passed on the current decentralized SITL branch. Five independent ten-agent external-ArduPilot SITL runs completed; every run reported `passed: true`, and all ten vehicles landed and disarmed. The suite also passed **289 tests** when run by itself.

This is software-in-the-loop evidence. The agents run independent ArduPilot SITL instances, but the fault injection uses loopback peer networking on the host. It does not validate physical radios, a real vehicle, or the VM's actual LAN path in these new fault cases.

## SITL cases

| Run | Mode | Fault or recovery exercised | Outcome |
|---|---|---|---|
| `external-sitl-20261025T065114.864333Z` | Decentralized, 10 agents | 2% peer loss; 25 ms mean injected delay; 8 ms jitter; directed peer cut and restore | Passed; all landed/disarmed; nominal state recovered |
| `external-sitl-20261025T065323.939422Z` | Hybrid, 10 agents | Same impairment profile; coordinator handover; directed peer cut and restore | Passed; all landed/disarmed; fallback and recovery recorded |
| `external-sitl-20261025T065533.217194Z` | Decentralized, 10 agents | Required peer blocked before launch; launch preflight refusal; restore then mission | Passed; refusal and recovery passed; all landed/disarmed |
| `external-sitl-20261025T070010.823598Z` | Decentralized, 10 agents | Operator process terminated in flight | Passed; all ten requested mission-expiry landing and landed/disarmed within 43.31 s |
| `external-sitl-20261025T070240.496314Z` | Hybrid, 10 agents | Coordinator handover plus operator process restart in flight | Passed; all agents fell back and recovered; replacement operator took over and resumed the 2 m north goal; all landed/disarmed |

Full raw run evidence is in the local ignored `runs/independent/` directory. Its per-run `operator-peer.key` files are disposable secrets and must not be committed. Run summaries contain agent and SITL process IDs that are historical after cleanup.

## Matched three-mode comparison

The paired comparison used 10 drones, 20 fixed seeds per condition, and three impairment scenarios (180 trials total): nominal impaired network, repeated outages, and burst repeated outages. Every mode passed 20/20 trials in all three scenarios against the model criteria (minimum separation 2.5 m; steady-state 95th-percentile error below 1 m).

Mean steady tracking error (95% interval across seeds):

| Scenario | Centralized | Decentralized | Hybrid |
|---|---:|---:|---:|
| Nominal impaired | 0.114 m (0.095–0.133) | 0.131 m (0.116–0.145) | 0.114 m (0.095–0.133) |
| Repeated outages | 0.114 m (0.095–0.133) | 0.183 m (0.169–0.197) | 0.114 m (0.095–0.133) |
| Burst repeated outages | 0.114 m (0.095–0.133) | 0.148 m (0.132–0.165) | 0.114 m (0.095–0.133) |

In this model, decentralized control incurs somewhat more tracking error during outages; hybrid remains close to centralized. This result is consistent with the current hybrid coordinator-assisted behavior, and does not establish that hybrid is universally superior. The hybrid and centralized controller implementations are similar in this engine, and the experiment uses centrally supplied goals in every mode.

The machine-readable full results, per-trial data, conditions, and caveats are in [THREE_MODE_10_DRONE_20_SEED_2026-10-25.json](THREE_MODE_10_DRONE_20_SEED_2026-10-25.json). Scope is pure-Python simulation with assumed baseline dynamics, not the SITL controller adapter.

## Test-suite note

An initial test-suite attempt ran concurrently with an SITL run and one UDP transport test collided on its PID-derived local port. After the SITL run ended, the suite was rerun alone and passed all 289 tests. This points to test/SITL concurrency causing a transient local-port collision; it was not a repeatable failure in an isolated suite.

## Limits and next evidence

- Injected packet loss and delay are software shapers, not measured drone-radio behavior.
- SITL agents used loopback transport in this campaign, not the host-to-VM network path.
- The Python mode comparison is not ArduPilot SITL and not physical identification.
- The fallback landing behavior is demonstrated in SITL only; do not infer real-flight safety from it.
- Next software milestone: repeat mode/fault cases across the real host/VM link, then test a second computer or VM as an independent agent host. Physical radio and flight tests remain a separate supervised hardware stage.
