# Decentralized spacing improvement — 2026-10-20

## Change made

The pure Python `SwarmSimulation` now defaults to a **3.0 m formation spacing** when
`control_mode="decentralized"`. Centralized and hybrid defaults remain at 2.5 m. An explicit
spacing passed to `set_formation` overrides either default. The config records this as a simulation
tuning choice based on this five-drone V-formation test, not a physical safety guarantee.

The repulsion gains were not increased. A coarse ten-seed sweep found that simply raising those
gains did not reliably solve the minimum-spacing shortfall. Increasing the desired V spacing to
3.0 m did, while keeping tracking error essentially unchanged. Wider 3.5 m and 4.0 m formations
also met the spacing criterion but take more room, so 3.0 m is the smaller tested buffer.

## Corrected paired experiment

The earlier comparison initialized the drones in a wider V than the target. That initial
comparison is preserved but superseded; the corrected script now initializes from the same target
geometry, with the same seeded small position perturbation, for both modes in each paired trial.

Both controllers were run for ten paired seeds (20261020–20261029) at **both** 2.5 m and 3.0 m
target spacing. Conditions were held constant within each pair: five drones, SITL-fitted dynamics,
0.08 m GPS noise, 10% packet loss, 50 ms mean channel latency, and the same hold / 6-second north
move / hold mission. Reported intervals are 95% Student-t intervals across ten trial means;
controller differences use paired per-seed differences.

| Target spacing | Controller | Steady mean tracking error (95% CI) | Mean of trial minimum spacing (95% CI) | Worst trial minimum spacing | Trials ≥2.5 m |
|---:|---|---:|---:|---:|---:|
| 2.5 m | Centralized | 0.111 m (0.092–0.130) | 2.286 m (2.223–2.349) | 2.109 m | 0/10 |
| 2.5 m | Decentralized | 0.236 m (0.217–0.254) | 2.285 m (2.224–2.346) | 2.111 m | 0/10 |
| 3.0 m | Centralized | 0.111 m (0.092–0.130) | 2.784 m (2.721–2.848) | 2.606 m | 10/10 |
| 3.0 m | Decentralized | 0.236 m (0.217–0.254) | 2.783 m (2.723–2.844) | 2.609 m | 10/10 |

At 3.0 m spacing, decentralized tracking error remained worse than centralized: the paired
steady-mean-error difference was **+0.125 m** (95% CI **+0.115 to +0.135 m**), and the paired
steady-window p95-error difference was **+0.277 m** (95% CI **+0.244 to +0.310 m**). The paired
difference in minimum spacing was **−0.001 m** (95% CI **−0.012 to +0.010 m**), with no clear
mode difference for this metric at that spacing. Neither mode had a simulated collision in these
trials.

This indicates the lower spacing at 2.5 m target was primarily a **formation margin problem** in
this setup, rather than a benefit that can be recovered by increasing only the decentralized
repulsion gain. The decentralized controller still needs tracking improvement.

## Verification

- Parameter sweep: six spacing/gain configurations, ten trials per configuration; raw data:
  `experiments/results/decentralized_5drone_spacing_sweep_20261020.json`.
- Corrected paired data at 2.5 m:
  `experiments/results/decentralized_5drone_paired_spacing25_20261020.json`.
- Corrected paired data at 3.0 m:
  `experiments/results/decentralized_5drone_paired_spacing30_20261020.json`.
- Full automated suite: **167 passed**.
- The five-drone impaired-peer SITL run remains valid: its console uses the independent
  `formation_library` path and already measured 4.139 m minimum separation. This spacing-default
  code change affects `SwarmSimulation`, not the live SITL console.

Reproduce the paired comparison with:

```bash
python3 experiments/compare_5drone_architectures.py --trials 10 --seed 20261020 --spacing 3.0
```

## Limits and next work

The result covers one five-drone V geometry, one moving mission, one dynamics profile, and one
communication impairment setting. It does not establish that 3.0 m is sufficient for other
formations, noisy real GPS, larger swarms, Gazebo, or physical drones. The potential-field
controller is still not a hard collision constraint. Next, retain the 3.0 m default for this
decentralized simulator path, repeat across more conditions, and improve the decentralized
tracking controller before claiming parity with centralized control.
