"""Build a validated, matched-mission comparison from three 10-drone SITL runs.

This summarizes one recorded run per architecture. It deliberately reports no
confidence intervals; repeated-seed uncertainty is provided separately by the
pure-Python comparison.
"""

import argparse
import json
import math
from pathlib import Path


def _load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _p95(values):
    values = sorted(float(value) for value in values)
    if not values:
        raise ValueError("cannot calculate p95 without observations")
    # Nearest-rank p95, used consistently for the 10-agent terminal snapshot.
    return values[max(0, math.ceil(0.95 * len(values)) - 1)]


def _centralized_snapshot(events_path: Path, expected_drones: int):
    rows = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()
            if line.strip()]
    starts = [row for row in rows if row.get("kind") == "run_start"]
    modes = [row for row in rows if row.get("kind") == "control_mode"]
    commands = [row for row in rows if row.get("kind") == "centroid_command"]
    move_commands = [row for row in commands
                     if math.isclose(float(row.get("north_m", 0)), 2.0, abs_tol=1e-6)
                     and math.isclose(float(row.get("east_m", 0)), 0.0, abs_tol=1e-6)]
    if not starts or starts[0].get("drones") != expected_drones:
        raise ValueError("centralized event log does not match the requested fleet size")
    if not modes or modes[0].get("mode") != "centralized":
        raise ValueError("centralized event log has the wrong control mode")
    if not move_commands:
        raise ValueError("centralized event log is missing the 2 m north mission command")
    command_index = rows.index(move_commands[-1])
    settled = [row for row in rows[command_index + 1:]
               if row.get("kind") == "sample" and row.get("settled") is True
               and len(row.get("positions_ned", [])) == expected_drones
               and len(row.get("targets_ned", [])) == expected_drones
               and all(row.get("active", [])) and all(row.get("telemetry_fresh", []))
               and all(position is not None and target is not None
                       for position, target in zip(row["positions_ned"], row["targets_ned"]))]
    if not settled:
        raise ValueError("centralized event log has no settled full-fleet sample after the move")
    sample = settled[-1]
    errors = [math.dist(position[:3], target[:3])
              for position, target in zip(sample["positions_ned"], sample["targets_ned"])]
    return {
        "terminal_snapshot_utc": sample.get("utc"),
        "terminal_snapshot_p95_3d_slot_error_m": _p95(errors),
        "minimum_sampled_separation_m": None,
        "converged": True,
        "all_landed_disarmed": True,
        "evidence_samples": len(settled),
        "run_start": starts[0],
    }


def _independent_summary(path: Path, expected_mode: str, expected_drones: int,
                         expected_seed: int, expected_peer):
    data = _load(path)
    if data.get("control_mode") != expected_mode or data.get("drones") != expected_drones:
        raise ValueError(f"{path} does not match mode={expected_mode}, drones={expected_drones}")
    if data.get("seed") != expected_seed:
        raise ValueError(f"{path} does not use the paired peer-channel seed")
    actual_peer = {
        "peer_loss": data.get("peer_loss"),
        "peer_latency_ms": data.get("peer_latency_ms"),
        "peer_jitter_ms": data.get("peer_jitter_ms"),
        "peer_burst_loss": data.get("peer_burst_loss"),
    }
    if actual_peer != expected_peer:
        raise ValueError(f"{path} has a different peer-channel profile: {actual_peer}")
    convergence = data.get("mission_convergence", {})
    if (not data.get("passed") or not convergence.get("converged")
            or convergence.get("goal_ne_m") != [2.0, 0.0]
            or convergence.get("formation") != "v"
            or convergence.get("target_altitude_m") != 5.0
            or not data.get("all_drones_landed_disarmed")):
        raise ValueError(f"{path} failed the shared mission/convergence/landing checks")
    errors = convergence.get("latest_per_agent_errors", {})
    if len(errors) != expected_drones:
        raise ValueError(f"{path} is missing per-agent terminal convergence errors")
    terminal_errors = [math.sqrt(float(error["horizontal_error_m"]) ** 2
                                 + float(error["vertical_error_m"]) ** 2)
                       for error in errors.values()]
    separation = data.get("separation_metrics", {})
    if not separation.get("safety_gate_passed"):
        raise ValueError(f"{path} failed its sampled separation gate")
    return {
        "terminal_snapshot_utc": convergence.get("latest_timestamp"),
        "terminal_snapshot_p95_3d_slot_error_m": _p95(terminal_errors),
        "minimum_sampled_separation_m": separation.get("minimum_horizontal_separation_m"),
        "converged": convergence["converged"],
        "all_landed_disarmed": data["all_drones_landed_disarmed"],
        "evidence_samples": convergence.get("aligned_samples"),
        "source_summary": str(path),
        "reproducibility": data.get("reproducibility"),
        "coordinator_switch_test": data.get("coordinator_switch_test"),
        "soak_test": data.get("soak_test"),
    }


def build_comparison(central_summary_path: Path, central_events_path: Path,
                     decentralized_path: Path, hybrid_path: Path,
                     output_path: Path):
    central = _load(central_summary_path)
    if (central.get("drones") != 10 or central.get("outcome") != "landed"
            or not central.get("pass")):
        raise ValueError("centralized run did not pass its 10-drone console criteria")
    central_snapshot = _centralized_snapshot(central_events_path, 10)
    central_snapshot["minimum_sampled_separation_m"] = central.get("minimum_separation_m")
    central_snapshot["all_landed_disarmed"] = central["checks"]["landing_confirmed"]
    central_snapshot["source_summary"] = str(central_summary_path)

    peer = {
        "peer_loss": 0.02,
        "peer_latency_ms": 25.0,
        "peer_jitter_ms": 5.0,
        "peer_burst_loss": False,
    }
    decentralized = _independent_summary(decentralized_path, "decentralized", 10, 1024, peer)
    hybrid = _independent_summary(hybrid_path, "hybrid", 10, 1024, peer)
    output = {
        "kind": "matched_three_mode_sitl_comparison",
        "drones": 10,
        "mission": {
            "formation": "v",
            "goal_ne_m": [2.0, 0.0],
            "target_altitude_m": 5.0,
            "horizontal_and_vertical_convergence_tolerance_m": 0.5,
            "peer_channel_for_decentralized_and_hybrid": peer,
            "paired_seed": 1024,
        },
        "results": {
            "centralized": central_snapshot,
            "decentralized": decentralized,
            "hybrid": hybrid,
        },
        "acceptance": {
            "all_modes_converged": all(row["converged"] for row in
                                        (central_snapshot, decentralized, hybrid)),
            "all_modes_landed_disarmed": all(row["all_landed_disarmed"] for row in
                                              (central_snapshot, decentralized, hybrid)),
            "all_modes_sampled_separation_at_least_2_5m": all(
                row["minimum_sampled_separation_m"] is not None
                and row["minimum_sampled_separation_m"] >= 2.5
                for row in (central_snapshot, decentralized, hybrid)
            ),
        },
        "scope_and_caveats": [
            "One real ArduPilot SITL run per mode; no confidence intervals or statistical significance claim.",
            "All modes used ten vehicles, V formation, 5 m target altitude, 2 m north goal, and the same convergence tolerance.",
            "Decentralized and hybrid used the same peer-channel profile and seed. Centralized has no peer-to-peer channel by design.",
            "The independent-agent modes use separate agent processes and loopback UDP; this is not a separate-computer LAN or physical-radio test.",
            "The terminal p95 is computed across ten vehicles at the last settled/aligned mission snapshot; centralized evidence uses telemetry positions versus assigned slots, while independent evidence uses validator position-versus-slot errors.",
            "Minimum separation is sampled telemetry, not a continuous collision guarantee.",
        ],
    }
    output["pass"] = all(output["acceptance"].values())
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--central-summary", type=Path, required=True)
    parser.add_argument("--central-events", type=Path, required=True)
    parser.add_argument("--decentralized-summary", type=Path, required=True)
    parser.add_argument("--hybrid-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = build_comparison(args.central_summary, args.central_events,
                              args.decentralized_summary, args.hybrid_summary,
                              args.output)
    print(json.dumps({"pass": result["pass"], "output": str(args.output),
                      "acceptance": result["acceptance"],
                      "terminal_p95_slot_error_m": {
                          mode: row["terminal_snapshot_p95_3d_slot_error_m"]
                          for mode, row in result["results"].items()},
                      "minimum_separation_m": {
                          mode: row["minimum_sampled_separation_m"]
                          for mode, row in result["results"].items()}}, indent=2))


if __name__ == "__main__":
    main()
