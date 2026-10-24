#!/usr/bin/env python3
"""Aggregate comparable external-agent SITL summaries without mixing profiles."""

import argparse
import json
import math
from pathlib import Path


PROFILE_FIELDS = (
    "topology",
    "peer_loss",
    "peer_latency_ms",
    "peer_jitter_ms",
    "peer_burst_loss",
    "peer_burst_start_rate",
    "peer_burst_continue_rate",
    "peer_duplicate_rate",
    "peer_reorder_rate",
    "peer_reorder_delay_ms",
)
PROFILE_DEFAULTS = {
    # Older summaries used these fixed values before they became CLI options.
    "peer_burst_start_rate": 0.04,
    "peer_burst_continue_rate": 0.65,
}


def profile_for(summary: dict) -> dict:
    """Return the settings that must match before runs can be compared."""
    missing = [
        field for field in PROFILE_FIELDS
        if field not in summary and field not in PROFILE_DEFAULTS
    ]
    if missing:
        raise ValueError(f"summary is missing profile fields: {', '.join(missing)}")
    return {
        **{
            field: summary.get(field, PROFILE_DEFAULTS.get(field))
            for field in PROFILE_FIELDS
        },
        "operator_loss": "operator_loss" in summary,
        "operator_restart": "operator_restart" in summary,
    }


def wilson_interval(successes: int, trials: int, z: float = 1.959963984540054) -> list[float]:
    """Return a 95% Wilson score interval for a binomial pass fraction."""
    if trials <= 0 or not 0 <= successes <= trials:
        raise ValueError("successes and trials must describe a non-empty binomial sample")
    fraction = successes / trials
    denominator = 1.0 + z * z / trials
    centre = (fraction + z * z / (2.0 * trials)) / denominator
    margin = z * math.sqrt(
        fraction * (1.0 - fraction) / trials + z * z / (4.0 * trials * trials)
    ) / denominator
    return [max(0.0, centre - margin), min(1.0, centre + margin)]


def summarize(summaries: list[tuple[Path, dict]]) -> dict:
    if not summaries:
        raise ValueError("at least one run summary is required")
    expected_profile = profile_for(summaries[0][1])
    rows = []
    for path, summary in summaries:
        if summary.get("kind") != "external_agent_sitl_validation":
            raise ValueError(f"{path}: not an external-agent SITL summary")
        if "passed" not in summary or "completed_utc" not in summary:
            raise ValueError(f"{path}: incomplete run summary")
        actual_profile = profile_for(summary)
        if actual_profile != expected_profile:
            raise ValueError(
                f"{path}: profile differs from the first run: "
                f"expected {expected_profile}, got {actual_profile}"
            )
        separation = summary.get("separation_metrics", {}).get(
            "minimum_horizontal_separation_m"
        )
        convergence = summary.get("mission_convergence", {})
        operator_loss = summary.get("operator_loss")
        peer_metrics = summary.get("peer_metrics", {}).values()
        dropped_loss = sum(int(item.get("dropped_loss", 0)) for item in peer_metrics)
        impairment_samples = sum(
            int(item.get("injected_delay_samples", 0))
            for item in summary.get("peer_metrics", {}).values()
        )
        row = {
            "path": str(path),
            "seed": summary.get("seed"),
            "passed": summary["passed"] is True,
            "converged": convergence.get("converged") is True,
            "all_five_landed_disarmed": summary.get("all_five_landed_disarmed") is True,
            "minimum_horizontal_separation_m": separation,
            "duplicate_packets": sum(int(item.get("duplicated_packets", 0)) for item in peer_metrics),
            "reordered_packets": sum(int(item.get("reorder_delayed_packets", 0)) for item in summary.get("peer_metrics", {}).values()),
            "replay_rejections": sum(int(item.get("rejected_replay", 0)) for item in summary.get("peer_metrics", {}).values()),
            "dropped_loss_packets": dropped_loss,
            "accepted_for_impairment_queue": impairment_samples,
            "empirical_loss_fraction": (
                dropped_loss / (dropped_loss + impairment_samples)
                if dropped_loss + impairment_samples else None
            ),
            "completed_utc": summary["completed_utc"],
        }
        if operator_loss is not None:
            row["operator_loss"] = {
                "operator_process_killed": operator_loss.get("operator_process_killed") is True,
                "all_agents_were_airborne": operator_loss.get("all_agents_were_airborne") is True,
                "all_agents_requested_mission_expiry_land": (
                    operator_loss.get("all_agents_requested_mission_expiry_land") is True
                ),
                "all_agents_landed_disarmed": (
                    operator_loss.get("all_agents_landed_disarmed") is True
                ),
                "seconds_after_operator_loss": operator_loss.get("seconds_after_operator_loss"),
            }
        rows.append(row)

    passed = sum(row["passed"] for row in rows)
    separations = [
        float(row["minimum_horizontal_separation_m"])
        for row in rows
        if row["minimum_horizontal_separation_m"] is not None
    ]
    total_dropped_loss = sum(row["dropped_loss_packets"] for row in rows)
    total_impairment_samples = sum(row["accepted_for_impairment_queue"] for row in rows)
    result = {
        "kind": "external_agent_sitl_batch_summary",
        "profile": expected_profile,
        "run_count": len(rows),
        "passed_count": passed,
        "failed_count": len(rows) - passed,
        "observed_pass_fraction": passed / len(rows),
        "pass_fraction_wilson_95": wilson_interval(passed, len(rows)),
        "all_runs_converged": all(row["converged"] for row in rows),
        "all_runs_landed_disarmed": all(row["all_five_landed_disarmed"] for row in rows),
        "minimum_horizontal_separation_across_runs_m": min(separations) if separations else None,
        "total_duplicate_packets": sum(row["duplicate_packets"] for row in rows),
        "total_reordered_packets": sum(row["reordered_packets"] for row in rows),
        "total_replay_rejections": sum(row["replay_rejections"] for row in rows),
        "total_dropped_loss_packets": total_dropped_loss,
        "total_accepted_for_impairment_queue": total_impairment_samples,
        "empirical_loss_fraction_across_runs": (
            total_dropped_loss / (total_dropped_loss + total_impairment_samples)
            if total_dropped_loss + total_impairment_samples else None
        ),
        "interpretation": (
            "Observed outcomes for this exact same-host injected-network profile only; "
            "the interval is descriptive and does not establish LAN, radio, or flight reliability."
        ),
        "runs": rows,
    }
    if expected_profile["operator_loss"]:
        operator_results = [row["operator_loss"] for row in rows]
        landing_times = [
            float(item["seconds_after_operator_loss"])
            for item in operator_results
            if item["seconds_after_operator_loss"] is not None
        ]
        result["operator_loss_checks_all_passed"] = all(
            item["operator_process_killed"]
            and item["all_agents_were_airborne"]
            and item["all_agents_requested_mission_expiry_land"]
            and item["all_agents_landed_disarmed"]
            for item in operator_results
        )
        result["maximum_seconds_to_landed_disarmed_after_operator_loss"] = (
            max(landing_times) if len(landing_times) == len(rows) else None
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summaries", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    loaded = [(path, json.loads(path.read_text(encoding="utf-8"))) for path in args.summaries]
    result = summarize(loaded)
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
