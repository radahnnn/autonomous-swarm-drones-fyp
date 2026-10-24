"""Repeat a five-drone direct-peer partition test against standalone ArduPilot SITL."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


class ConsoleProcess:
    def __init__(self, process, master, log_path):
        self.process = process
        self.master = master
        self.log = log_path.open("w", encoding="utf-8")
        self.buffer = ""

    def read(self, timeout=0.25):
        ready, _, _ = select.select([self.master], [], [], timeout)
        if not ready:
            return
        try:
            chunk = os.read(self.master, 65536).decode("utf-8", errors="replace")
        except OSError:
            return
        self.buffer += chunk
        self.log.write(chunk)
        self.log.flush()

    def send(self, value):
        os.write(self.master, (value + "\n").encode())

    def wait_until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read(0.25)
            if predicate():
                return
            if self.process.poll() is not None:
                raise RuntimeError(f"console exited before {label}; see console.log")
        raise TimeoutError(f"timed out waiting for {label}")

    def close(self):
        self.log.close()
        try:
            os.close(self.master)
        except OSError:
            pass


def read_events(path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def latest_statuses(events):
    latest = {}
    for event in events:
        if event.get("kind") == "agent_status":
            latest[event["drone_id"]] = event
    return latest


def run_seed(seed, loss, latency_ms, burst_loss=False, duplicate_rate=0.0,
             reorder_rate=0.0, reorder_delay_ms=150.0, roster_path=None,
             auth_key_path=None):
    command = [sys.executable, "-u", "-m", "sitl.swarm_console",
               "--drones", "5", "--standalone", "--control-mode", "decentralized",
               "--agent-runtime", "independent", "--peer-loss", str(loss),
               "--peer-latency-ms", str(latency_ms), "--peer-seed", str(seed)]
    command += ["--peer-duplicate-rate", str(duplicate_rate),
                "--peer-reorder-rate", str(reorder_rate),
                "--peer-reorder-delay-ms", str(reorder_delay_ms)]
    if burst_loss:
        command.append("--peer-burst-loss")
    if roster_path:
        command += ["--peer-roster", str(roster_path)]
    if auth_key_path:
        command += ["--peer-auth-key-file", str(auth_key_path)]
    master, slave = pty.openpty()
    process = subprocess.Popen(command, cwd=ROOT, stdin=slave, stdout=slave, stderr=slave,
                               start_new_session=True,
                               env={**os.environ, "PYTHONPATH": str(ROOT)})
    os.close(slave)
    console = ConsoleProcess(process, master, ROOT / "runs" / "independent" /
                             f"partition-harness-{process.pid}.log")
    run_dir = None
    try:
        console.wait_until(lambda: ">> Independent-node evidence:" in console.buffer,
                           30, "evidence directory")
        match = re.search(r">> Independent-node evidence: ([^\r\n]+)", console.buffer)
        if not match:
            raise RuntimeError("console did not report its evidence directory")
        run_dir = Path(match.group(1).strip())
        events_path = run_dir / "events.jsonl"

        def events():
            return read_events(events_path)

        console.wait_until(lambda: any(row.get("kind") == "nodes_ready" for row in events()),
                           400, "five EKF-ready agents")
        console.send("launch")
        console.wait_until(
            lambda: all(index in latest_statuses(events()) and
                        latest_statuses(events())[index].get("airborne") and
                        latest_statuses(events())[index].get("position", [0, 0, 0])[2] < -3.5
                        for index in range(5)),
            180, "all five vehicles airborne at altitude",
        )
        console.send("link-block 2 4")
        time.sleep(0.15)
        console.send("link-block 4 2")

        def endpoints_landed():
            states = latest_statuses(events())
            return all(index in states and states[index].get("safety_state") == "peer_link_failsafe"
                       and states[index].get("command") == "landed"
                       and not states[index].get("armed") for index in (1, 3))

        console.wait_until(endpoints_landed, 60, "both partition endpoints land autonomously")
        isolated_endpoint_landed_at = time.time()
        statuses_after = latest_statuses(events())
        others_continued = all(statuses_after[index].get("airborne") for index in (0, 2, 4))
        console.send("land")
        console.wait_until(
            lambda: any(row.get("kind") == "session_end" and row.get("landed")
                        for row in events()),
            90, "all five vehicles landed and disarmed",
        )
        session_end = next(row for row in reversed(events())
                           if row.get("kind") == "session_end")
        link_faults = [row for row in events() if row.get("kind") == "peer_link_fault"]
        final_statuses = latest_statuses(events())
        peer_metrics = [final_statuses[index].get("peer_metrics", {})
                        for index in range(5) if index in final_statuses]
        duplicated = sum(metrics.get("duplicated_packets", 0) for metrics in peer_metrics)
        reorder_delayed = sum(metrics.get("reorder_delayed_packets", 0) for metrics in peer_metrics)
        rejected_replay = sum(metrics.get("rejected_replay", 0) for metrics in peer_metrics)
        rejected_auth = sum(metrics.get("rejected_auth", 0) for metrics in peer_metrics)
        fault_started_at = min(
            datetime.fromisoformat(row["utc"]).timestamp() for row in link_faults
        )
        safety_started_at = {}
        for row in events():
            if (row.get("kind") == "agent_status" and
                    row.get("safety_state") == "peer_link_failsafe" and
                    row.get("drone_id") in (1, 3)):
                safety_started_at.setdefault(row["drone_id"], row["timestamp"])
        detection_delay_s = max(safety_started_at.values()) - fault_started_at
        result = {
            "seed": seed,
            "peer_loss": loss,
            "peer_latency_ms": latency_ms,
            "peer_burst_loss": burst_loss,
            "peer_duplicate_rate": duplicate_rate,
            "peer_reorder_rate": reorder_rate,
            "peer_reorder_delay_ms": reorder_delay_ms,
            "peer_roster": str(roster_path) if roster_path else None,
            "peer_auth_enabled": bool(auth_key_path),
            "duplicated_datagrams": duplicated,
            "reorder_delayed_datagrams": reorder_delayed,
            "rejected_replayed_datagrams": rejected_replay,
            "rejected_auth_datagrams": rejected_auth,
            "partition": [[1, 3], [3, 1]],
            "partition_started_unix": fault_started_at,
            "endpoints_landed_unix": isolated_endpoint_landed_at,
            "failsafe_detection_delay_s": round(detection_delay_s, 2),
            "endpoint_touchdown_after_partition_s": round(
                isolated_endpoint_landed_at - fault_started_at, 2,
            ),
            "both_partition_endpoints_landed_autonomously": True,
            "other_three_remained_airborne_until_operator_land": others_continued,
            "all_five_landed_at_session_end": bool(session_end.get("landed")),
            "fault_event_count": len(link_faults),
            "evidence_dir": str(run_dir),
        }
        (run_dir / "partition_validation.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8",
        )
        impairments_observed = (
            (duplicate_rate == 0 or duplicated > 0) and
            (reorder_rate == 0 or reorder_delayed > 0)
        )
        if not (others_continued and session_end.get("landed") and len(link_faults) == 2
                and impairments_observed and rejected_auth == 0):
            raise RuntimeError(f"partition validation acceptance failed: {result}")
        return result
    except Exception:
        if process.poll() is None:
            try:
                console.send("land")
                process.wait(timeout=55)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=70)
                except subprocess.TimeoutExpired:
                    raise RuntimeError(
                        f"SITL console {process.pid} did not stop after LAND/interrupt; "
                        "leave its simulator processes running for manual recovery"
                    )
        raise
    finally:
        console.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="67,68", help="comma-separated deterministic peer seeds")
    parser.add_argument("--peer-loss", type=float, default=0.02)
    parser.add_argument("--peer-latency-ms", type=float, default=25.0)
    parser.add_argument("--peer-burst-loss", action="store_true")
    parser.add_argument("--peer-duplicate-rate", type=float, default=0.0)
    parser.add_argument("--peer-reorder-rate", type=float, default=0.0)
    parser.add_argument("--peer-reorder-delay-ms", type=float, default=150.0)
    parser.add_argument("--peer-roster", default=None,
                        help="optional shared static peer UDP roster JSON")
    parser.add_argument("--peer-auth-key-file", default=None,
                        help="optional shared HMAC key file for peer messages")
    args = parser.parse_args()
    seeds = [int(value) for value in args.seeds.split(",") if value.strip()]
    results = []
    for seed in seeds:
        results.append(run_seed(seed, args.peer_loss, args.peer_latency_ms, args.peer_burst_loss,
                                args.peer_duplicate_rate, args.peer_reorder_rate,
                                args.peer_reorder_delay_ms, args.peer_roster,
                                args.peer_auth_key_file))
    output = ROOT / "runs" / "independent" / "peer_partition_trials.json"
    previous = json.loads(output.read_text(encoding="utf-8")) if output.exists() else {"trials": []}
    all_trials = list(previous.get("trials", []))
    for row in results:
        identity = (row["seed"], row["peer_loss"], row["peer_latency_ms"],
                    row["peer_burst_loss"], row["peer_duplicate_rate"],
                    row["peer_reorder_rate"], row["peer_reorder_delay_ms"],
                    row["peer_roster"], row["peer_auth_enabled"])
        all_trials = [existing for existing in all_trials if
                      (existing.get("seed"), existing.get("peer_loss"),
                       existing.get("peer_latency_ms"), existing.get("peer_burst_loss", False),
                       existing.get("peer_duplicate_rate", 0.0),
                       existing.get("peer_reorder_rate", 0.0),
                       existing.get("peer_reorder_delay_ms", 150.0),
                       existing.get("peer_roster"),
                       existing.get("peer_auth_enabled", False)) != identity]
        all_trials.append(row)
    summary = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario": "five independent agents, two-way D2-D4 peer partition, local LAND policy under seeded network impairments",
        "trials": all_trials,
        "passed": all(row["both_partition_endpoints_landed_autonomously"] and
                       row["other_three_remained_airborne_until_operator_land"] and
                       row["all_five_landed_at_session_end"] and
                       (row.get("peer_duplicate_rate", 0.0) == 0 or
                        row.get("duplicated_datagrams", 0) > 0) and
                       (row.get("peer_reorder_rate", 0.0) == 0 or
                        row.get("reorder_delayed_datagrams", 0) > 0) and
                       row.get("rejected_auth_datagrams", 0) == 0
                       for row in all_trials),
    }
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    if not summary["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
