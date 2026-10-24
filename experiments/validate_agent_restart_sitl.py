"""Restart one independent agent during a five-drone SITL mission and record recovery."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import pty
import re
import subprocess
import sys
import time

from experiments.validate_independent_operator_loss import ConsolePty, terminate_pids


def read_events(path: Path) -> list[dict]:
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def latest_statuses(events: list[dict]) -> dict[int, dict]:
    latest = {}
    for event in events:
        if event.get("kind") == "agent_status":
            latest[event["drone_id"]] = event
    return latest


def landed(status: dict) -> bool:
    position = status.get("position")
    return bool(
        status.get("armed") is False and status.get("airborne") is False
        and isinstance(position, list)
        and len(position) == 3 and float(position[2]) > -0.5
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drones", type=int, default=5)
    parser.add_argument("--restart-id", type=int, default=3,
                        help="one-based drone ID to restart while airborne")
    parser.add_argument("--seed", type=int, default=103)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    if args.drones != 5:
        parser.error("this validation trial is defined for exactly five drones")
    if not 1 <= args.restart_id <= args.drones:
        parser.error("--restart-id must be a valid one-based drone ID")

    root = Path(__file__).resolve().parents[1]
    command = [sys.executable, "-u", "-m", "sitl.swarm_console",
               "--drones", "5", "--standalone", "--control-mode", "decentralized",
               "--agent-runtime", "independent", "--peer-seed", str(args.seed)]
    master, slave = pty.openpty()
    process = subprocess.Popen(command, cwd=root, stdin=slave, stdout=slave, stderr=slave,
                               start_new_session=True,
                               env={**os.environ, "PYTHONPATH": str(root)})
    os.close(slave)
    transcript = root / "runs" / "independent" / f"restart-harness-{process.pid}.log"
    terminal = ConsolePty(process, master, transcript)
    run_dir = None
    manifest = None
    result = {"kind": "agent_restart_trial", "seed": args.seed,
              "restart_drone_id": args.restart_id, "started_utc": datetime.now(
                  timezone.utc).isoformat(), "passed": False}

    def wait_events(predicate, timeout: float, reason: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = read_events(run_dir / "events.jsonl")
            if predicate(current):
                return current
            if process.poll() is not None:
                raise RuntimeError(f"operator exited before {reason}")
            terminal.read(0.1)
            time.sleep(0.05)
        raise TimeoutError(f"timed out waiting for {reason}")

    try:
        terminal.wait_for(lambda value: ">> Independent-node evidence:" in value,
                          30.0, "run evidence directory")
        match = re.search(r">> Independent-node evidence: ([^\r\n]+)", terminal.text)
        if not match:
            raise RuntimeError("console did not print the evidence directory")
        run_dir = Path(match.group(1).strip())
        terminal.transcript.close()
        terminal.transcript = (run_dir / "console.log").open("a", encoding="utf-8")
        manifest_path = run_dir / "processes.json"
        terminal.wait_for(lambda value: "Independent Swarm > " in value, 420.0,
                          "all agents becoming EKF-ready")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        terminal.send("launch")

        def all_airborne(events):
            states = latest_statuses(events)
            return len(states) == args.drones and all(
                states[i].get("armed") is True and states[i].get("airborne") is True
                and states[i].get("position", [0, 0, 0])[2] < -3.5
                for i in range(args.drones)
            )

        events = wait_events(all_airborne, 180.0, "all five vehicles reaching flight altitude")
        terminal.send("w 2")
        time.sleep(3.0)
        terminal.send(f"restart {args.restart_id}")
        events = wait_events(
            lambda rows: any(row.get("kind") == "agent_process_restarted"
                             and row.get("drone_id") == args.restart_id - 1 for row in rows),
            10.0, "agent restart event",
        )
        result["restart_event"] = next(
            row for row in reversed(events)
            if row.get("kind") == "agent_process_restarted"
            and row.get("drone_id") == args.restart_id - 1
        )

        restart_id = args.restart_id - 1

        def full_fleet_landed(rows):
            states = latest_statuses(rows)
            state = states.get(restart_id, {})
            return (
                state.get("recovery_state") == "landed_after_process_restart"
                and landed(state)
                and all(landed(states.get(index, {})) for index in range(args.drones))
            )

        events = wait_events(full_fleet_landed, args.timeout,
                             "all five vehicles landing and disarming after restart")
        states = latest_statuses(events)
        restart_state = states[restart_id]
        peer_landers = sorted(
            index + 1 for index, state in states.items()
            if index != restart_id and state.get("safety_state") == "peer_landing_failsafe"
        )
        all_landed = all(landed(states.get(index, {})) for index in range(args.drones))
        result.update({
            "restart_agent_landed": True,
            "restart_recovery_state": restart_state.get("recovery_state"),
            "restart_safety_state": restart_state.get("safety_state"),
            "peer_landing_failsafe_ids": peer_landers,
            "all_five_landed_after_restart": all_landed,
            "final_statuses": {str(index + 1): status
                               for index, status in sorted(states.items())},
            "passed": bool(all_landed and peer_landers),
        })
        result["completed_utc"] = datetime.now(timezone.utc).isoformat()
        (run_dir / "agent_restart_summary.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"passed": result["passed"], "run_dir": str(run_dir),
                          "summary": str(run_dir / "agent_restart_summary.json"),
                          "peer_landing_failsafe_ids": peer_landers,
                          "all_five_landed": all_landed}))
        if not result["passed"]:
            raise RuntimeError("restart trial did not prove fleet-wide landing after peer propagation")
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["completed_utc"] = datetime.now(timezone.utc).isoformat()
        if run_dir is not None:
            (run_dir / "agent_restart_summary.json").write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8")
        raise
    finally:
        if process.poll() is None:
            try:
                os.write(master, b"land\n")
                process.wait(timeout=90.0)
            except (OSError, subprocess.TimeoutExpired):
                process.kill()
                process.wait(timeout=5.0)
        if manifest is not None:
            terminate_pids(manifest.get("agent_pids", []))
            terminate_pids(manifest.get("sitl_pids", []))
        terminal.close()


if __name__ == "__main__":
    main()
