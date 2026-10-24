"""Collect node status after the independent operator console is terminated."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import time


def _terminate_exact_pids(pids: list[int], wait_s: float = 4.0) -> None:
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except ProcessLookupError:
                pass
        if not alive:
            return
        time.sleep(0.1)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--land-timeout", type=float, default=45.0)
    args = parser.parse_args()

    manifest_path = args.run_dir / "processes.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    sockets = []
    for drone_id in range(args.count):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 22000 + drone_id))
        sock.setblocking(False)
        sockets.append(sock)
    out_path = args.run_dir / "operator_loss_status.jsonl"
    latest = {}
    landed_after_airborne = set()
    started = time.monotonic()
    with out_path.open("a", encoding="utf-8") as out:
        while time.monotonic() - started < args.timeout:
            for drone_id, sock in enumerate(sockets):
                while True:
                    try:
                        raw, _ = sock.recvfrom(8192)
                    except BlockingIOError:
                        break
                    try:
                        status = json.loads(raw.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if status.get("type") != "status" or status.get("drone_id") != drone_id:
                        continue
                    latest[drone_id] = status
                    if status.get("airborne"):
                        landed_after_airborne.discard(drone_id)
                    elif drone_id in latest and status.get("command", "").startswith("landing:"):
                        landed_after_airborne.add(drone_id)
                    record = {"utc": datetime.now(timezone.utc).isoformat(),
                              "kind": "node_status_after_console_loss", **status}
                    out.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
                    out.flush()
            if len(latest) == args.count and len(landed_after_airborne) == args.count:
                break
            time.sleep(0.05)

        success = len(latest) == args.count and len(landed_after_airborne) == args.count
        if not success:
            # If the mission lease does not lead to LAND before timeout, send a direct
            # emergency LAND command to each node and continue observing.
            command = {"version": 1, "type": "command", "sequence": 999999,
                       "action": "land", "target_alt_m": 5.0}
            payload = json.dumps(command, separators=(",", ":")).encode("utf-8")
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                for drone_id in range(args.count):
                    sender.sendto(payload, ("127.0.0.1", 21000 + drone_id))
            land_started = time.monotonic()
            while time.monotonic() - land_started < args.land_timeout:
                for drone_id, sock in enumerate(sockets):
                    while True:
                        try:
                            raw, _ = sock.recvfrom(8192)
                        except BlockingIOError:
                            break
                        try:
                            status = json.loads(raw.decode("utf-8"))
                        except (UnicodeDecodeError, json.JSONDecodeError):
                            continue
                        if status.get("type") != "status":
                            continue
                        latest[drone_id] = status
                        if not status.get("airborne"):
                            landed_after_airborne.add(drone_id)
                        out.write(json.dumps({"utc": datetime.now(timezone.utc).isoformat(),
                                              "kind": "observer_land_fallback", **status},
                                             allow_nan=False, separators=(",", ":")) + "\n")
                        out.flush()
                if len(landed_after_airborne) == args.count:
                    success = True
                    break
                time.sleep(0.05)
        out.write(json.dumps({"utc": datetime.now(timezone.utc).isoformat(),
                              "kind": "observer_result", "all_landed": success,
                              "agents_seen": sorted(latest),
                              "landed_agents": sorted(landed_after_airborne)},
                             separators=(",", ":")) + "\n")
        out.flush()

    for sock in sockets:
        sock.close()
    if success:
        _terminate_exact_pids(manifest.get("agent_pids", []))
        _terminate_exact_pids(manifest.get("sitl_pids", []))
    print(json.dumps({"all_landed": success, "status_file": str(out_path),
                      "agents_seen": sorted(latest),
                      "landed_agents": sorted(landed_after_airborne)}))
    if not success:
        raise SystemExit("Observer timeout; processes were kept alive for manual recovery.")


if __name__ == "__main__":
    main()
