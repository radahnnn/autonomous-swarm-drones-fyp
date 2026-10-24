"""Static peer-address roster for direct UDP agent-to-agent transport."""

import json
from pathlib import Path


ROSTER_VERSION = 1


def load_peer_roster(path: str | Path | None, count: int,
                     port_base: int = 20000) -> dict[int, dict]:
    """Load one static roster shared by every agent; default to localhost ports."""
    count = int(count)
    if count < 1:
        raise ValueError("peer roster must contain at least one agent")
    if path is None:
        return {
            index: {
                "bind_host": "127.0.0.1",
                "host": "127.0.0.1",
                "port": int(port_base) + index,
                "command_bind_host": "127.0.0.1",
                "command_host": "127.0.0.1",
                "command_port": 21000 + index,
                "status_host": "127.0.0.1",
                "status_port": 22000 + index,
            }
            for index in range(count)
        }

    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read peer roster {path}: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("version") != ROSTER_VERSION:
        raise ValueError(f"peer roster version must be {ROSTER_VERSION}")
    rows = payload.get("agents")
    if not isinstance(rows, list) or len(rows) != count:
        raise ValueError(f"peer roster must list exactly {count} agents")

    roster = {}
    peer_bindings = set()
    operator_bindings = set()
    status_ports = set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("each peer roster agent must be an object")
        try:
            agent_id = int(row["id"])
            bind_host = str(row["bind_host"]).strip()
            host = str(row["host"]).strip()
            port = int(row["port"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("roster agents require id, bind_host, host, and port") from exc
        if not 0 <= agent_id < count or agent_id in roster:
            raise ValueError("peer roster IDs must be unique and range from 0 to count-1")
        if not bind_host or not host or host == "0.0.0.0":
            raise ValueError("peer roster hosts must be set; advertised host cannot be 0.0.0.0")
        if not 1 <= port <= 65535:
            raise ValueError("peer roster UDP port must be from 1 to 65535")
        command_bind_host = str(row.get("command_bind_host", "127.0.0.1")).strip()
        command_host = str(row.get("command_host", "127.0.0.1")).strip()
        status_host = str(row.get("status_host", "127.0.0.1")).strip()
        try:
            command_port = int(row.get("command_port", 21000 + agent_id))
            status_port = int(row.get("status_port", 22000 + agent_id))
        except (TypeError, ValueError) as exc:
            raise ValueError("operator command/status ports must be UDP ports") from exc
        if not command_bind_host or not command_host or not status_host:
            raise ValueError("operator command/status hosts must not be empty")
        if not (1 <= command_port <= 65535 and 1 <= status_port <= 65535):
            raise ValueError("operator command/status ports must be from 1 to 65535")
        peer_binding = (host, bind_host, port)
        operator_binding = (command_host, command_bind_host, command_port)
        if peer_binding in peer_bindings:
            raise ValueError("peer bind address/port pairs must be unique per host")
        if operator_binding in operator_bindings:
            raise ValueError("operator command bind address/port pairs must be unique per host")
        if status_port in status_ports:
            raise ValueError("operator status ports must be unique")
        peer_bindings.add(peer_binding)
        operator_bindings.add(operator_binding)
        status_ports.add(status_port)
        roster[agent_id] = {
            "bind_host": bind_host, "host": host, "port": port,
            "command_bind_host": command_bind_host,
            "command_host": command_host, "command_port": command_port,
            "status_host": status_host, "status_port": status_port,
        }
    if set(roster) != set(range(count)):
        raise ValueError("peer roster must include every ID from 0 to count-1 exactly once")
    return roster


def load_auth_key(path: str | Path | None) -> bytes | None:
    """Read a shared HMAC key; an absent path deliberately selects development-only mode."""
    if path is None:
        return None
    try:
        # Treat the key as binary. Stripping whitespace corrupts perfectly valid
        # random keys when their first or last byte happens to be ASCII whitespace.
        key = Path(path).read_bytes()
    except OSError as exc:
        raise ValueError(f"cannot read peer authentication key {path}: {exc}") from exc
    if len(key) < 32:
        raise ValueError("peer authentication key must contain at least 32 bytes")
    return key
