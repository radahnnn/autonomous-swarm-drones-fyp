"""Development operator-to-agent UDP intent transport for independent SITL nodes."""

import hashlib
import hmac
import json
import socket


def encode_message(message: dict, auth_key: bytes | None = None) -> bytes:
    body = json.dumps(message, allow_nan=False, separators=(",", ":"),
                      sort_keys=True).encode("utf-8")
    if auth_key is None:
        return body
    signature = hmac.new(auth_key, body, hashlib.sha256).hexdigest()
    return json.dumps({"body": body.decode("utf-8"), "hmac_sha256": signature},
                      separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_message(raw: bytes, auth_key: bytes | None = None) -> dict | None:
    try:
        value = json.loads(raw.decode("utf-8"))
        if auth_key is not None:
            if (not isinstance(value, dict) or not isinstance(value.get("body"), str) or
                    not isinstance(value.get("hmac_sha256"), str)):
                return None
            body = value["body"].encode("utf-8")
            expected = hmac.new(auth_key, body, hashlib.sha256).hexdigest()
            if not hmac.compare_digest(value["hmac_sha256"], expected):
                return None
            value = json.loads(body.decode("utf-8"))
        elif isinstance(value, dict) and "hmac_sha256" in value:
            return None
        return value if isinstance(value, dict) else None
    except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
        return None


def send_to_nodes(message: dict, count: int | None = None, port_base: int = 21000,
                  host: str = "127.0.0.1", endpoints=None,
                  auth_key: bytes | None = None) -> int:
    payload = encode_message(message, auth_key)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        targets = (list(endpoints.values()) if endpoints is not None else
                   [(host, int(port_base) + drone_id) for drone_id in range(int(count))])
        for target in targets:
            sock.sendto(payload, (str(target[0]), int(target[1])))
    return len(targets)


class NodeOperatorSocket:
    """Receive operator commands addressed directly to this agent process."""

    def __init__(self, drone_id: int, port_base: int = 21000,
                 host: str = "127.0.0.1", bind_host: str | None = None,
                 port: int | None = None, auth_key: bytes | None = None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.auth_key = auth_key
        self.socket.bind((bind_host or host,
                          int(port) if port is not None else int(port_base) + int(drone_id)))
        self.socket.setblocking(False)

    def poll(self) -> list[dict]:
        messages = []
        while True:
            try:
                raw, _address = self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                value = decode_message(raw, self.auth_key)
                if value is not None:
                    messages.append(value)
            except OSError:
                break
        return messages

    def close(self) -> None:
        self.socket.close()
