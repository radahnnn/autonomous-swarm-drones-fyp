"""A dead flight-controller link must not spin or flood the log (UNIT)."""
import time

from sitl import flight_prep
from sitl.independent_agent_node import IndependentAgentNode


class DeadConn:
    """Mimics pymavlink on a closed TCP socket: recv_match returns None immediately."""

    def __init__(self):
        self.calls = 0

    def recv_match(self, **kwargs):
        self.calls += 1
        return None


def test_wait_ekf_ready_is_paced_and_gives_up_on_a_dead_link(monkeypatch, capsys):
    monkeypatch.setattr(flight_prep, "request_message_interval", lambda *a, **k: None)
    conn = DeadConn()
    started = time.time()
    ok = flight_prep.wait_ekf_ready(conn, "Node 1", wall_timeout=30.0, no_data_timeout=1.2)
    elapsed = time.time() - started
    assert ok is False
    assert elapsed < 3.0                                  # gave up on the link, not the 30 s timeout
    assert conn.calls <= 8                                # paced (was tens of thousands per second)
    assert "link lost" in capsys.readouterr().out


def test_wait_ekf_ready_still_succeeds_on_a_live_link(monkeypatch):
    monkeypatch.setattr(flight_prep, "request_message_interval", lambda *a, **k: None)

    class Msg:
        def __init__(self, kind, **fields):
            self._kind = kind
            self.__dict__.update(fields)

        def get_type(self):
            return self._kind

    required = flight_prep._EKF_REQUIRED

    class LiveConn:
        def __init__(self):
            self.queue = [Msg("GPS_RAW_INT", fix_type=6)] + [Msg("EKF_STATUS_REPORT", flags=required)] * 5

        def recv_match(self, **kwargs):
            return self.queue.pop(0) if self.queue else None

    assert flight_prep.wait_ekf_ready(LiveConn(), "Node 1", wall_timeout=10.0, stable=3) is True


def test_agent_declares_the_link_dead_only_after_the_silence_limit():
    node = object.__new__(IndependentAgentNode)
    node.command_in_progress = False
    node.last_heartbeat_at = 1000.0
    assert node._link_dead(1000.0 + IndependentAgentNode.LINK_DEAD_EXIT_S - 1) is False
    assert node._link_dead(1000.0 + IndependentAgentNode.LINK_DEAD_EXIT_S + 1) is True
    node.command_in_progress = True                       # a long arm/takeoff helper is running
    assert node._link_dead(1000.0 + 500) is False
