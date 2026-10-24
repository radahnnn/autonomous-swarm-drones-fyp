import threading
import time
from types import SimpleNamespace

import numpy as np

from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.operator_dashboard import PAGE, dashboard_state


def make_session(*, airborne=False):
    now = time.time()
    return SimpleNamespace(
        lock=threading.Lock(),
        latest={
            0: {
                "operator_received_at": now,
                "safety_state": "nominal",
                "airborne": airborne,
                "armed": airborne,
                "position": [1.0, 2.0, -5.0 if airborne else 0.0],
                "elected_leader_id": 0,
                "active_member_ids": [0, 1],
                "peer_ids": [1],
                "peer_live_ids": [1],
            },
            1: {
                "operator_received_at": now,
                "safety_state": "nominal",
                "airborne": airborne,
                "armed": airborne,
                "position": [1.0, -2.0, -5.0 if airborne else 0.0],
                "elected_leader_id": 0,
                "active_member_ids": [0, 1],
                "peer_ids": [0],
                "peer_live_ids": [0],
            },
        },
        count=2,
        control_mode="decentralized",
        formation="v",
        goal_ne=np.array([5.0, 0.0]),
        target_alt_m=5.0,
        intent_enabled=True,
        args=SimpleNamespace(obstacle=["8,0,2"], dashboard_port=8765),
    )


def test_dashboard_state_summarizes_agents_and_obstacles():
    state = dashboard_state(make_session(airborne=True))
    assert state["control_mode"] == "decentralized"
    assert state["healthy"] == 2
    assert state["airborne"] == 2
    assert state["leader"] == 1
    assert state["obstacles"] == [[8.0, 0.0, 2.0]]
    assert state["agents"][1]["peers"] == [1]


def test_dashboard_exposes_all_modes_and_locks_switching_in_flight():
    assert all(f'data-mode="{mode}"' in PAGE
               for mode in ("centralized", "decentralized", "hybrid"))
    assert "Land all drones before changing architecture" in PAGE
    assert "b.disabled=state.airborne>0&&!active" in PAGE


def test_mode_relaunch_commands_match_controller_topology():
    console = object.__new__(IndependentSwarmConsole)
    console.count = 10
    console.args = SimpleNamespace(dashboard_port=8765)

    centralized = console.mode_relaunch_command("centralized")
    decentralized = console.mode_relaunch_command("decentralized")
    hybrid = console.mode_relaunch_command("hybrid")

    assert centralized.endswith("--control-mode centralized")
    assert "--agent-runtime" not in centralized
    assert "--control-mode decentralized --agent-runtime independent" in decentralized
    assert "--control-mode hybrid --agent-runtime independent" in hybrid
    assert decentralized.endswith("--dashboard-port 8765")
