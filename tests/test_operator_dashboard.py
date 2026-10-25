import threading
import time
from types import SimpleNamespace

import numpy as np

from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.operator_dashboard import PAGE, dashboard_state
from sitl.mode_supervisor import ModeSupervisor


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
    assert "Land and disarm all drones before changing architecture" in PAGE
    assert "b.disabled=!!state.transitioning||state.airborne>0||state.armed>0||!state.healthy" in PAGE
    assert "Formation shape" in PAGE
    assert "Kill agent" in PAGE
    assert "Revive / restart" in PAGE


def test_dashboard_reports_grounded_shape_and_agent_controls():
    session = make_session()
    session.args.adaptive_swarm = True
    state = dashboard_state(session)
    assert state["formation_change_enabled"] is True
    assert state["formation"] == "v"
    assert state["armed"] == 0
    assert state["formations"]
    assert state["agents"][0]["agent_action_enabled"] is True


def test_independent_formation_change_requires_all_agents_grounded():
    console = object.__new__(IndependentSwarmConsole)
    console.args = SimpleNamespace(adaptive_swarm=True)
    console.count = 2
    console.lock = threading.Lock()
    received = time.time()
    console.latest = {
        0: {"armed": False, "airborne": False, "operator_received_at": received},
        1: {"armed": False, "airborne": False, "operator_received_at": received},
    }
    console.formation = "v"
    console._publish_intent = lambda: None
    console.event = lambda *_args, **_kwargs: None
    console._set_formation("line")
    assert console.formation == "line"

    console.latest[1]["armed"] = True
    console._set_formation("ring")
    assert console.formation == "line"


def test_centralized_dashboard_state_uses_vehicle_telemetry():
    now = time.time()
    central = SimpleNamespace(
        n=2, control_mode="centralized", lock=threading.Lock(),
        last_pos=[np.array([1., 2., 0.]), np.array([1., -2., 0.])],
        last_pos_at=[now, now], is_airborne=[False, False],
        last_armed_state=[False, False], drone_health=[True, True],
        simulated_faults=[False, False], current_leader_idx=0,
        decentralized_guidance=None, current_formation_style="v",
        centroid_pos=np.array([3., 4., -5.]),
    )
    state = dashboard_state(central)
    assert state["control_mode"] == "centralized"
    assert state["formation_ready"] is True
    assert state["healthy"] == 2
    assert state["goal"] == [3., 4.]
    assert state["peer_fault_enabled"] is False


def test_supervisor_builds_correct_child_topologies():
    supervisor = object.__new__(ModeSupervisor)
    supervisor.count = 10
    supervisor.child_port = 8766
    supervisor.params = __import__("pathlib").Path("sitl/swarm_params.parm")
    central = supervisor._child_command("centralized")
    dec = supervisor._child_command("decentralized")
    hybrid = supervisor._child_command("hybrid")
    assert "--dashboard-port" in central and "--agent-runtime" not in central
    assert "--agent-runtime" in dec and "independent" in dec and "--adaptive-swarm" in dec
    assert "--control-mode" in hybrid and hybrid[hybrid.index("--control-mode") + 1] == "hybrid"


def test_dashboard_explains_persistent_grounded_mode_switch():
    assert "controller will restart" in PAGE
    assert "dashboard stays connected" in PAGE


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
