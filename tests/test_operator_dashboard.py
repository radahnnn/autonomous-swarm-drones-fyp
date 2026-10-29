import json
import socket
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

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
    assert state["minimum_separation_m"] == 4.0


def test_dashboard_exposes_all_modes_and_locks_switching_in_flight():
    assert all(f'data-mode="{mode}"' in PAGE
               for mode in ("centralized", "decentralized", "hybrid"))
    assert "Land and disarm all drones before changing architecture" in PAGE
    assert "b.disabled=!!state.transitioning||state.airborne>0||state.armed>0||!state.healthy" in PAGE
    assert "Formation shape" in PAGE
    assert "Kill agent" in PAGE
    assert "Revive / restart" in PAGE


def test_dashboard_routes_preflight_to_controller():
    session = SimpleNamespace(count=2, last_command_result=None)
    def handle(command):
        assert command == "preflight"
        session.last_command_result = {
            "accepted": False, "message": "Network preflight blocked launch.",
            "preflight": {"passed": False, "minimum_separation_m": 1.3},
        }
    session.handle = handle
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    from sitl.operator_dashboard import start_dashboard
    server = start_dashboard(session, port)
    try:
        request = Request(
            f"http://127.0.0.1:{port}/command",
            data=json.dumps({"command": "preflight"}).encode(),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        try:
            urlopen(request, timeout=2)
            assert False, "failed preflight should return a conflict response"
        except HTTPError as response:
            payload = json.loads(response.read())
            assert response.code == 409
            assert payload["preflight"]["minimum_separation_m"] == 1.3
    finally:
        server.shutdown()
        server.server_close()


def test_dashboard_reports_grounded_shape_and_agent_controls():
    session = make_session()
    session.args.adaptive_swarm = True
    state = dashboard_state(session)
    assert state["formation_change_enabled"] is True
    assert state["formation"] == "v"
    assert state["armed"] == 0
    assert state["formations"]
    assert state["agents"][0]["agent_action_enabled"] is True
    assert state["minimum_separation_m"] == 4.0
    assert "Check safety" in PAGE
    assert "path clearance" in PAGE


def test_independent_formation_change_requires_all_agents_grounded():
    console = object.__new__(IndependentSwarmConsole)
    console.args = SimpleNamespace(adaptive_swarm=True)
    console.count = 2
    console.lock = threading.Lock()
    received = time.time()
    console.latest = {
        0: {"armed": False, "airborne": False, "operator_received_at": received,
            "position_age_s": 0.0, "position": [0.0, -3.0, 0.0], "safety_state": "nominal"},
        1: {"armed": False, "airborne": False, "operator_received_at": received,
            "position_age_s": 0.0, "position": [0.0, 3.0, 0.0], "safety_state": "nominal"},
    }
    console.formation = "v"
    console.formation_transition_deadline = None
    console.formation_transition_started_at = None
    console.formation_slots = None
    console.formation_start_slots = None
    console.formation_transition_s = 0.0
    console.formation_resume_goal = None
    console.goal_ne = np.zeros(2)
    console._publish_intent = lambda: None
    console.event = lambda *_args, **_kwargs: None
    console._set_formation("line")
    assert console.formation == "line"

    console.latest[1]["armed"] = True
    console.latest[1]["airborne"] = True
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
