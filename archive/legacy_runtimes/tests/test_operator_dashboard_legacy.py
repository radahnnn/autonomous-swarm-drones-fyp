"""Retired 2026-11-04 with the legacy runtimes (user decision, item 5); moved unchanged from tests/test_operator_dashboard.py. Not collected."""


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
