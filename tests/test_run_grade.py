import json
import numpy as np

from sitl.run_grade import grade_run


def _run(tmp_path, result="PASS", driver="unexpectedly_dead=[]\ndriver done; failed=False\n", statuses=None,
         gate_metric="horizontal"):
    ev = tmp_path / "evidence" / "x"
    ev.mkdir(parents=True)
    (ev / "run_report.json").write_text(json.dumps({"result": result, "drones": 2, "min_gap_m": 4.0,
                                                    "true_min_gap_m": 3.9, "gate_metric": gate_metric}))
    if driver is not None:
        (tmp_path / "driver.log").write_text(driver)
    if statuses is None:
        statuses = []
        for d in (0, 1):
            statuses += [dict(drone_id=d, airborne=True, armed=True, command="mission:1", true_position_ne=[0, d]),
                         dict(drone_id=d, airborne=True, armed=True, command="landing:operator", true_position_ne=[0, d]),
                         dict(drone_id=d, airborne=False, armed=False, command="landed")]
    (ev / "events.jsonl").write_text("".join(json.dumps({"kind": "agent_status", **s}) + "\n" for s in statuses))
    return tmp_path


def test_clean_mission_passes(tmp_path):
    assert grade_run(_run(tmp_path))["verdict"] == "PASS"


def test_a_drone_landed_by_the_guard_is_not_a_pass(tmp_path):
    statuses = [dict(drone_id=0, airborne=True, armed=True, command="mission:1", true_position_ne=[0, 0]),
                dict(drone_id=0, airborne=True, armed=True, command="landing:position_estimate_or_flyaway",
                     true_position_ne=[0, 0]),
                dict(drone_id=0, airborne=False, armed=False, command="landed"),
                dict(drone_id=1, airborne=True, armed=True, command="landing:operator", true_position_ne=[0, 1]),
                dict(drone_id=1, airborne=False, armed=False, command="landed")]
    g = grade_run(_run(tmp_path, statuses=statuses))
    assert g["verdict"] == "INCOMPLETE" and any("D1 landed by position_estimate_or_flyaway" in r for r in g["reasons"])


def test_a_late_reason_after_the_operator_land_does_not_count(tmp_path):
    # Gazebo re-check 2026-11-04: telemetry_timeout appeared after D2 had landed and disarmed at shutdown.
    statuses = []
    for d in (0, 1):
        statuses += [dict(drone_id=d, airborne=True, armed=True, command="landing:operator", true_position_ne=[0, d]),
                     dict(drone_id=d, airborne=False, armed=False, command="landed"),
                     dict(drone_id=d, airborne=False, armed=False, command="landing:telemetry_timeout"),
                     dict(drone_id=d, airborne=False, armed=False, command="landed")]
    assert grade_run(_run(tmp_path, statuses=statuses))["verdict"] == "PASS"


def test_missed_milestones_still_airborne_and_missing_truth_are_not_a_pass(tmp_path):
    statuses = [dict(drone_id=0, airborne=True, armed=True, command="mission:1", true_position_ne=None),
                dict(drone_id=1, airborne=True, armed=True, command="mission:1", true_position_ne=[0, 1])]
    g = grade_run(_run(tmp_path, driver="expect 'V restored' (line 12): TIMEOUT\nunexpectedly_dead=[]\n",
                       statuses=statuses))
    text = " ".join(g["reasons"])
    assert g["verdict"] == "INCOMPLETE"
    assert "V restored" in text and "not landed and disarmed" in text and "coverage" in text


def test_live_fail_stays_fail(tmp_path):
    assert grade_run(_run(tmp_path, result="FAIL"))["verdict"] == "FAIL"


def test_true_separation_uses_the_gate_metric_and_keeps_the_horizontal_minimum():
    from sitl.run_metrics import RunMetrics
    m = RunMetrics(2, gate_metric="ellipsoid", vertical_radius_m=2.0)
    m._track_truth({0: np.array([0.0, 0.0, 10.0]), 1: np.array([1.0, 0.0, 12.0])}, 0.0)   # 1 m apart, 2 m layers
    assert m.true_min_gap_m >= 2.5 and m.true_violations == 0 and m.true_min_horizontal_m == 1.0
    flat = RunMetrics(2, gate_metric="ellipsoid", vertical_radius_m=2.0)
    flat._track_truth({0: np.array([0.0, 0.0]), 1: np.array([1.0, 0.0])}, 0.0)            # no true altitude: horizontal
    assert flat.true_min_gap_m == 1.0 and flat.true_violations == 1


def test_simstate_and_sim_state_truth_parsing():
    # Live 2026-11-04: SIMSTATE has no alt field; SIM_STATE (ID 108) supplies 3-D altitude without breaking SIMSTATE.
    import pytest
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
    from sitl.common_frame import CommonCoordinateFrame
    from sitl.independent_agent_node import IndependentAgentNode

    assert "alt" not in mavlink.MAVLink_simstate_message.fieldnames
    assert "alt" in mavlink.MAVLink_sim_state_message.fieldnames

    class FakeMsg:
        def __init__(self, kind, **fields):
            self._kind = kind
            for k, v in fields.items():
                setattr(self, k, v)

        def get_type(self):
            return self._kind

        def get_srcSystem(self):
            return 1

        def get_srcComponent(self):
            return 1

    class FakeConn:
        def __init__(self, msgs):
            self._msgs = list(msgs)

        def recv_match(self, blocking=False):
            return self._msgs.pop(0) if self._msgs else None

    node = object.__new__(IndependentAgentNode)
    node.mavlink_system_id = 1
    node.frame = CommonCoordinateFrame(-35.363261, 149.165230, 584.0)
    node.armed = False
    node.airborne = False
    node.landing = False
    node.position = None
    node.position_at = 0.0
    node.local_position = None
    node.global_origin = None
    node.true_position_ne = None
    node.true_altitude_m = None
    node.true_position_at = 0.0
    node.true_altitude_at = None

    # SIMSTATE alone -> horizontal truth only, true_altitude_m stays None (no AttributeError).
    node.conn = FakeConn([FakeMsg("SIMSTATE", lat=int(-35.363261 * 1e7), lng=int(149.165230 * 1e7))])
    node._read_vehicle()
    assert node.true_position_ne is not None and node.true_altitude_m is None

    # SIM_STATE -> populates true_altitude_m relative to datum_alt, and subsequent SIMSTATE does not clobber it.
    node.conn = FakeConn([
        FakeMsg("SIM_STATE", lat_int=int(-35.363261 * 1e7), lon_int=int(149.165230 * 1e7), alt=591.2),
        FakeMsg("SIMSTATE", lat=int(-35.363261 * 1e7), lng=int(149.165230 * 1e7)),
    ])
    node._read_vehicle()
    assert node.true_position_ne is not None
    assert node.true_altitude_m == pytest.approx(7.2, abs=1e-4)


def test_grade_run_requires_3d_altitude_truth_for_layered_gates(tmp_path):
    g_missing = grade_run(_run(tmp_path / "missing_alt", gate_metric="ellipsoid"))
    assert g_missing["verdict"] == "INCOMPLETE"
    assert any("true-3d-altitude coverage" in r for r in g_missing["reasons"])
    assert g_missing["true_3d_coverage_min"] == 0.0

    statuses_3d = []
    for d in (0, 1):
        statuses_3d += [
            dict(drone_id=d, airborne=True, armed=True, command="mission:1",
                 true_position_ne=[0, d], true_altitude_m=5.0 + 2.0 * d),
            dict(drone_id=d, airborne=True, armed=True, command="landing:operator",
                 true_position_ne=[0, d], true_altitude_m=5.0 + 2.0 * d),
            dict(drone_id=d, airborne=False, armed=False, command="landed"),
        ]
    g_full = grade_run(_run(tmp_path / "full_3d", statuses=statuses_3d, gate_metric="ellipsoid"))
    assert g_full["verdict"] == "PASS"
    assert g_full["true_3d_coverage_min"] == 1.0
