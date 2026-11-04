import json
import numpy as np

from sitl.run_grade import grade_run


def _run(tmp_path, result="PASS", driver="unexpectedly_dead=[]\ndriver done; failed=False\n", statuses=None):
    ev = tmp_path / "evidence" / "x"
    ev.mkdir(parents=True)
    (ev / "run_report.json").write_text(json.dumps({"result": result, "drones": 2, "min_gap_m": 4.0,
                                                    "true_min_gap_m": 3.9}))
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


def test_simstate_has_no_altitude_field_so_truth_must_not_depend_on_it():
    # Live 2026-11-04: reading SIMSTATE.alt raised and silently dropped all true positions (coverage 0 %).
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
    assert "alt" not in mavlink.MAVLink_simstate_message.fieldnames
    import inspect

    from sitl import independent_agent_node
    assert "msg.alt" not in inspect.getsource(independent_agent_node)
