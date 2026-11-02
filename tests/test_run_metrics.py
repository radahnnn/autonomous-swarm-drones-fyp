"""Live metrics and the run report (UNIT)."""
import json

import pytest

from sitl.run_metrics import RunMetrics


def status(x, y, z=-5.0, *, now, airborne=True, age=0.2, target=None, mode="hybrid"):
    return {"position": [x, y, z], "airborne": airborne, "operator_received_at": now - age,
            "position_target_ne": target, "control_mode": mode, "coordinator_connected": True}


def feed(metrics, t, positions, **kw):
    metrics.update(t, {i: status(*p, now=t, **kw) for i, p in enumerate(positions)})


def test_idle_and_grounded_swarms_produce_no_gap():
    m = RunMetrics(3)
    feed(m, 0.0, [(0, 0), (1, 0), (2, 0)], airborne=False)       # grounded drones may be close
    snap = m.snapshot()
    assert snap["state"] == "n/a" and snap["gap_m"] is None and m.report()["result"] == "NO DATA"


def test_gap_margin_pair_and_ok_state():
    m = RunMetrics(3)
    feed(m, 0.0, [(0, 0), (3.0, 0), (9.0, 0)])
    snap = m.snapshot()
    assert snap["gap_m"] == 3.0 and snap["margin_m"] == 0.5 and snap["pair"] == (1, 2)
    assert snap["state"] == "ok" and m.report(1.0)["result"] == "PASS"


def test_warning_then_violation_episodes_are_counted_once_each():
    m = RunMetrics(2)
    series = [(0.0, 4.0), (0.6, 2.6), (1.2, 2.55), (1.8, 4.0), (2.4, 2.4), (3.0, 2.3), (3.6, 4.0),
              (4.2, 2.2), (4.8, 4.0)]
    for t, gap in series:
        feed(m, t, [(0, 0), (gap, 0)])
    assert m.warnings == 1                       # 2.6 and 2.55 are one continuous warning episode
    assert m.violations == 2                     # 2.4 -> 2.3 is one episode, 2.2 is another
    assert m.violation_time_s == pytest.approx(0.6)   # only the 2.4 -> 2.3 interval spans two samples
    report = m.report(5.0)
    assert report["result"] == "FAIL" and report["min_gap_m"] == 2.2
    assert report["violation_episodes"] == 2 and report["min_gap_at_s"] == 4.2


def test_3d_gap_is_reported_separately_and_can_be_the_gate():
    m = RunMetrics(2)
    feed(m, 0.0, [(0, 0, -5.0), (1.0, 0, -8.0)])                  # 1 m apart horizontally, 3 m vertically
    assert m.snapshot()["gap_m"] == 1.0 and m.snapshot()["gap_3d_m"] == pytest.approx(3.16, abs=1e-2)
    assert m.snapshot()["state"] == "violation"
    m3 = RunMetrics(2, gate_metric="3d")
    feed(m3, 0.0, [(0, 0, -5.0), (1.0, 0, -8.0)])
    assert m3.snapshot()["state"] == "ok"
    with pytest.raises(ValueError):
        RunMetrics(2, gate_metric="vertical")


def test_stale_statuses_and_garbage_are_ignored():
    m = RunMetrics(3)
    now = 10.0
    m.update(now, {
        0: status(0, 0, now=now), 1: status(1.0, 0, now=now, age=9.0),           # stale: ignored
        2: {"position": ["x", 0, 0], "airborne": True, "operator_received_at": now},
    })
    assert m.snapshot()["gap_m"] is None
    assert m.snapshot()["drones"][1]["link_age_s"] == 9.0


def test_tracking_error_and_link_age_are_collected_per_drone():
    m = RunMetrics(2)
    feed(m, 0.0, [(0, 0), (6, 0)], target=[0.3, 0.4])
    snap = m.snapshot()
    assert snap["drones"][0]["tracking_error_m"] == 0.5 and snap["drones"][0]["mode"] == "hybrid"
    assert m.report(1.0)["mean_tracking_error_m"] is not None and snap["max_link_age_s"] == 0.2


def test_sampling_period_limits_the_update_rate_and_history_is_bounded():
    m = RunMetrics(2, sample_period_s=0.5, history_s=5.0)
    for k in range(200):
        feed(m, k * 0.1, [(0, 0), (4, 0)])
    assert m.samples <= 41 and len(m.history) <= 10
    assert len(m.snapshot(max_points=5)["history"]) <= 5


def test_events_are_counted_and_the_report_is_written(tmp_path):
    m = RunMetrics(2)
    feed(m, 0.0, [(0, 0), (4, 0)])
    m.note_event("mode_switch_complete", 1.0, mode="decentralized")
    m.note_event("mode_switch_complete", 2.0, mode="hybrid")
    json_path, md_path = m.write_report(tmp_path / "out", ended_at=3.0)
    data = json.loads(json_path.read_text())
    assert data["counters"] == {"mode_switch_complete": 2} and data["result"] == "PASS"
    text = md_path.read_text()
    assert "Run report — PASS" in text and "sampled" in text and "mode_switch_complete" in text


def test_ground_contact_while_armed_fails_the_run_but_landing_does_not():
    from sitl.run_metrics import RunMetrics

    def st(t, alt, mode=4, north=0.0):
        return {"armed": True, "airborne": alt > 1.0, "vehicle_mode": mode, "operator_received_at": t,
                "position": [north, 0.0, -alt]}

    m = RunMetrics(2, sample_period_s=0.001)
    for t, alt in enumerate([5.0, 5.0, 0.2, 3.0, 0.1]):
        m.update(float(t), {0: st(float(t), alt), 1: st(float(t), 5.0, north=10.0)})
    assert m.ground_contacts == 2 and m.report(10.0)["result"] == "FAIL"
    landed = RunMetrics(1, sample_period_s=0.001)
    for t, (alt, mode) in enumerate([(5.0, 4), (2.0, 9), (0.1, 9)]):
        landed.update(float(t), {0: st(float(t), alt, mode)})
    assert landed.ground_contacts == 0
