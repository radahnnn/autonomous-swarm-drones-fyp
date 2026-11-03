import json

from tools.campaign_report import summarise


def _run(root, name, result, est, true, ok=True):
    d = root / name / "evidence" / "x"
    d.mkdir(parents=True)
    (d / "run_report.json").write_text(json.dumps({"result": result, "min_gap_m": est, "true_min_gap_m": true}))
    (root / name / "driver.log").write_text("health: unexpectedly_dead=[]\n" + ("" if ok else "expect 'x': TIMEOUT\n"))


def test_groups_runs_by_scenario_and_reports_distributions(tmp_path):
    _run(tmp_path, "calm-1", "PASS", 5.2, 5.1)
    _run(tmp_path, "calm-2", "PASS", 5.0, 4.9)
    _run(tmp_path, "gap-1", "FAIL", 1.0, 0.9, ok=False)
    text = summarise(tmp_path)
    assert "| calm | 2 | 2 / 0 / 0 | 2/2 | 5.10 (5.00) | 5.00 (4.90) |" in text
    assert "| gap | 1 | 0 / 1 / 0 | 0/1 |" in text
