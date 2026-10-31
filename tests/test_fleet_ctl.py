import pytest

from sitl.fleet_ctl import Step, parse_scenario, write_checksums


def test_parse_all_step_kinds():
    steps = parse_scenario("""
# comment
wait swarm ready 900
send patrol square 12 0.3
sleep 2.5
status
kill-agent 3
expect (landed|Landed) 120   # trailing comment
health
""")
    assert [s.kind for s in steps] == ["wait", "send", "sleep", "status", "kill-agent", "expect", "health"]
    assert steps[0] == Step("wait", "swarm ready", 900.0, 3)
    assert steps[1].arg == "patrol square 12 0.3"
    assert steps[4].arg == "3" and steps[5].arg == "(landed|Landed)"


@pytest.mark.parametrize("bad", ["wait nothing", "wait x -1", "wait ( 5", "sleep abc", "sleep -1", "kill-agent 0",
                                 "kill-agent x", "send", "fly 1"])
def test_parse_rejects_malformed_steps(bad):
    with pytest.raises(ValueError):
        parse_scenario(bad)


def test_shipped_scenario_parses():
    from pathlib import Path
    assert parse_scenario(Path("scenarios/election_10.txt").read_text())


def test_checksums_cover_files_and_exclude_themselves(tmp_path):
    (tmp_path / "a.log").write_text("x")
    (tmp_path / "sub").mkdir()
    (tmp_path / "sub" / "b.log").write_text("y")
    path = write_checksums(tmp_path)
    write_checksums(tmp_path)                      # idempotent
    lines = path.read_text().strip().splitlines()
    assert len(lines) == 2 and all("SHA256SUMS" not in line for line in lines)
