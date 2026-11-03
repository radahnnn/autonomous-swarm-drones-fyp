from tools.parameter_table import OUT, render


def test_parameter_table_matches_the_code():
    assert OUT.read_text(encoding="utf-8") == render(), "run: python3 -m tools.parameter_table"
