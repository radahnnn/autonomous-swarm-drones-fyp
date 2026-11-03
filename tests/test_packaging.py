"""CI installs `.[all]`: it must contain everything in `dev` and `sitl` (Codex review #12: pyflakes was missing)."""
from pathlib import Path

import pytest

tomllib = pytest.importorskip("tomllib")      # Python 3.11+; CI also runs 3.10


def test_all_extra_contains_dev_and_sitl():
    extras = tomllib.loads(Path("pyproject.toml").read_text())["project"]["optional-dependencies"]
    assert set(extras["dev"]) | set(extras["sitl"]) <= set(extras["all"])
