import numpy as np
import pytest

from sitl import patrol_layout
from sitl.formation_library import FORMATION_SCALE_ENV, scaled_pair_sector_centres, slots_for


def test_scale_spreads_every_formation_and_the_survey_points(monkeypatch):
    base = slots_for("v", 6)
    monkeypatch.setenv(FORMATION_SCALE_ENV, "1.4")
    np.testing.assert_allclose(slots_for("v", 6)[:, :2], base[:, :2] * 1.4)
    pairs = slots_for("pairs", 6)[:, :2]
    centres = scaled_pair_sector_centres(3)
    for k in range(3):
        np.testing.assert_allclose((pairs[2 * k] + pairs[2 * k + 1]) / 2, centres[2 * k], atol=1e-9)
    np.testing.assert_allclose(patrol_layout.survey_points(), centres)


@pytest.mark.parametrize("bad", ["0.5", "4", "nan", "x"])
def test_bad_scale_is_rejected(monkeypatch, bad):
    monkeypatch.setenv(FORMATION_SCALE_ENV, bad)
    with pytest.raises(ValueError):
        slots_for("v", 3)


def test_patrol_layout_still_keeps_poles_out_of_the_pairs_way_when_spread(monkeypatch):
    import math
    monkeypatch.setenv(FORMATION_SCALE_ENV, "1.4")
    for style in ("pairs", "pairs_b"):
        for slot in slots_for(style, 6)[:, :2]:
            for north, east, radius in patrol_layout.poles():
                assert math.hypot(slot[0] - north, slot[1] - east) > radius + 3.0
