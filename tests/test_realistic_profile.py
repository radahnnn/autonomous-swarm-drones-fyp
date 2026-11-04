from types import SimpleNamespace

import pytest

from sitl.independent_swarm_console import REALISTIC_PEER_LINK, SITL_SENSOR_PROFILE_OVERRIDES
from sitl.swarm_console import apply_realistic


def _args(**kw):
    base = dict(realistic=True, sitl_sensor_profile=None, peer_loss=0.0, peer_latency_ms=30.0,
                peer_jitter_ms=None, peer_burst_loss=False)
    base.update(kw)
    return SimpleNamespace(**base)


def test_realistic_sets_profile_and_degraded_link():
    args = _args()
    apply_realistic(args)
    assert args.sitl_sensor_profile == "realistic"
    for name, value in REALISTIC_PEER_LINK.items():
        assert getattr(args, name) == value


def test_explicit_link_settings_are_kept_and_conflicting_profile_rejected():
    args = _args(peer_loss=0.2)
    apply_realistic(args)
    assert args.peer_loss == 0.2 and args.peer_latency_ms == REALISTIC_PEER_LINK["peer_latency_ms"]
    with pytest.raises(ValueError):
        apply_realistic(_args(sitl_sensor_profile="stress"))
    off = _args(realistic=False)
    apply_realistic(off)
    assert off.sitl_sensor_profile is None and off.peer_loss == 0.0


def test_profile_covers_every_realism_dimension():
    keys = set(SITL_SENSOR_PROFILE_OVERRIDES["realistic"])
    for needed in ("SIM_GPS1_HNSE", "SIM_GPS1_LAG_MS", "SIM_WIND_SPD", "SIM_WIND_TURB", "SIM_BATT_CAP_AH",
                   "SIM_BATT_RES_OHM", "BATT_FS_LOW_ACT", "SIM_VIB_MOT_MAX", "SIM_ACC1_RND", "SIM_GYR1_RND"):
        assert needed in keys
    assert SITL_SENSOR_PROFILE_OVERRIDES["realistic"]["BATT_FS_LOW_ACT"] == "1"     # land, never RTL into the swarm


def test_low_voltage_threshold_is_below_the_simulated_full_pack():
    profile = SITL_SENSOR_PROFILE_OVERRIDES["realistic"]
    assert float(profile["BATT_LOW_VOLT"]) < 12.6          # SITL quad = 3S at 12.6 V full (live finding)
    assert "SIM_BATT_VOLTAGE" not in profile                # ignored by the quad model; do not pretend otherwise


def test_sensor_profile_defaults_to_matek_compass_for_independent_runtime():
    # Kept from the retired tests/test_swarm_console.py (2026-11-04): this helper is still used by the entry point.
    from sitl.swarm_console import resolve_sitl_sensor_profile

    assert resolve_sitl_sensor_profile(None, "independent") == "matek-single-compass"
    assert resolve_sitl_sensor_profile(None, "independent", external_agents=True) == "baseline"
    assert resolve_sitl_sensor_profile("stress", "independent") == "stress"
