"""Tests for the unified swarm console: formation library, fleet layout and console logic (no hardware)."""

import itertools
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from sitl.fleet import DATUM_LAT, MAX_DRONES, build_fleet
from sitl.formation_library import MIN_SPACING, STYLES, assign_slots, slots_for
import sitl.swarm_console as console_module
from sitl.swarm_console import SwarmConsole, handle_command


# ---------------------------------------------------------------- formation library
@pytest.mark.parametrize("style", STYLES)
@pytest.mark.parametrize("n", range(1, MAX_DRONES + 1))
def test_slots_have_right_shape_and_safe_spacing(style, n):
    slots = slots_for(style, n)
    assert slots.shape == (n, 3)
    assert np.all(slots[:, 2] == 0.0)
    for i, j in itertools.combinations(range(n), 2):
        assert np.linalg.norm(slots[i, :2] - slots[j, :2]) >= MIN_SPACING - 1e-9


def test_legacy_v_tables_are_preserved():
    assert slots_for("v", 3)[:, :2].tolist() == [[0, 0], [-3, -3], [-3, 3]]
    assert slots_for("v", 5)[:, :2].tolist() == [[0, 0], [-3, -3], [-3, 3], [-6, -6], [-6, 6]]


def test_unknown_style_and_empty_swarm_rejected():
    with pytest.raises(ValueError):
        slots_for("blob", 3)
    with pytest.raises(ValueError):
        slots_for("v", 0)


def test_assign_slots_is_a_permutation_and_minimises_travel():
    slots = slots_for("v", 3)
    # Drone 0 sits at the right wing slot, drone 2 at the apex: matching must follow the drones.
    current = np.array([[-3.0, 3.0, 0.0], [-3.0, -3.0, 0.0], [0.0, 0.0, 0.0]])
    assert assign_slots(current, slots) == [2, 1, 0]
    assert sorted(assign_slots(np.random.default_rng(0).normal(size=(6, 3)), slots_for("ring", 6))) == list(range(6))


# ---------------------------------------------------------------- fleet layout
def test_fleet_ports_and_homes():
    fleet = build_fleet(5)
    assert [d.port for d in fleet] == [14552, 14562, 14572, 14582, 14592]
    assert [d.tcp_port for d in fleet] == [5760, 5770, 5780, 5790, 5800]
    assert [d.fdm_port for d in fleet] == [9002, 9012, 9022, 9032, 9042]
    assert [d.sysid for d in fleet] == [1, 2, 3, 4, 5]
    assert len({d.home_gazebo for d in fleet}) == 1          # Gazebo: one shared home
    assert len({d.home_standalone for d in fleet}) == 5      # standalone: one home per V slot


def test_standalone_homes_follow_the_v_slots():
    fleet = build_fleet(3)
    lat = [float(d.home_standalone.split(",")[0]) for d in fleet]
    lon = [float(d.home_standalone.split(",")[1]) for d in fleet]
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(DATUM_LAT))
    assert (lon[2] - lon[1]) * m_per_deg_lon == pytest.approx(6.0, abs=0.1)   # wings 6 m apart (E-W)
    assert (lat[0] - lat[1]) * 111_320.0 == pytest.approx(3.0, abs=0.1)        # wings 3 m behind apex


@pytest.mark.parametrize("n", [0, MAX_DRONES + 1])
def test_fleet_size_limits(n):
    with pytest.raises(ValueError):
        build_fleet(n)


# ---------------------------------------------------------------- console logic
def _console(n=5, airborne=True):
    c = SwarmConsole(n=n, use_gazebo=False)
    c.connections = [MagicMock() for _ in range(n)]
    c.frame_origins = [np.zeros(3) for _ in range(n)]
    if airborne:
        c.is_airborne = [True] * n
        c.current_offsets = np.array([[0.0, 3.0 * k, 0.0] for k in range(n)])
        c._calculate_active_topology()
    return c


def test_console_starts_grounded_with_no_targets():
    c = _console(airborne=False)
    assert not any(c.is_airborne)
    assert np.all(c.target_offsets == 0.0)


def test_topology_assigns_every_active_drone_a_distinct_slot():
    c = _console(5)
    rows = {tuple(r[:2]) for r in c.target_offsets}
    assert len(rows) == 5
    assert {tuple(r[:2]) for r in slots_for("v", 5)} == rows


def test_failure_promotes_leader_but_reserves_failed_slot_until_rejoin():
    c = _console(5)
    leader = c.current_leader_idx
    c.inject_failure(leader + 1)
    assert not c.is_airborne[leader] and c.simulated_faults[leader]
    assert c.current_leader_idx != leader and c._active(c.current_leader_idx)
    assert np.allclose(c.target_offsets, c.current_offsets)
    assert np.allclose(c.target_offsets[leader], c.current_offsets[leader])
    c.connections[leader].set_mode.assert_called_with(9)  # failed drone told to LAND


def test_auto_recovery_waits_for_stable_link_and_confirmed_disarm():
    c = _console(3, airborne=False)
    c.auto_recover = True
    c.simulated_faults[1] = True
    c.failure_causes[1] = "heartbeat_timeout"
    c.last_real_heartbeat[1] = 9.5
    c.last_armed_state[1] = False
    c.recovery_stable_since[1] = 6.0
    c.revive_drone = MagicMock()

    c._consider_auto_recovery(10.0)

    c.revive_drone.assert_called_once_with(2, auto_liftoff=True)
    assert c.recovery_attempted[1]


def test_auto_recovery_does_not_relaunch_operator_failed_or_armed_drone():
    c = _console(3, airborne=False)
    c.auto_recover = True
    c.simulated_faults[1] = True
    c.failure_causes[1] = "operator"
    c.last_real_heartbeat[1] = 10.0
    c.last_armed_state[1] = False
    c.recovery_stable_since[1] = 1.0
    c.revive_drone = MagicMock()

    c._consider_auto_recovery(10.0)

    c.revive_drone.assert_not_called()
    assert c.recovery_stable_since[1] is None


def test_failure_of_unknown_drone_is_rejected_without_error(capsys):
    c = _console(3)
    c.inject_failure(9)
    assert "Drone ID must be 1 to 3" in capsys.readouterr().out
    assert all(c.is_airborne)


def test_set_formation_style_updates_targets_and_validates():
    c = _console(5)
    c.set_formation_style("line")
    assert {round(float(r[1]), 1) for r in c.target_offsets} == {-6.0, -3.0, 0.0, 3.0, 6.0}
    c.set_formation_style("nonsense")
    assert c.current_formation_style == "line"


def test_liftoff_of_airborne_drone_is_refused(capsys):
    c = _console(3)
    c.liftoff_drone(1)
    assert "already airborne" in capsys.readouterr().out
    assert not any(c.busy)


def test_launch_with_nothing_grounded_does_nothing(capsys):
    c = _console(3)
    c.launch_swarm()
    assert "Nothing to launch" in capsys.readouterr().out


def test_setpoints_go_only_to_airborne_drones_with_common_yaw():
    c = _console(3, airborne=False)
    c.is_airborne = [True, False, True]
    c.current_offsets = slots_for("v", 3).copy()
    c.send_formation_setpoints()
    c.connections[1].mav.set_position_target_local_ned_send.assert_not_called()
    for i in (0, 2):
        args = c.connections[i].mav.set_position_target_local_ned_send.call_args[0]
        assert not args[4] & (1 << 10) and args[-2] == c.FORMATION_YAW_RAD


def test_commands_move_morph_and_exit(monkeypatch):
    monkeypatch.setattr(console_module, "_armed_state", lambda _conn: False)
    c = _console(3)
    start = c.centroid_pos.copy()
    assert handle_command(c, "w 10") is True
    assert c.centroid_pos[0] == pytest.approx(start[0] + 10.0)
    assert handle_command(c, "u") is True and c.centroid_pos[2] == pytest.approx(start[2] - 2.0)
    assert handle_command(c, "tri") is True and c.current_formation_style == "triangle"
    assert handle_command(c, "") is True
    assert handle_command(c, "q") is False
    assert c.running is False
    for conn in c.connections:
        conn.set_mode.assert_called_with(9)


def test_land_all_waits_for_every_vehicle_to_disarm(monkeypatch):
    # First poll still has two armed vehicles; landing completes only after the
    # second poll confirms all three have disarmed.
    states = iter([True, True, False, False, False, False])
    monkeypatch.setattr(console_module, "_armed_state", lambda _conn: next(states))
    monkeypatch.setattr(console_module.time, "sleep", lambda _seconds: None)
    c = _console(3)

    assert c.land_all(timeout=1.0) is True
    for conn in c.connections:
        conn.mav.command_long_send.assert_called_once()
