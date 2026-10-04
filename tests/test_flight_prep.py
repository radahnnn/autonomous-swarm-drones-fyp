"""Tests for sitl.flight_prep: arming/takeoff must be verified, never assumed."""

from unittest.mock import MagicMock

from pymavlink import mavutil

from sitl.flight_prep import arm_with_retry, takeoff_and_verify

ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def _msg(mtype, **kw):
    m = MagicMock()
    m.get_type.return_value = mtype
    for k, v in kw.items():
        setattr(m, k, v)
    return m


def _conn(messages):
    """Connection whose recv_match yields `messages` in order, then None forever."""
    conn = MagicMock()
    it = iter(messages)
    conn.recv_match.side_effect = lambda *a, **k: next(it, None)
    return conn


def test_arm_succeeds_once_heartbeat_reports_armed():
    conn = _conn([
        _msg("STATUSTEXT", text="Arm: Need Position Estimate"),
        _msg("HEARTBEAT", base_mode=0),
        _msg("HEARTBEAT", base_mode=ARMED),
    ])
    assert arm_with_retry(conn, "D0", timeout=10.0) is True
    assert conn.mav.command_long_send.called


def test_arm_fails_loudly_when_never_armed():
    conn = _conn([_msg("STATUSTEXT", text="Arm: Need Position Estimate")])
    assert arm_with_retry(conn, "D0", timeout=2.5) is False


def test_takeoff_confirmed_when_altitude_rises():
    conn = _conn([_msg("GLOBAL_POSITION_INT", relative_alt=2500)])
    assert takeoff_and_verify(conn, "D0", 5.0, timeout=5.0) is True


def test_takeoff_reports_failure_when_vehicle_stays_on_ground():
    conn = _conn([_msg("GLOBAL_POSITION_INT", relative_alt=10)])
    assert takeoff_and_verify(conn, "D0", 5.0, timeout=2.0) is False


def test_gazebo_mode_uses_one_shared_home():
    """Staggered SITL homes shift each drone away from its Gazebo spawn point and break the V."""
    from sitl.start_and_climb import SwarmStartAndClimb

    homes = {cfg["home"] for cfg in SwarmStartAndClimb().drone_configs}
    assert len(homes) == 1


def test_formation_setpoints_command_one_common_heading():
    """All drones get position + the same fixed yaw (yaw-ignore bit 10 cleared), so they face one way."""
    import numpy as np

    from sitl.start_and_climb import SwarmStartAndClimb

    swarm = SwarmStartAndClimb()
    swarm.connections = [MagicMock() for _ in range(3)]
    swarm.frame_origins = [np.zeros(3)] * 3
    swarm.send_formation_setpoints()
    yaws = set()
    for conn in swarm.connections:
        args = conn.mav.set_position_target_local_ned_send.call_args[0]
        type_mask, yaw = args[4], args[-2]
        assert not type_mask & (1 << 10), "yaw must not be ignored"
        assert type_mask & (1 << 11), "yaw rate stays ignored"
        yaws.add(yaw)
    assert yaws == {swarm.FORMATION_YAW_RAD}
