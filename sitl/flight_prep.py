"""
Shared arming / takeoff helpers for the SITL swarm flight scripts.

Why this exists: ArduCopter refuses to arm in GUIDED until the EKF has a position
estimate ("Arm: Need Position Estimate"). With Gazebo running slower than real time
that can take well over the 15-25 s the flight scripts used to wait, and the scripts
then printed "Takeoff commanded" for a vehicle that was never armed. These helpers
retry until the vehicle really is armed (and really climbing) or fail loudly.
"""

import time
from typing import Optional

from pymavlink import mavutil

GUIDED_MODE = 4
_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
_FORCE_ARM_MAGIC = 21196


def _drain(conn, wanted=("HEARTBEAT", "COMMAND_ACK", "STATUSTEXT", "GLOBAL_POSITION_INT"), budget: float = 0.3):
    """Read pending messages for up to `budget` seconds, returning the latest of each type."""
    latest = {}
    t_end = time.time() + budget
    while time.time() < t_end:
        msg = conn.recv_match(type=list(wanted), blocking=False)
        if msg is None:
            time.sleep(0.02)
            continue
        latest[msg.get_type()] = msg
    return latest


def is_armed(conn) -> bool:
    latest = _drain(conn, ("HEARTBEAT",), budget=0.4)
    hb = latest.get("HEARTBEAT")
    return bool(hb and (hb.base_mode & _ARMED))


def arm_with_retry(conn, label: str, timeout: float = 180.0, force_after: float = 90.0, cancelled=None) -> bool:
    """
    Put the vehicle in GUIDED and arm it, retrying until the heartbeat reports ARMED.
    A normal arm is retried first (it succeeds as soon as the EKF has a position
    estimate); after `force_after` seconds a forced arm is attempted as a last resort.
    Returns True only if the vehicle is actually armed.
    """
    t0 = time.time()
    last_reason = ""
    last_print = 0.0
    while time.time() - t0 < timeout:
        if cancelled is not None and cancelled():
            return False
        conn.set_mode(GUIDED_MODE)
        force = (time.time() - t0) >= force_after
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, _FORCE_ARM_MAGIC if force else 0, 0, 0, 0, 0, 0,
        )
        t_poll = time.time()
        while time.time() - t_poll < 2.0:
            if cancelled is not None and cancelled():
                return False
            latest = _drain(conn, budget=0.3)
            st = latest.get("STATUSTEXT")
            if st is not None and "arm" in st.text.lower():
                last_reason = st.text
            hb = latest.get("HEARTBEAT")
            if hb is not None and (hb.base_mode & _ARMED):
                print(f"  [{label}] ARMED after {time.time() - t0:.0f}s")
                return True
        if time.time() - last_print > 10.0:
            last_print = time.time()
            print(f"  [{label}] waiting to arm ({time.time() - t0:.0f}s) {last_reason}".rstrip())
    print(f"  [{label}] ERROR: not armed after {timeout:.0f}s. Last message: {last_reason or 'none'}")
    return False


def takeoff_and_verify(conn, label: str, target_alt: float, timeout: float = 60.0, min_climb: float = 0.5) -> bool:
    """Command takeoff and confirm the vehicle actually leaves the ground (relative alt >= min_climb)."""
    t0 = time.time()
    last_cmd = 0.0
    alt: Optional[float] = None
    while time.time() - t0 < timeout:
        if time.time() - last_cmd > 5.0:
            last_cmd = time.time()
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, float(target_alt),
            )
        latest = _drain(conn, ("GLOBAL_POSITION_INT",), budget=0.3)
        gp = latest.get("GLOBAL_POSITION_INT")
        if gp is not None:
            alt = gp.relative_alt / 1000.0
            if alt >= min_climb:
                print(f"  [{label}] Takeoff confirmed (alt {alt:.2f} m)")
                return True
    print(f"  [{label}] ERROR: no climb after {timeout:.0f}s (alt {alt if alt is not None else 'unknown'})")
    return False


# EKF_STATUS_FLAGS bits (MAVLink common.xml)
_EKF_ATTITUDE = 1
_EKF_VEL_H = 2
_EKF_VEL_V = 4
_EKF_POS_REL = 8
_EKF_POS_ABS = 16
_EKF_POS_VERT_ABS = 32
_EKF_CONST_POS_MODE = 128
_EKF_UNINITIALIZED = 1024
_EKF_REQUIRED = _EKF_ATTITUDE | _EKF_VEL_H | _EKF_VEL_V | _EKF_POS_REL | _EKF_POS_ABS | _EKF_POS_VERT_ABS
_EKF_FORBIDDEN = _EKF_CONST_POS_MODE | _EKF_UNINITIALIZED


def request_message_interval(conn, msg_id: int, hz: float) -> None:
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
    )


def wait_ekf_ready(conn, label: str, wall_timeout: float = 300.0, stable: int = 3, cancelled=None) -> bool:
    """
    Block until the EKF reports a converged absolute position (and GPS has a 3D fix)
    in `stable` consecutive EKF_STATUS_REPORT messages. Progress is driven by the
    vehicle's own telemetry, so it adapts to any Gazebo real-time factor.
    Must not be called while another thread is reading from `conn`.
    """
    request_message_interval(conn, 193, 2.0)  # EKF_STATUS_REPORT
    request_message_interval(conn, 24, 2.0)   # GPS_RAW_INT
    t0 = time.time()
    last_print = 0.0
    good = 0
    fix = 0
    flags = 0
    while time.time() - t0 < wall_timeout:
        if cancelled is not None and cancelled():
            return False
        m = conn.recv_match(type=["EKF_STATUS_REPORT", "GPS_RAW_INT"], blocking=True, timeout=1.0)
        if m is not None:
            if m.get_type() == "GPS_RAW_INT":
                fix = m.fix_type
            else:
                flags = m.flags
                ok = (flags & _EKF_REQUIRED) == _EKF_REQUIRED and not (flags & _EKF_FORBIDDEN) and fix >= 3
                good = good + 1 if ok else 0
                if good >= stable:
                    print(f"  [{label}] EKF ready after {time.time() - t0:.0f}s (flags=0x{flags:x}, gps_fix={fix})")
                    return True
        if time.time() - last_print > 10.0:
            last_print = time.time()
            print(f"  [{label}] waiting for EKF ({time.time() - t0:.0f}s) flags=0x{flags:x} gps_fix={fix}")
    print(f"  [{label}] ERROR: EKF not ready after {wall_timeout:.0f}s (flags=0x{flags:x}, gps_fix={fix})")
    return False
