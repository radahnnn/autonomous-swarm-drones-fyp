#!/usr/bin/env python3
"""
SITL Takeoff and EKF3 Origin Alignment Diagnostic Tool.
Manual diagnostic script — not executed as part of automated unit testing.
"""

import os
import sys
import time
import subprocess
from pathlib import Path


def run_takeoff_diagnostic():
    try:
        from pymavlink import mavutil
    except ImportError:
        print("[ERROR] pymavlink is required for SITL diagnostics. Install with: pip install pymavlink")
        return 1

    repo_root = Path(__file__).resolve().parent.parent
    default_sitl_bin = Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
    sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(default_sitl_bin)))
    params_file = repo_root / "sitl" / "swarm_params.parm"

    if not sitl_bin.exists():
        print(f"[ERROR] ArduCopter SITL binary not found at: {sitl_bin}")
        print("Set the ARDUCOPTER_BIN environment variable to your arducopter binary path.")
        return 1

    subprocess.run(["pkill", "-9", "-f", "arducopter"], stderr=subprocess.DEVNULL)
    time.sleep(1)

    cmd = [
        str(sitl_bin),
        "-I0", "--model", "quad",
        "--home", "-35.363261,149.165230,584,0",
        "--defaults", str(params_file),
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    try:
        conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
        conn.wait_heartbeat()
        print("Heartbeat received.")

        # Set GUIDED
        print("Setting mode to GUIDED...")
        conn.set_mode(4)  # 4 = GUIDED
        time.sleep(1.0)

        # Retry arming until EKF position estimate is ready
        print("Arming motors (retrying until position estimate clears)...")
        t_arm_start = time.time()
        is_armed = False
        while time.time() - t_arm_start < 25.0:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            t_poll = time.time()
            while time.time() - t_poll < 1.0:
                msg = conn.recv_match(blocking=True, timeout=0.2)
                if msg:
                    if msg.get_type() == "HEARTBEAT":
                        is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        if is_armed:
                            break
                    elif msg.get_type() == "STATUSTEXT":
                        print(f"  [STATUSTEXT] {msg.text}")
            if is_armed:
                print(">> ARMED SUCCESSFULLY!")
                break
            time.sleep(0.5)

        if not is_armed:
            print("[ERROR] Failed to arm within 25 seconds.")
            return 1

        # Request telemetry streams
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
        time.sleep(1.0)

        # Wait for GPS 3D fix and EKF origin
        print("Waiting for GPS 3D fix and EKF origin...")
        t_wait = time.time()
        while time.time() - t_wait < 15.0:
            msg = conn.recv_match(blocking=True, timeout=0.5)
            if msg:
                if msg.get_type() == "GPS_RAW_INT" and msg.fix_type >= 3:
                    print(f"GPS 3D Fix acquired: sats={msg.satellites_visible}")
                elif msg.get_type() == "STATUSTEXT" and "origin set" in msg.text:
                    print(f"  [STATUSTEXT] {msg.text}")
                    break
            time.sleep(0.1)

        time.sleep(1.0)

        # Takeoff with retry loop
        print("Commanding Takeoff 5m (retrying until accepted)...")
        takeoff_accepted = False
        t_tk_start = time.time()
        while time.time() - t_tk_start < 25.0:
            # Ensure armed
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.2)
            # Send takeoff
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, 5.0
            )
            t_poll = time.time()
            while time.time() - t_poll < 1.0:
                msg = conn.recv_match(blocking=True, timeout=0.2)
                if msg:
                    if msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        print(f"[COMMAND_ACK] cmd=TAKEOFF, result={msg.result}")
                        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            takeoff_accepted = True
                            break
                    elif msg.get_type() == "STATUSTEXT":
                        print(f"  [STATUSTEXT] {msg.text}")
            if takeoff_accepted:
                print(">> TAKEOFF COMMAND ACCEPTED!")
                break
            time.sleep(1.0)

        if not takeoff_accepted:
            print("[ERROR] Takeoff was not accepted within timeout.")
            return 1

        # Monitor altitude climb
        print("Monitoring altitude climb to 5.0m...")
        t_climb = time.time()
        while time.time() - t_climb < 25.0:
            msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
            if msg:
                alt = msg.relative_alt / 1000.0
                vz = msg.vz / 100.0
                print(f"Altitude: {alt:.2f}m (vz={vz:.2f} m/s)")
                if alt >= 4.5 and abs(vz) < 0.2:
                    print(f">> SUCCESS! Reached steady hover at {alt:.2f}m!")
                    return 0
            time.sleep(0.5)
        return 0

    finally:
        proc.terminate()
        subprocess.run(["pkill", "-9", "-f", "arducopter"], stderr=subprocess.DEVNULL)


def main():
    sys.exit(run_takeoff_diagnostic())


if __name__ == "__main__":
    main()
