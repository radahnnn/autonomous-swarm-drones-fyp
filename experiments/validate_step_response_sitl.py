"""
Validate Sim Against SITL: Position Step Response & Parameter Fitting (Task C Item 1).
1. Launches ArduPilot SITL in headless GUIDED mode.
2. Arms, takes off to 5.0m altitude, and establishes stable hover.
3. Injects a 5.0m North position step setpoint.
4. Logs SITL position and velocity response at 20 Hz.
5. Fits first-order attitude lag (tau) and aerodynamic rotor drag (c_d) for swarm_core.
6. Reports the fitted parameters, residual RMSE, and saves CSV and plot.
"""

import os
import sys
import time
import math
import subprocess
import shutil
import numpy as np
import pandas as pd
from scipy.optimize import minimize
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pathlib import Path
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

from swarm_core.drone import Drone


def run_sitl_step_test(sim_time_limit: float = 8.0, step_distance: float = 5.0):
    if mavutil is None:
        print("[ERROR] pymavlink is required for SITL validation. Install with: pip install pymavlink")
        sys.exit(1)

    print("=================================================================")
    print("  TASK C.1: ARDUPILOT SITL 5m POSITION STEP RESPONSE TEST        ")
    print("=================================================================")

    # Ensure any stray SITL processes are killed
    subprocess.run(["pkill", "-9", "-f", "arducopter"], stderr=subprocess.DEVNULL)
    time.sleep(1)

    repo_root = Path(__file__).resolve().parent.parent
    default_sitl_bin = Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
    sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(default_sitl_bin)))

    if not sitl_bin.exists():
        print(f"[ERROR] ArduCopter SITL binary not found at: {sitl_bin}")
        print("Please install ArduPilot SITL or set the ARDUCOPTER_BIN environment variable.")
        sys.exit(1)

    home_loc = "-35.363261,149.165230,584,0"
    params_file = str(repo_root / "sitl" / "swarm_params.parm")

    # Launch ArduCopter SITL headless with custom parameters
    cmd = [
        str(sitl_bin),
        "-I0",
        "--model", "quad",
        "--home", home_loc,
        "--defaults", params_file,
        "--speedup", "1",
    ]
    print(f"Launching SITL binary: {' '.join(cmd)}")
    sitl_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    try:
        # Connect via TCP
        print("Connecting MAVLink to tcp:127.0.0.1:5760...")
        conn = None
        for attempt in range(15):
            try:
                conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
                msg = conn.wait_heartbeat(timeout=3.0)
                if msg:
                    print("Connected to ArduPilot SITL!")
                    break
            except Exception:
                time.sleep(0.5)

        if not conn:
            raise RuntimeError("Failed to connect to SITL MAVLink.")

        # Set parameter ARMING_CHECK to 0 for instant automated test arming
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            b"ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.5)

        # Request continuous 20 Hz telemetry stream for all messages (including LOCAL_POSITION_NED)
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
        )
        time.sleep(0.5)

        # 1. Wait for EKF origin and GPS lock (takes ~10-12s after boot in ArduPilot SITL)
        print("Waiting for EKF origin and GPS 3D lock...")
        t_wait = time.time()
        origin_set = False
        while time.time() - t_wait < 25.0:
            msg = conn.recv_match(blocking=True, timeout=0.5)
            if msg:
                if msg.get_type() == "STATUSTEXT" and "origin set" in msg.text:
                    print(f"  [STATUSTEXT] {msg.text}")
                    origin_set = True
                    break
            time.sleep(0.1)

        # Allow EKF to finish field elevation and heading initialization
        time.sleep(2.0)

        # 2. Set GUIDED mode
        print("Setting mode to GUIDED...")
        conn.set_mode(4)  # 4 = GUIDED
        time.sleep(1.0)

        # 3. Arm motors
        print("Arming motors...")
        t_arm_start = time.time()
        is_armed = False
        while time.time() - t_arm_start < 15.0:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0
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
            raise RuntimeError("Failed to arm motors within timeout.")

        # 4. Command Takeoff to 5.0m and verify acceptance
        print("Commanding takeoff to 5.0m...")
        takeoff_accepted = False
        for attempt in range(5):
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, 5.0
            )
            t_ack = time.time()
            while time.time() - t_ack < 1.0:
                msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
                if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    print(f"Takeoff COMMAND_ACK: result={msg.result}")
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        takeoff_accepted = True
                        break
            if takeoff_accepted:
                break
            time.sleep(0.5)

        if not takeoff_accepted:
            raise RuntimeError("Takeoff command was rejected by autopilot (result != ACCEPTED).")

        # 5. Wait until hovering at 5.0m altitude
        print("Waiting for takeoff to stabilize at 5.0m...")
        t_start = time.time()
        hover_reached = False
        last_print = 0
        while time.time() - t_start < 25.0:
            latest_msg = None
            while True:
                m = conn.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=False)
                if not m:
                    break
                latest_msg = m
            
            if latest_msg:
                if latest_msg.get_type() == "GLOBAL_POSITION_INT":
                    rel_alt = latest_msg.relative_alt / 1000.0
                    vz = latest_msg.vz / 100.0
                else:
                    rel_alt = -latest_msg.z
                    vz = latest_msg.vz
                
                if time.time() - last_print > 1.0:
                    print(f"  Climbing: alt={rel_alt:.2f}m, vz={vz:.2f} m/s")
                    last_print = time.time()

                if rel_alt >= 4.5 and abs(vz) < 0.30:
                    print(f">> Reached steady hover at alt={rel_alt:.2f}m!")
                    hover_reached = True
                    break
            time.sleep(0.05)

        if not hover_reached:
            raise RuntimeError(f"Drone failed to climb to 5.0m target altitude within 25s (last alt={rel_alt if 'rel_alt' in locals() else 'None'}).")

        time.sleep(1.5)  # Let hover settle completely

        # Ingest origin local position
        x0, y0, z0 = 0.0, 0.0, -5.0
        for _ in range(20):
            msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
            if msg:
                x0, y0, z0 = msg.x, msg.y, msg.z
                break
            time.sleep(0.1)
        print(f"Origin hover position: X={x0:.2f}m, Y={y0:.2f}m, Z={z0:.2f}m")
        if z0 > -3.0:
            raise RuntimeError(f"Vehicle vertical altitude abnormal (Z={z0:.2f}m).")

        # Command 5m North step: target X = x0 + step_distance
        target_x = x0 + step_distance
        target_y = y0
        target_z = z0

        print(f">> INJECTING {step_distance:.1f}m POSITION STEP (Target X={target_x:.2f}m)...")
        type_mask = 0b0000111111111000  # Position setpoint only

        data_log = []
        t_step_start = time.time()

        while time.time() - t_step_start < sim_time_limit:
            t_now = time.time() - t_step_start

            # Continuously stream setpoint at 20 Hz
            conn.mav.set_position_target_local_ned_send(
                0, conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                target_x, target_y, target_z,
                0, 0, 0, 0, 0, 0, 0, 0
            )

            # Drain MAVLink message buffer
            latest_pos = None
            while True:
                m = conn.recv_match(blocking=False)
                if not m:
                    break
                if m.get_type() == "LOCAL_POSITION_NED":
                    latest_pos = m

            if latest_pos:
                rel_x = latest_pos.x - x0
                rel_y = latest_pos.y - y0
                data_log.append({
                    "time": t_now,
                    "x": rel_x,
                    "y": rel_y,
                    "vx": latest_pos.vx,
                    "vy": latest_pos.vy,
                    "target_x": step_distance,
                })
            elif len(data_log) > 0:
                # If frame skipped this 50ms tick, carry forward previous state
                last_frame = data_log[-1].copy()
                last_frame["time"] = t_now
                data_log.append(last_frame)

            time.sleep(0.05)

        print(f"Recorded {len(data_log)} telemetry frames from SITL.")
        return pd.DataFrame(data_log)

    finally:
        # Cleanly terminate SITL process
        sitl_proc.terminate()
        try:
            sitl_proc.wait(timeout=2.0)
        except Exception:
            sitl_proc.kill()
        subprocess.run(["pkill", "-9", "-f", "arducopter"], stderr=subprocess.DEVNULL)


def simulate_swarm_core_step(times: np.ndarray, tau: float, drag: float, step_dist: float = 5.0, kp: float = 1.8, kd: float = 2.2):
    """Simulates swarm_core Drone kinematics with attitude lag tau and rotor drag."""
    drone = Drone(
        drone_id=0,
        initial_position=np.array([0.0, 0.0]),
        max_speed=3.0,
        max_accel=2.5,
        attitude_tau=tau,
        drag_coeff=drag,
        measurement_noise_std=0.0,
    )
    target = np.array([step_dist, 0.0])
    xs = []
    vxs = []

    dt = times[1] - times[0] if len(times) > 1 else 0.05
    for t in times:
        # Commanded acceleration: PD tracking law
        p_err = target - drone.position
        v_err = -drone.velocity
        a_cmd = kp * p_err + kd * v_err
        drone.set_control_input(a_cmd)
        drone.step(dt)
        xs.append(drone.position[0])
        vxs.append(drone.velocity[0])

    return np.array(xs), np.array(vxs)


def fit_model_parameters(sitl_df: pd.DataFrame, step_dist: float = 5.0):
    """Finds optimal (tau, drag) minimizing RMSE against SITL position trajectory."""
    times = sitl_df["time"].values
    x_sitl = sitl_df["x"].values

    def loss(params):
        tau, drag = params
        x_sim, _ = simulate_swarm_core_step(times, tau, drag, step_dist=step_dist)
        rmse = np.sqrt(np.mean((x_sim - x_sitl) ** 2))
        return rmse

    # Initial guess from assumed config: tau=0.18, drag=0.20
    init_guess = [0.25, 0.25]
    bounds = [(0.05, 1.50), (0.01, 1.50)]

    res = minimize(loss, init_guess, bounds=bounds, method="L-BFGS-B")
    best_tau, best_drag = res.x
    best_rmse = res.fun
    return best_tau, best_drag, best_rmse


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)
    csv_path = f"{output_dir}/sitl_step_response.csv"

    # 1. Run SITL step test or load existing CSV
    if "--refit-only" in sys.argv and os.path.exists(csv_path):
        print(f"Loading existing SITL telemetry CSV from: {csv_path}")
        sitl_df = pd.read_csv(csv_path)
    else:
        sitl_df = run_sitl_step_test(sim_time_limit=8.0, step_distance=5.0)
        if len(sitl_df) == 0:
            print("Error: No data recorded from SITL.")
            sys.exit(1)
        sitl_df.to_csv(csv_path, index=False)
        print(f"Saved SITL telemetry CSV to: {csv_path}")

    # 2. Fit tau and drag parameters
    best_tau, best_drag, best_rmse = fit_model_parameters(sitl_df, step_dist=5.0)

    print("\n=================================================================")
    print("  PARAMETER IDENTIFICATION & MODEL FITTING RESULTS               ")
    print("=================================================================")
    print(f"  Assumed Initial Parameters : tau = 0.180 s, drag = 0.200 1/s")
    print(f"  Fitted Parameters from SITL: tau = {best_tau:.3f} s, drag = {best_drag:.3f} 1/s")
    print(f"  Residual RMSE vs SITL Track: {best_rmse:.4f} m ({best_rmse*100:.2f} cm)")
    print("=================================================================")

    # 3. Simulate with fitted parameters and assumed parameters
    times = sitl_df["time"].values
    x_fitted, vx_fitted = simulate_swarm_core_step(times, best_tau, best_drag, step_dist=5.0)
    x_assumed, vx_assumed = simulate_swarm_core_step(times, 0.18, 0.20, step_dist=5.0)

    # 4. Generate validation plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.2), dpi=180)

    # Position Response
    ax1.plot(times, sitl_df["x"], "k-", linewidth=2.5, label="ArduPilot SITL (Actual 5m Step)")
    ax1.plot(times, x_fitted, "g--", linewidth=2.0, label=f"swarm_core Fitted ($\\tau$={best_tau:.3f}s, $c_d$={best_drag:.3f})")
    ax1.plot(times, x_assumed, "r:", linewidth=1.8, label="swarm_core Baseline ($\\tau$=0.18s, $c_d$=0.20)")
    ax1.axhline(5.0, color="blue", linestyle="--", alpha=0.5, label="Target Step (5.0m)")
    ax1.set_xlabel("Time (s)", fontweight="bold")
    ax1.set_ylabel("Position (m)", fontweight="bold")
    ax1.set_title(f"Position Step Response Comparison (RMSE = {best_rmse*100:.1f} cm)", fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="lower right")

    # Velocity Response
    ax2.plot(times, sitl_df["vx"], "k-", linewidth=2.5, label="ArduPilot SITL Velocity")
    ax2.plot(times, vx_fitted, "g--", linewidth=2.0, label="swarm_core Fitted Velocity")
    ax2.plot(times, vx_assumed, "r:", linewidth=1.8, label="swarm_core Baseline Velocity")
    ax2.set_xlabel("Time (s)", fontweight="bold")
    ax2.set_ylabel("Velocity (m/s)", fontweight="bold")
    ax2.set_title("Velocity Profile Overlay", fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    plt.tight_layout()
    plot_path = f"{output_dir}/sitl_step_response_fit.png"
    plt.savefig(plot_path)
    plt.close(fig)

    print(f"Validation plot saved to: {plot_path}.")


if __name__ == "__main__":
    main()
