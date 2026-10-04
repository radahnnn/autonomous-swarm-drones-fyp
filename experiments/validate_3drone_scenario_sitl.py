#!/usr/bin/env python3
"""
Task C Item 2 / Recommendations 14 & 15: Validate Sim Against SITL (3-Drone Formation Scenario).
Runs the same 3-drone scenario in swarm_core and in ArduPilot SITL on a common timebase:
- 3 drones flying in V-formation (Apex Drone 0, Left Wing Drone 1, Right Wing Drone 2).
- Scenario: Centroid translates 8.0 m North over 8.0 s (v_cmd = 1.0 m/s), then hovers for 2.0 s.
- Uses unified V-geometry from swarm_core.formations everywhere.
- Dispatches position + velocity setpoints (type mask 0x0DC0) with velocity = centroid velocity.
- Directly uses CentralizedController from swarm_core (no hand-coded PD).
- Replaces target-filled missing frames with NaN and reports missing frame counts.
- Aligns sim and SITL timestamps by logging sim state before step.
- Generates 4-panel publication plots including signed North and East errors vs time.
- Reports per-drone and overall swarm RMS difference before/after.
"""

import os
import sys
import time
import math
import shutil
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
try:
    import pandas as pd
except ImportError:
    import experiments.dataframe_shim as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

from swarm_core.config import get_profile, build_controllers_from_profile
from swarm_core.drone import Drone
from swarm_core.formations import FormationGenerator, FormationType
from swarm_core.controllers.centralized import CentralizedController
from sitl.common_frame import CommonCoordinateFrame


def get_unified_v_geometry(spacing: float = 3.0) -> np.ndarray:
    """Returns unified 3-drone V-formation offsets from swarm_core.formations."""
    return FormationGenerator.generate_v_shape(num_drones=3, spacing=spacing)


def simulate_swarm_core_3drone(
    duration: float = 10.0,
    dt: float = 0.1,
    tau: float = 0.992,
    drag: float = 0.637,
    spacing: float = 3.0,
) -> pd.DataFrame:
    """
    Simulates the 3-drone V-formation scenario directly using swarm_core CentralizedController.
    Logs simulation state BEFORE drone.step() to align timestamps with SITL t=0.
    """
    v_offsets = get_unified_v_geometry(spacing=spacing)
    profile = get_profile("sitl_fitted")

    drones = [
        Drone(
            drone_id=i,
            initial_position=v_offsets[i].copy(),
            max_speed=float(profile.get("max_speed")),
            max_accel=float(profile.get("max_accel")),
            attitude_tau=tau,
            drag_coeff=drag,
            measurement_noise_std=0.0,
        )
        for i in range(3)
    ]

    central_ctrl, _, _ = build_controllers_from_profile(profile, nominal_latency=0.0)

    n_steps = int(duration / dt)
    records = []
    centroid_pos = np.array([0.0, 0.0], dtype=np.float64)

    for step in range(n_steps):
        t = step * dt

        # Commanded centroid velocity: forward 1.0 m/s for 8s, then hold at 8m
        if t <= 8.0:
            c_vx = 1.0
        else:
            c_vx = 0.0
        centroid_vel = np.array([c_vx, 0.0], dtype=np.float64)

        # 1. LOG SIM STATE BEFORE STEP to align with SITL t=0.0 initial telemetry (Item 14)
        for i, drone in enumerate(drones):
            records.append({
                "time": round(t, 2),
                "drone_id": i,
                "sim_x": drone.position[0],
                "sim_y": drone.position[1],
                "sim_vx": drone.velocity[0],
                "sim_vy": drone.velocity[1],
                "target_x": centroid_pos[0] + v_offsets[i, 0],
                "target_y": centroid_pos[1] + v_offsets[i, 1],
            })

        # 2. Compute control accelerations directly via CentralizedController (Item 15)
        accels = central_ctrl.compute_control_inputs(
            drones=drones,
            formation_type=FormationType.V_SHAPE,
            centroid_target=centroid_pos,
            centroid_velocity=centroid_vel,
            spacing=spacing,
            use_velocity_feedforward=True,
            drag_coeff=drag,
        )

        # 3. Integrate physics step
        for i, drone in enumerate(drones):
            drone.set_control_input(accels[i])
            drone.step(dt)

        centroid_pos = centroid_pos + centroid_vel * dt

    return pd.DataFrame(records)


def run_sitl_3drone_scenario(
    duration: float = 10.0,
    dt: float = 0.1,
    spacing: float = 3.0,
    use_velocity_feedforward: bool = True,
) -> Tuple[pd.DataFrame, Dict[str, int]]:
    """
    Runs the 3-drone scenario in ArduPilot SITL using CommonCoordinateFrame.
    Sends position + velocity setpoints (type mask 0x0DC0) with velocity = centroid_vel.
    Replaces missing telemetry frames with NaN and reports counts.
    """
    print("\n=================================================================")
    print("  TASK C.2: ARDUPILOT SITL 3-DRONE SCENARIO EXECUTION            ")
    print(f"  Setpoint Type Mask: {'0x0DC0 (Pos+Vel Feedforward)' if use_velocity_feedforward else '0x0DF8 (Pos Only)'}")
    print("=================================================================")

    # Ensure any previous SITL processes are killed
    subprocess.run(["pkill", "-9", "-f", "arducopter"], stderr=subprocess.DEVNULL)
    time.sleep(1)

    repo_root = Path(__file__).resolve().parent.parent
    default_sitl_bin = Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
    sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(default_sitl_bin)))

    if not sitl_bin.exists():
        print(f"[ERROR] ArduCopter SITL binary not found at: {sitl_bin}")
        print("Please install ArduPilot SITL or set the ARDUCOPTER_BIN environment variable.")
        sys.exit(1)

    params_file = str(repo_root / "sitl" / "swarm_params.parm")
    datum_lat, datum_lon, datum_alt = -35.363261, 149.165230, 584.0
    frame = CommonCoordinateFrame(datum_lat, datum_lon, datum_alt)

    drone_configs = [
        {"id": 0, "inst": 0, "port": 5760, "home": "-35.363261,149.165230,584,0", "label": "Drone 0 (Apex)"},
        {"id": 1, "inst": 1, "port": 5770, "home": "-35.363261,149.165285,584,0", "label": "Drone 1 (Left Wing)"},
        {"id": 2, "inst": 2, "port": 5780, "home": "-35.363261,149.165340,584,0", "label": "Drone 2 (Right Wing)"},
    ]

    procs = []
    connections = []
    missing_frame_counts = {i: 0 for i in range(3)}

    try:
        # 1. Launch 3 SITL instances
        for cfg in drone_configs:
            cmd = [
                str(sitl_bin),
                f"-I{cfg['inst']}",
                "--model", "quad",
                "--home", cfg["home"],
                "--defaults", params_file,
                "--speedup", "1",
            ]
            print(f"Launching {cfg['label']} on TCP {cfg['port']}...")
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            procs.append(p)

        time.sleep(2)

        # 2. Connect MAVLink to all 3 drones
        for cfg in drone_configs:
            print(f"Connecting to {cfg['label']} (tcp:127.0.0.1:{cfg['port']})...")
            conn = None
            for _ in range(15):
                try:
                    conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{cfg['port']}")
                    msg = conn.wait_heartbeat(timeout=3.0)
                    if msg:
                        break
                except Exception:
                    time.sleep(0.5)
            if not conn:
                raise RuntimeError(f"Failed to connect to {cfg['label']}")
            connections.append(conn)

        # Set parameter ARMING_CHECK to 0 strictly in SITL simulation
        for conn in connections:
            conn.mav.param_set_send(
                conn.target_system, conn.target_component,
                b"ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
            )
        time.sleep(0.5)

        # 3. Wait for EKF origin lock
        print("\nWaiting for EKF origin and GPS lock on all 3 drones...")
        t_wait_start = time.time()
        ready_flags = [False, False, False]
        while time.time() - t_wait_start < 25.0 and not all(ready_flags):
            for i, conn in enumerate(connections):
                if not ready_flags[i]:
                    while True:
                        msg = conn.recv_match(blocking=False)
                        if not msg:
                            break
                        if msg.get_type() == "STATUSTEXT" and "origin set" in msg.text.lower():
                            print(f"  [SITL {drone_configs[i]['label']}] EKF origin set!")
                            ready_flags[i] = True
                            break
            time.sleep(0.1)

        time.sleep(2.0)

        # 4. Set GUIDED mode, arm, and takeoff
        print("\nSetting GUIDED mode, arming, and commanding takeoff on all 3 drones...")
        for i, conn in enumerate(connections):
            conn.set_mode(4)  # GUIDED
            time.sleep(0.3)

            is_armed = False
            t_arm_start = time.time()
            while time.time() - t_arm_start < 25.0:
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0
                )
                t_poll = time.time()
                while time.time() - t_poll < 0.5:
                    m = conn.recv_match(type="HEARTBEAT", blocking=False)
                    if m and (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        is_armed = True
                        print(f"  [{drone_configs[i]['label']}] ARMED!")
                        break
                if is_armed:
                    break
                time.sleep(0.3)

            if not is_armed:
                raise RuntimeError(f"Failed to arm {drone_configs[i]['label']}")

            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, 5.0
            )

        # 5. Monitor climb to 5.0m hover
        print("Monitoring takeoff climb to 5.0m...")
        t_climb = time.time()
        hover_flags = [False, False, False]
        while time.time() - t_climb < 25.0 and not all(hover_flags):
            for i, conn in enumerate(connections):
                if not hover_flags[i]:
                    latest_m = None
                    while True:
                        m = conn.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=False)
                        if not m:
                            break
                        latest_m = m
                    if latest_m:
                        alt = (
                            latest_m.relative_alt / 1000.0
                            if latest_m.get_type() == "GLOBAL_POSITION_INT"
                            else -latest_m.z
                        )
                        vz = latest_m.vz / 100.0 if latest_m.get_type() == "GLOBAL_POSITION_INT" else latest_m.vz
                        if alt >= 4.5 and abs(vz) < 0.35:
                            hover_flags[i] = True
                            print(f"  [{drone_configs[i]['label']}] In steady hover at {alt:.2f}m!")
            time.sleep(0.1)

        time.sleep(2.0)

        # 6. Compute True EKF Origin per drone: origin = global_ned - local_ned (Item 17)
        frame_origins = []
        for i, conn in enumerate(connections):
            g_msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
            l_msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
            if g_msg and l_msg:
                g_ned = frame.gps_to_global_ned(g_msg.lat / 1e7, g_msg.lon / 1e7, g_msg.relative_alt / 1000.0)
                l_ned = np.array([l_msg.x, l_msg.y, l_msg.z])
                origin = g_ned - l_ned
                frame_origins.append(origin)
                print(f"  [{drone_configs[i]['label']}] Computed EKF Frame Origin: N={origin[0]:.2f}m, E={origin[1]:.2f}m")
            else:
                frame_origins.append(np.array([0.0, 5.0 * i, 0.0]))

        # Unified V-formation geometry (Item 14)
        v_offsets = get_unified_v_geometry(spacing=spacing)

        # Assemble Swarm into Initial V-Formation Slots
        print("\n>> Assembling swarm into initial V-formation slots...")
        t_assemble_start = time.time()
        while time.time() - t_assemble_start < 4.0:
            for i, conn in enumerate(connections):
                tgt_global = np.array([v_offsets[i, 0], v_offsets[i, 1], -5.0])
                tgt_local = tgt_global - frame_origins[i]
                conn.mav.set_position_target_local_ned_send(
                    0, conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111111000,
                    tgt_local[0], tgt_local[1], tgt_local[2],
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            time.sleep(0.1)
        print(">> Swarm established in initial V-formation!")

        # 7. Execute 10.0s Trajectory Scenario: Translate Swarm 8.0m North
        print("\n>> EXECUTING 10.0s 3-DRONE V-FORMATION SCENARIO...")
        sitl_records = []
        n_steps = int(duration / dt)
        t_scenario_start = time.time()

        # Type mask: 0x0DC0 (Pos + Vel feedforward) vs 0x0DF8 (Pos only) (Item 14)
        type_mask = 0x0DC0 if use_velocity_feedforward else 0b0000111111111000

        for step in range(n_steps):
            t_sim = step * dt
            t_target_clock = t_scenario_start + t_sim

            if t_sim <= 8.0:
                c_x = 1.0 * t_sim
                c_vx = 1.0
            else:
                c_x = 8.0
                c_vx = 0.0
            c_y = 0.0
            c_vy = 0.0

            # Dispatch setpoints to all 3 drones
            for i, conn in enumerate(connections):
                tgt_global = np.array([c_x + v_offsets[i, 0], c_y + v_offsets[i, 1], -5.0])
                tgt_local = tgt_global - frame_origins[i]

                conn.mav.set_position_target_local_ned_send(
                    0, conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(tgt_local[0]), float(tgt_local[1]), float(tgt_local[2]),
                    float(c_vx), float(c_vy), 0.0,
                    0, 0, 0, 0, 0, 0, 0, 0
                )

            # Ingest telemetry from all 3 drones
            for i, conn in enumerate(connections):
                latest_m = None
                while True:
                    m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                    if not m:
                        break
                    latest_m = m

                if latest_m:
                    lat = latest_m.lat / 1e7
                    lon = latest_m.lon / 1e7
                    rel_alt = latest_m.relative_alt / 1000.0
                    cur_global_ned = frame.gps_to_global_ned(lat, lon, rel_alt)
                    cur_vx = latest_m.vx / 100.0
                    cur_vy = latest_m.vy / 100.0
                else:
                    # REPLACE TARGET-FILLED MISSING FRAMES WITH NaN (Item 14)
                    cur_global_ned = np.array([np.nan, np.nan, np.nan])
                    cur_vx, cur_vy = np.nan, np.nan
                    missing_frame_counts[i] += 1

                sitl_records.append({
                    "time": round(t_sim, 2),
                    "drone_id": i,
                    "sitl_x": cur_global_ned[0],
                    "sitl_y": cur_global_ned[1],
                    "sitl_vx": cur_vx,
                    "sitl_vy": cur_vy,
                })

            time_to_sleep = t_target_clock + dt - time.time()
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

        print(f"Recorded {len(sitl_records)} synchronized telemetry frames.")
        print(f"Missing Frames Recorded: {missing_frame_counts} (Total={sum(missing_frame_counts.values())})")
        return pd.DataFrame(sitl_records), missing_frame_counts

    finally:
        print("\nShutting down SITL processes...")
        for p in procs:
            p.terminate()
            try:
                p.wait(timeout=1.5)
            except Exception:
                p.kill()
        subprocess.run(["pkill", "-9", "-f", "arducopter"], stderr=subprocess.DEVNULL)


def generate_comparison_plots(merged_df: pd.DataFrame, output_dir: str):
    """
    Generates publication 4-panel comparison plots including signed North and East errors vs time.
    """
    fig, axes = plt.subplots(2, 2, figsize=(16, 12), dpi=180)

    drone_colors = ["#1f77b4", "#2ca02c", "#d62728"]
    drone_labels = ["Drone 0 (Apex)", "Drone 1 (Left Wing)", "Drone 2 (Right Wing)"]

    # 1. 2D Spatial Trajectory Overlay (X vs Y)
    ax1 = axes[0, 0]
    for i in range(3):
        sub = merged_df[merged_df["drone_id"] == i]
        ax1.plot(sub["sim_y"], sub["sim_x"], linestyle="--", linewidth=2.0, color=drone_colors[i],
                 label=f"{drone_labels[i]} (swarm_core)")
        ax1.plot(sub["sitl_y"], sub["sitl_x"], linestyle="-", linewidth=2.2, color=drone_colors[i], alpha=0.85,
                 label=f"{drone_labels[i]} (ArduPilot SITL)")

    ax1.set_xlabel("East Position Y (m)", fontweight="bold")
    ax1.set_ylabel("North Position X (m)", fontweight="bold")
    ax1.set_title("A: 3-Drone V-Formation 2D Trajectory Overlay\n(swarm_core vs ArduPilot SITL)", fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="lower right", fontsize=8)
    ax1.axis("equal")

    # 2. Euclidean Tracking Difference vs Time
    ax2 = axes[0, 1]
    for i in range(3):
        sub = merged_df[merged_df["drone_id"] == i]
        errors = np.sqrt((sub["sim_x"] - sub["sitl_x"])**2 + (sub["sim_y"] - sub["sitl_y"])**2)
        ax2.plot(sub["time"], errors, linewidth=2.0, color=drone_colors[i], label=f"{drone_labels[i]} Error")

    ax2.set_xlabel("Scenario Time (s)", fontweight="bold")
    ax2.set_ylabel("Position Difference ||sim - sitl|| (m)", fontweight="bold")
    ax2.set_title("B: Euclidean Trajectory Discrepancy Over Time", fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper left")

    # 3. Signed North Error vs Time (Item 14)
    ax3 = axes[1, 0]
    for i in range(3):
        sub = merged_df[merged_df["drone_id"] == i]
        err_north = sub["sitl_x"] - sub["sim_x"]
        ax3.plot(sub["time"], err_north, linewidth=2.0, color=drone_colors[i], label=f"{drone_labels[i]} (SITL - Sim)")

    ax3.axhline(0.0, color="black", linestyle=":", linewidth=1.2)
    ax3.set_xlabel("Scenario Time (s)", fontweight="bold")
    ax3.set_ylabel("Signed North Error X_sitl - X_sim (m)", fontweight="bold")
    ax3.set_title("C: Signed North Error Over Time (Lead/Lag Assessment)", fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right")

    # 4. Signed East Error vs Time (Item 14)
    ax4 = axes[1, 1]
    for i in range(3):
        sub = merged_df[merged_df["drone_id"] == i]
        err_east = sub["sitl_y"] - sub["sim_y"]
        ax4.plot(sub["time"], err_east, linewidth=2.0, color=drone_colors[i], label=f"{drone_labels[i]} (SITL - Sim)")

    ax4.axhline(0.0, color="black", linestyle=":", linewidth=1.2)
    ax4.set_xlabel("Scenario Time (s)", fontweight="bold")
    ax4.set_ylabel("Signed East Error Y_sitl - Y_sim (m)", fontweight="bold")
    ax4.set_title("D: Signed East Error Over Time (Cross-Track Deviation)", fontweight="bold")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend(loc="upper right")

    plt.tight_layout()
    plot_path = os.path.join(output_dir, "swarm_core_vs_sitl_overlay.png")
    plt.savefig(plot_path)
    plt.close(fig)

    print(f"Validation plot saved to: {plot_path}.")
    artifact_dir = Path("/home/drone/.gemini/antigravity/brain/28220ca6-e68a-487a-8a59-6e79ee58f6f6")
    if artifact_dir.exists():
        dest = artifact_dir / "swarm_core_vs_sitl_overlay.png"
        shutil.copyfile(plot_path, dest)
        print(f"Copied overlay artifact to: {dest}")


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)
    merged_csv_path = os.path.join(output_dir, "swarm_core_vs_sitl_3drones.csv")

    # 1. Simulate in pure swarm_core using CentralizedController directly
    print("Simulating 3-drone scenario in swarm_core with fitted parameters & unified geometry...")
    sim_df = simulate_swarm_core_3drone(duration=10.0, dt=0.1, tau=0.992, drag=0.637, spacing=3.0)

    # 2. Run SITL scenario or load cached comparison
    if "--sim-only" in sys.argv and os.path.exists(merged_csv_path):
        print(f"Loading cached multi-drone SITL comparison from: {merged_csv_path}")
        merged_df = pd.read_csv(merged_csv_path)
    else:
        sitl_bin = Path(os.environ.get("ARDUCOPTER_BIN", str(Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter")))
        if not sitl_bin.exists() and os.path.exists(merged_csv_path):
            print(f"[NOTE] SITL binary not found; using existing synchronized trajectory dataset at: {merged_csv_path}")
            merged_df = pd.read_csv(merged_csv_path)
            # Update sim columns with timestamps aligned
            merged_df["sim_x"] = sim_df["sim_x"].values
            merged_df["sim_y"] = sim_df["sim_y"].values
            merged_df["sim_vx"] = sim_df["sim_vx"].values
            merged_df["sim_vy"] = sim_df["sim_vy"].values
        else:
            sitl_df, missing_counts = run_sitl_3drone_scenario(
                duration=10.0,
                dt=0.1,
                spacing=3.0,
                use_velocity_feedforward=True,
            )
            merged_df = pd.merge(sim_df, sitl_df, on=["time", "drone_id"])

        merged_df["error_m"] = np.sqrt(
            (merged_df["sim_x"] - merged_df["sitl_x"])**2 +
            (merged_df["sim_y"] - merged_df["sitl_y"])**2
        )
        merged_df.to_csv(merged_csv_path, index=False)
        print(f"Saved synchronized multi-drone comparison CSV to: {merged_csv_path}")

    # 3. Calculate Trajectory RMS Difference
    print("\n=================================================================")
    print("  TASK C.2: TRAJECTORY VALIDATION METRICS (SIM VS SITL)          ")
    print("=================================================================")
    rms_per_drone = []
    for i in range(3):
        sub = merged_df[merged_df["drone_id"] == i]
        valid_errs = sub["error_m"].dropna()
        rms = float(np.sqrt(np.mean(valid_errs**2)))
        max_err = float(np.max(valid_errs))
        rms_per_drone.append(rms)
        print(f"  Drone {i}: Trajectory RMS Difference = {rms:.4f} m ({rms*100:.2f} cm) | Max Discrepancy = {max_err:.4f} m")

    overall_rms = float(np.sqrt(np.mean(merged_df["error_m"].dropna()**2)))
    print(f"  Overall Swarm Trajectory RMS Difference: {overall_rms:.4f} m ({overall_rms*100:.2f} cm)")
    print("=================================================================")

    # 4. Generate plots
    generate_comparison_plots(merged_df, output_dir)


if __name__ == "__main__":
    main()
