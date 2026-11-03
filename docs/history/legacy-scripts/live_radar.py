"""
Live Real-Time Swarm Radar GUI for SITL Drones.
Connects to Drone 1 (UDP 14550) and Drone 2 (UDP 14560),
and opens an interactive GUI window displaying live positions, altitudes, trails, and inter-drone spacing.
"""

import sys
import time
from typing import Dict, List
import matplotlib
# Use TkAgg or Qt5Agg for interactive window display
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil


def main():
    print("=================================================================")
    print("  Launching Real-Time Swarm Radar GUI for SITL Drones")
    print("=================================================================")

    # Setup MAVLink connections
    print("Connecting to Drone 1 on udpin:127.0.0.1:14550...")
    c1 = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    print("Connecting to Drone 2 on udpin:127.0.0.1:14560...")
    c2 = mavutil.mavlink_connection("udpin:127.0.0.1:14560")

    # Request position streams at 10 Hz
    for conn in [c1, c2]:
        msg = conn.wait_heartbeat(timeout=5.0)
        if msg:
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,
                1,
            )

    # Initialize plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("Swarm Drones SITL: Real-Time Formation Radar")

    # Colors and styles
    ax.set_facecolor("#121212")
    fig.patch.set_facecolor("#1a1a1a")

    d1_trail_x, d1_trail_y = [], []
    d2_trail_x, d2_trail_y = [], []

    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([0.0, 3.0, 0.0])
    alt1, alt2 = 0.0, 0.0

    print("GUI window opened! Updating live display...")

    try:
        while plt.fignum_exists(fig.number):
            # Drain incoming packets
            while True:
                msg = c1.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                if not msg:
                    break
                p1 = np.array([msg.x, msg.y, msg.z])
                alt1 = -msg.z

            while True:
                msg = c2.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                if not msg:
                    break
                p2 = np.array([msg.x, msg.y, msg.z])
                alt2 = -msg.z

            # Update trails
            d1_trail_x.append(p1[1])  # East is horizontal (X on plot)
            d1_trail_y.append(p1[0])  # North is vertical (Y on plot)
            d2_trail_x.append(p2[1])
            d2_trail_y.append(p2[0])

            # Keep last 50 points
            if len(d1_trail_x) > 50:
                d1_trail_x.pop(0)
                d1_trail_y.pop(0)
                d2_trail_x.pop(0)
                d2_trail_y.pop(0)

            # Redraw
            ax.clear()
            ax.set_xlim(-6, 6)
            ax.set_ylim(-6, 6)
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", color="#333333", alpha=0.7)

            # Draw range rings from origin
            for r in [1, 2, 3, 4, 5]:
                circle = plt.Circle((0, 0), r, color="#2a2a2a", fill=False, linestyle=":")
                ax.add_patch(circle)

            # Draw trails
            if len(d1_trail_x) > 1:
                ax.plot(d1_trail_x, d1_trail_y, color="#00ffcc", alpha=0.4, linewidth=1.5)
            if len(d2_trail_x) > 1:
                ax.plot(d2_trail_x, d2_trail_y, color="#ffcc00", alpha=0.4, linewidth=1.5)

            # Draw line connecting drones
            dist = np.linalg.norm(p1 - p2)
            ax.plot([p1[1], p2[1]], [p1[0], p2[0]], color="#ffffff", linestyle="--", alpha=0.6, linewidth=1.2)
            mid_x = (p1[1] + p2[1]) / 2.0
            mid_y = (p1[0] + p2[0]) / 2.0
            ax.text(mid_x, mid_y + 0.3, f"{dist:.2f} m", color="white", fontsize=10, ha="center",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="#333333", alpha=0.8))

            # Draw Drone 1 (Lead)
            ax.scatter(p1[1], p1[0], color="#00ffcc", s=180, edgecolors="white", linewidth=1.5, zorder=5)
            ax.text(p1[1], p1[0] - 0.45, f"Drone 1 (Lead)\nAlt: {alt1:.1f}m", color="#00ffcc",
                    fontweight="bold", fontsize=9, ha="center")

            # Draw Drone 2 (Follower)
            ax.scatter(p2[1], p2[0], color="#ffcc00", s=180, edgecolors="white", linewidth=1.5, zorder=5)
            ax.text(p2[1], p2[0] - 0.45, f"Drone 2 (Follower)\nAlt: {alt2:.1f}m", color="#ffcc00",
                    fontweight="bold", fontsize=9, ha="center")

            # HUD header
            hud_text = (
                f"SITL Multi-Drone Telemetry\n"
                f"Status: IN FLIGHT | Mode: GUIDED\n"
                f"Formation: LINE (3.0m East offset)\n"
                f"Inter-Drone Spacing: {dist:.2f} m"
            )
            ax.text(0.03, 0.97, hud_text, transform=ax.transAxes, color="white",
                    verticalalignment="top", fontsize=9,
                    bbox=dict(boxstyle="round", facecolor="#222222", alpha=0.85))

            ax.set_title("Autonomous Swarm SITL Real-Time Radar", color="white", fontsize=12, fontweight="bold")
            ax.set_xlabel("East Offset (meters)", color="gray")
            ax.set_ylabel("North Offset (meters)", color="gray")
            ax.tick_params(colors="gray")

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nRadar GUI closed.")


if __name__ == "__main__":
    main()
