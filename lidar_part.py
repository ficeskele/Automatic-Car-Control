"""
lidar_avoidance.py
===================

LiDAR-based obstacle-avoidance routine for a differential-drive robot.
The script exposes:

* **polar_to_xy** – convert (angle°, distance mm) → (x, y) in metres
* **process_scan** – transform one LiDAR scan to X/Y point clouds
* **plot_points** – render a single scan on a Matplotlib axis
* **detect_front** – simple heuristic to check if something is directly ahead
* **third_part** – main loop: read scans → plot → detect → send wheel commands

Implementation preserves original behaviour while cleaning up style and
translating comments to English. No application logic has been modified.
"""

from __future__ import annotations

import math
import time
from typing import List, Tuple

import matplotlib.pyplot as plt
from rplidar import RPLidar, RPLidarException

from func0527 import send_motor, arduino_delay

# ---------------------------------------------------------------------------
# Helpers – geometry & preprocessing
# ---------------------------------------------------------------------------

def polar_to_xy(angle_deg: float, dist_mm: float) -> Tuple[float, float]:
    """Convert polar coordinates to Cartesian metres.

    Args:
        angle_deg: Bearing reported by the LiDAR [0–360).
        dist_mm:   Range to the target point in millimetres.

    Returns:
        (x, y) in metres, where ``x = r·cosθ`` and ``y = r·sinθ``.
    """
    r = dist_mm / 1000.0
    rad = math.radians(angle_deg)
    return r * math.cos(rad), r * math.sin(rad)


def process_scan(scan) -> Tuple[List[float], List[float]]:
    """Convert one LiDAR scan to X/Y point lists (metres)."""
    xs, ys = [], []
    for _quality, angle, dist in scan:
        if dist > 0:
            x, y = polar_to_xy(angle, dist)
            xs.append(x)
            ys.append(y)
    return xs, ys

# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def plot_points(ax, xs: List[float], ys: List[float]) -> None:
    """Scatter-plot the current LiDAR point cloud on *ax*."""
    ax.clear()
    ax.scatter(xs, ys, s=5, c="k")
    ax.set_title("LiDAR point cloud (single scan)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.grid(True, linestyle="--", alpha=0.3)

# ---------------------------------------------------------------------------
# Simple front-obstacle detector
# ---------------------------------------------------------------------------

def detect_front(
    xs: List[float],
    ys: List[float],
    y_thresh: float = 0.05,
    x_min: float = 0.3,
) -> bool:
    """Return *True* if any point lies directly in front of the robot.

    A point is considered *in front* when ``|y| < y_thresh`` *and* ``x < x_min``.
    """
    for x, y in zip(xs, ys):
        if x < x_min and abs(y) < y_thresh:
            print(">>> Obstacle detected ahead!")
            return True
    return False

# ---------------------------------------------------------------------------
# Main routine – obstacle avoidance based on LiDAR scans
# ---------------------------------------------------------------------------

def lidar_avoidance(
    ser,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 115200,
) -> None:
    """Continuous LiDAR scan / plot / avoidance loop (Ctrl+C to exit)."""

    try:
        lidar = RPLidar(port, baudrate=baudrate, timeout=1)
    except RPLidarException as e:
        print(f">>> Cannot connect to LiDAR: {e}")
        return

    print(">>> LiDAR connected – press Ctrl+C to stop")

    turn_direction = "left"  # preferred bypass direction
    last_obstacle = False
    obstacle = False

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))

    scans = lidar.iter_scans(max_buf_meas=200)

    try:
        while True:
            # Try to fetch next full rotation
            try:
                scan = next(scans)
            except RPLidarException:
                # Transient read error – simply retry on next loop
                continue
            except StopIteration:
                # Iterator exhausted; attempt a soft reconnect
                print(">>> LiDAR iterator ended – reconnecting …")
                try:
                    lidar.stop()
                    lidar.disconnect()
                    time.sleep(0.5)
                    lidar = RPLidar(port, baudrate=baudrate, timeout=1)
                    scans = lidar.iter_scans(max_buf_meas=200)
                    print(">>> Reconnected – resuming scans")
                    continue
                except RPLidarException as e:
                    print(f">>> Reconnect failed: {e}")
                    break

            # 1. Convert scan to Cartesian points
            xs, ys = process_scan(scan)

            # 2. Visualise
            plot_points(ax, xs, ys)

            last_obstacle = obstacle
            obstacle = detect_front(xs, ys)

            print("-" * 47)
            print(f" NEXT turn: {turn_direction}")
            print("-" * 47)

            if obstacle and not last_obstacle:
                # Decide turn direction and execute avoidance manoeuvre
                if turn_direction == "left":
                    print(">>> TURN LEFT")
                    turn_direction = "right"
                    send_motor(ser, 0, 70)  # rotate CCW
                else:
                    print(">>> TURN RIGHT")
                    turn_direction = "left"
                    send_motor(ser, 70, 0)  # rotate CW

                arduino_delay(ser, 1000)
                send_motor(ser, 0, 0)  # stop after turn

            else:
                print(">>> MOVING FORWARD")
                send_motor(ser, 40, 40)

            plt.pause(0.05)

    except KeyboardInterrupt:
        print("\n>>> User interrupt – stopping scans")
    finally:
        # Graceful shutdown
        lidar.stop()
        lidar.disconnect()
        plt.ioff()
        plt.close()
        print(">>> LiDAR connection closed – routine ended")
