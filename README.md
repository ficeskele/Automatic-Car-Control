# Automatic-Car-Control

*(Raspberry Pi + Arduino differential-drive platform)*

---

## 📁 Repository structure

| File | Description |
|------|-------------|
| **`lane_following_single.py`** | Captures camera frames, tracks a single guide-line with HSV + contour vision, steers the robot via an incremental PID controller, detects 90° turns, and can trigger the LiDAR routine after a completed left turn. |
| **`lidar_avoidance.py`** | Self-contained LiDAR obstacle-avoidance loop: plots real-time point clouds, checks for obstacles directly in front, and issues turn/forward commands through serial. |
| **`func0527.py`** | Low-level helpers shared by both scripts (`send_motor`, `arduino_delay`, …). |
| **`requirements.txt`** | Python dependencies (`opencv-python`, `numpy`, `matplotlib`, `rplidar-python`, `pyserial`). |

---

## 🛠 Hardware / Wiring

| Module | Purpose | Notes |
|--------|---------|-------|
| **Raspberry Pi / Jetson / other SBC** | Runs the Python vision & LiDAR code. | USB camera on **/dev/video0**.<br>RPLidar on **/dev/ttyUSB0** *(adjust in code if needed)*. |
| **Arduino (motor driver)** | Receives simple `L<left>,R<right>` PWM commands over UART. | Default **/dev/ttyUSB1**, baud **57600**. |
| **USB camera** | Top-down view of the guide line. |
| **RPLidar A1/A2** | 360° laser scan for obstacle detection. | Powered via USB 5 V. |
| **Motor driver + DC motors** | Differential drive. | PWM range 0–255 assumed. |

**Serial map**

| Device | Path (default) | Baudrate |
|--------|----------------|----------|
| Arduino | `/dev/ttyUSB1` | 57600 |
| RPLidar | `/dev/ttyUSB0` | 115200 |

---
