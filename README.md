# Automatic-Car-Control

Line-Following Robot with LiDAR Obstacle Avoidance

(Raspberry Pi + Arduino differential-drive platform)

⸻

📁  Repository structure

File	Description
lane_following_single.py	Main program  →  Captures camera frames, tracks a single guide-line with HSV + contour vision, steers the robot with an incremental PID controller, detects 90° turns, and (optionally) triggers the LiDAR routine after a completed left turn.
lidar_avoidance.py	Self-contained LiDAR obstacle-avoidance loop: plots real-time point-clouds, checks for obstacles directly in front, issues turn / forward commands through serial.
func0527.py (existing)	Low-level helpers shared by both scripts (send_motor, arduino_delay, etc.).
requirements.txt (suggested)	Python dependencies (opencv-python, numpy, matplotlib, rplidar-python, pyserial).


⸻

🛠  Hardware / Wiring

Module	Purpose	Notes
Raspberry Pi / Jetson / Linux SBC	Runs the Python vision & LiDAR code.	Connect USB camera on /dev/video0.Connect RPLidar on /dev/ttyUSB0 (or adjust in code).
Arduino (motor driver)	Receives simple “L<left>,R<right>” PWM commands over UART.	Default port /dev/ttyUSB1, baud 57600.
USB camera	Top-down view of the guide-line.	
RPLidar A1/A2	360° laser scan for obstacle detection.	Powered via USB (5 V).
Motor driver + DC motors	Differential drive.	PWM range 0–255 assumed.

Serial map — adjust in scripts if your ports differ:

Device	Example path	Baudrate
Arduino	/dev/ttyUSB1	57600
RPLidar	/dev/ttyUSB0	115200


⸻

🔧  Installation

# 1. Clone repo & install Python deps
git clone https://github.com/<you>/line-follow-lidar.git
cd line-follow-lidar
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt  # or install packages manually

# 2. Enable camera (Pi) and verify serial devices exist


⸻

▶️  Running the programs

1. Line-following (camera + PID)

python lane_following_single.py

	•	Opens a preview window (q to quit).
	•	Green dot / red line show the detected centroid & vehicle mid-line.
	•	LEFT_TURN / RIGHT_TURN messages appear when a 90° horizontal guide-line is seen.
	•	After finishing a left turn the script calls third_part() (can be swapped for calling lidar_avoidance.py if you keep them separate).

2. LiDAR obstacle avoidance (stand-alone)

python lidar_avoidance.py

	•	Live scatter plot of the current 360° scan.
	•	When a point satisfies |y| < 0.05 m and x < 0.30 m it is treated as an obstacle ahead.
	•	Robot alternates its bypass direction (left → right → …) each time a new obstacle is encountered.
	•	Ctrl + C leaves the loop gracefully.

Tip – If you want the line-follower to fall back to LiDAR automatically, import
lidar_avoidance.third_part in lane_following_single.py and call it instead of the inlined version.

⸻

⚙️  Key parameters

Section	Variable	Default	What it does
PID	k=[0.3, 0.002, 0.85]	Tune Kp / Ki / Kd for smoother tracking.	
PWM mapping	base_pwm=40, scale=0.78	Base forward speed & how aggressively steering PWM is scaled.	
LiDAR filter	y_thresh=0.05, x_min=0.30	Defines the “front corridor” for obstacle detection.	

Adjust these constants in the scripts rather than rewiring logic.

⸻

🧩  Extending
	•	Add a .gitignore (e.g. Python template) and a LICENSE.
	•	Replace the simple front corridor detector with clustering or corridor mapping.
	•	Publish IMU data to fuse heading and create curved trajectories instead of in-place pivots.
	•	Integrate ROS 2 nodes if you need distributed control and RViz visualisation.

⸻

🙏  Credits & License
	•	RPLidar Python SDK – © Slamtec
	•	Example PID / lane-tracking logic inspired by many DIY line-follower projects.

Project code © 2025 Your Name. Licensed under the MIT License (see LICENSE).