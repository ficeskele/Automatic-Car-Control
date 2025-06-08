"""
lane_following_single.py
========================

Refactored lane-following script consolidated into **one file** while preserving
original logic. The script contains:

1. Arduino I/O helpers (serial delay, motor PWM)
2. Incremental PID controller for lane centering
3. Frame-processing routine for single-line detection (top-down view)
4. Main control loop (camera → CV → PID → motor)

All comments are translated to English, dead code removed, and log messages are
structured for easier debugging.
"""

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

import time
import cv2
import serial
import numpy as np
from lidar_part import lidar_avoidance  # Stage-3 routine executed after a left turn

# ---------------------------------------------------------------------------
# Arduino I/O helpers
# ---------------------------------------------------------------------------

def arduino_delay(ser, ms: int) -> None:
    """Request Arduino to wait *ms* milliseconds via serial command ``W,<ms>``."""
    ser.write(f"W,{ms}\n".encode())


def send_motor(ser, left_pwm: int, right_pwm: int) -> None:
    """Send differential-drive PWM command: ``L<left>,R<right>``."""
    ser.write(f"L{int(left_pwm)},R{int(right_pwm)}\n".encode())

# ---------------------------------------------------------------------------
# PID lane-centering controller
# ---------------------------------------------------------------------------

class PIDLaneCentering:
    """Incremental PID controller for keeping the robot centred on the lane."""

    def __init__(self, target: float = 0.0, k=(1.0, 0.0, 0.0), output_limits=(-100, 100)):
        self.kp, self.ki, self.kd = k
        self.target = target
        self.lower, self.upper = output_limits

        self.error = 0.0
        self.prev_error = 0.0
        self.sum_error = 0.0

    def compute(self, offset: float) -> float:
        """Return corrective value given the current *offset*."""
        self.error = self.target - offset

        p = self.kp * self.error
        i = self.ki * self.sum_error
        d = self.kd * (self.error - self.prev_error)

        output = max(self.lower, min(self.upper, p + i + d))

        self.prev_error = self.error
        self.sum_error += self.error
        return output

# ---------------------------------------------------------------------------
# Helper: convert PID correction to motor PWM pair
# ---------------------------------------------------------------------------

def map_correction_to_pwm(correction: float, base_pwm: int, scale: float):
    """Convert *correction* to (left_pwm, right_pwm) bounded to 0–255."""
    delta = int(scale * correction)
    left  = int(np.clip(base_pwm - delta, 0, 255))
    right = int(np.clip(base_pwm + delta, 0, 255))
    return left, right

# ---------------------------------------------------------------------------
# Computer-vision: single-line processing
# ---------------------------------------------------------------------------

def process_frame_single_line(frame, last_offset: float):
    """Return (offset, flag) for a top-down single-line lane-following view.

    * offset  > 0 → line is to the right → steer right
    * offset  < 0 → line is to the left  → steer left
    * flag is either ``False`` (normal), ``True`` (drift adjust),
      ``"LEFT_TURN"``, or ``"RIGHT_TURN"`` according to heuristics.
    """
    h, w = frame.shape[:2]

    # 1. Binarise using HSV mask (assumes dark line on lighter ground)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))

    # 2. Morphological cleanup: close holes then remove noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

    # 3. Detect horizontal lines via HoughP to infer sharp turns
    edges = cv2.Canny(mask, 100, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                            minLineLength=50, maxLineGap=50)
    if lines is not None:
        left_th  = int(w * 0.1)   # <10 % from left border → treat as left edge
        right_th = int(w * 0.9)   # >90 % from left border → treat as right edge
        for x1, y1, x2, y2 in lines[:, 0]:
            if abs(y2 - y1) < 20:  # near-horizontal
                if x1 < left_th or x2 < left_th:
                    return last_offset, "LEFT_TURN"
                if x1 > right_th or x2 > right_th:
                    return last_offset, "RIGHT_TURN"

    # 4. Use contour centroid for normal tracking
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        # No line detected: keep previous offset, signal drift adjustment
        return last_offset, True

    cnt = max(contours, key=cv2.contourArea)
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return last_offset, False

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    offset = cx - (w // 2)

    # Debug overlay: centroid and centre line
    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)

    return offset, False

# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # --------------------------- Hardware connection ---------------------- #
    SERIAL_PORT = "/dev/ttyUSB1"
    BAUDRATE = 57600

    print(f"[INFO] Connecting to Arduino on {SERIAL_PORT} @ {BAUDRATE} …")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    time.sleep(2)  # allow Arduino to reboot
    print("[INFO] Arduino ready.")

    # ----------------------------- Video source -------------------------- #
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera 0")

    # --------------------------- PID initialisation ---------------------- #
    pid = PIDLaneCentering(k=[0.3, 0.002, 0.85], output_limits=(-25, 25))
    last_offset: float = 0.0

    # ------------------------------ Loop flags -------------------------- #
    state = 0      # current state returned by CV model
    last_state = 0 # previous iteration state
    finished = False

    # ------------------------------- Main loop -------------------------- #
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Frame grab failed, exiting …")
            break

        h, w = frame.shape[:2]
        last_state = state
        last_offset, state = process_frame_single_line(frame, last_offset)

        print("=" * 48)
        print(f"Prev state : {last_state}")
        print(f"Curr state : {state}")
        print(f"Offset     : {last_offset:+.2f}")
        print("=" * 48)

        # --------------------------- LEFT-TURN flow ---------------------- #
        if state == "LEFT_TURN" and last_state is False:
            print("[INFO] Detected LEFT_TURN pattern.")

            send_motor(ser, -30, -30)   # short brake
            arduino_delay(ser, 700)

            send_motor(ser, -30, 70)    # pivot left
            arduino_delay(ser, 1000)

            send_motor(ser, 50, 50)     # drive out of the corner
            arduino_delay(ser, 4000)

            send_motor(ser, 0, 0)       # full stop
            arduino_delay(ser, 500)

            finished = True            
            print("[INFO] LEFT TURN sequence complete.")

        # --------------------------- RIGHT-TURN flow --------------------- #
        elif state == "RIGHT_TURN" and last_state is False:
            print("[INFO] Detected RIGHT_TURN pattern.")

            send_motor(ser, 50, 50)     # short acceleration
            arduino_delay(ser, 700)

            send_motor(ser, 70, 0)      # pivot right
            arduino_delay(ser, 1000)

            last_state = state          # avoid re-trigger
            last_offset = -300          # force left bias to reacquire lane
            print("[INFO] RIGHT TURN sequence complete.")

        # --------------- Drift-to-the-right compensation ----------------- #
        elif state is True and last_offset > 0:
            print("[INFO] Adjusting right drift …")
            while state is True:
                ret, frame