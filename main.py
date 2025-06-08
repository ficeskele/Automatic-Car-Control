import time
import cv2
import serial
import numpy as np  # still required by process_frame_single_line_gpt

from function import (
    send_motor,
    PIDLaneCentering,
    process_frame_single_line,
    map_correction_to_pwm,
    arduino_delay,
)
from lidar_part import lidar_avoidance  # Post-left-turn flow (stage 3)

# ------------------------- Hardware connection ------------------------- #
SERIAL_PORT = "/dev/ttyUSB1"
BAUDRATE = 57600

print(f"[INFO] Connecting to Arduino on {SERIAL_PORT} @ {BAUDRATE} ...")
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
time.sleep(2)  # Allow time for Arduino to reboot
print("[INFO] Arduino ready.")

# ------------------------- Video source ------------------------- #
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera 0")

# ------------------------- PID controller ------------------------- #
pid = PIDLaneCentering(k=[0.3, 0.002, 0.85], output_limits=(-25, 25))
last_offset: float = 0.0

# ------------------------- Loop flags ------------------------- #
#   state:      current road condition flag returned by CV model
#   last_state: previous iteration flag
#   finished:   set True after completing a LEFT_TURN to exit main loop
state = 0
last_state = 0
finished = False

# ------------------------- Main control loop ------------------------- #
while True:
    # Grab a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("[WARN] Frame grab failed, exiting ...")
        break

    h, w = frame.shape[:2]

    # Detect lane offset and current state
    last_state = state
    last_offset, state = process_frame_single_line(frame, last_offset)

    print("=" * 48)
    print(f"Prev state : {last_state}")
    print(f"Curr state : {state}")
    print(f"Offset     : {last_offset:+.2f}")
    print("=" * 48)

    # --------------------- LEFT-TURN sequence --------------------- #
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

        finished = True             # trigger stage-3 flow
        print("[INFO] LEFT TURN sequence complete.")

    # --------------------- RIGHT-TURN sequence --------------------- #
    elif state == "RIGHT_TURN" and last_state is False:
        print("[INFO] Detected RIGHT_TURN pattern.")

        send_motor(ser, 50, 50)     # short acceleration
        arduino_delay(ser, 700)

        send_motor(ser, 70, 0)      # pivot right
        arduino_delay(ser, 1000)

        last_state = state          # avoid re-trigger
        last_offset = -300          # force left bias to reacquire lane
        print("[INFO] RIGHT TURN sequence complete.")

    # ------------------ Drift-to-the-right compensation ------------------ #
    elif state is True and last_offset > 0:
        print("[INFO] Adjusting right drift ...")
        # Keep turning left until state clears (no longer TRUE)
        while state is True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame grab failed during drift adjust.")
                break

            send_motor(ser, 40, 0)  # left wheel forward, right wheel stop
            last_offset, state = process_frame_single_line(frame, last_offset)
        continue  # restart main loop after correction

    # ------------------ Standard PID lane following ------------------ #
    if not finished:
        correction = pid.compute(last_offset)
        left_pwm, right_pwm = map_correction_to_pwm(
            correction,
            40,   # cruise speed
            0.78  # turning ratio
        )
        print(f"[CTRL] PWM → L:{left_pwm:+d}  R:{right_pwm:+d}")
        send_motor(ser, left_pwm, right_pwm)
    else:
        # Exit: stop motors and break loop
        send_motor(ser, 0, 0)
        break

    # ------------------ Debug preview window ------------------ #
    cv2.line(frame, (w // 2, h), (0, h // 2), (0, 255, 0), 2)
    cv2.imshow("lane", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        print("[INFO] Quit signal received.")
        send_motor(ser, 0, 0)
        break

# ------------------------- Cleanup ------------------------- #
if finished:
    print("[INFO] Entering third_part flow ...")
    lidar_avoidance(ser)

cap.release()
cv2.destroyAllWindows()
ser.close()
print("[INFO] Program terminated gracefully.")