import time
import queue
import sys
import threading
sys.path.append("..")
import robot
from utils import should_stop
from motor_thread import MotorThread

SERIAL_LOCK = threading.Lock()

arlo = robot.Robot()

cmd_queue = queue.Queue()
motor = MotorThread(arlo, cmd_queue, serial_lock=SERIAL_LOCK)
motor.start()

while True:
    with SERIAL_LOCK:
        front_dist = arlo.read_front_ping_sensor()
        left_dist = arlo.read_left_ping_sensor()
        right_dist = arlo.read_right_ping_sensor()
    if should_stop(front_dist, left_dist, right_dist):
        if not motor.is_turning():
            motor.hard_stop()
            if left_dist != -1 and left_dist < 500:
                cmd_queue.put(("turn_n_degrees", 999))
            else:
                cmd_queue.put(("turn_n_degrees", -999))
    elif not motor.is_driving_forward():
        motor.hard_stop()
        cmd_queue.put(("drive_n_cm_forward", 0, 10000))

    time.sleep(0.01)
