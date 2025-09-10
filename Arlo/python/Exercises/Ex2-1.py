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
queue_empty = True

cmd_queue = queue.Queue()
motor = MotorThread(arlo, cmd_queue, serial_lock=SERIAL_LOCK)
motor.start()

cmd_queue.put(("drive_n_cm_forward", 0, 10000))


while True:
    if motor.is_busy():
        time.sleep(0.05)
        continue

    with SERIAL_LOCK:
        front_dist = arlo.read_front_ping_sensor()
        left_dist = arlo.read_left_ping_sensor()
        right_dist = arlo.read_right_ping_sensor()
    if should_stop(front_dist, left_dist, right_dist):
        motor.hard_stop()
        if left_dist != -1 and left_dist < 500:
            cmd_queue.put(("turn_90_degrees", 0, True))
        else:
            cmd_queue.put(("turn_90_degrees", 1, True))
        cmd_queue.put(("drive_n_cm_forward", 0, 10000))

    time.sleep(0.05)
