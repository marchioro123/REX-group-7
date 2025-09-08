import time
import queue
import sys
sys.path.append("..")
import robot
from motor_thread import MotorThread

arlo = robot.Robot()

cmd_queue = queue.Queue()
motor = MotorThread(arlo, cmd_queue)
motor.start()

while True:
    front_dist = arlo.read_front_ping_sensor()
    if (front_dist == -1 or front_dist < 500):
        cmd_queue.put(("drive_n_cm_forward", 0, 5))
    else:
        left_dist = arlo.read_front_ping_sensor()
        right_dist = arlo.read_front_ping_sensor()

        if not (left_dist == -1 or left_dist < 500):
            cmd_queue.put(("turn_90_degrees_left", 0))
        elif not (right_dist == -1 or right_dist < 500):
            cmd_queue.put(("turn_n_degrees_right", 90))
        else:
            cmd_queue.put(("turn_n_degrees_right", 180))

    time.sleep(0.01)
