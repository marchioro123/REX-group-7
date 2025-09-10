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

cmd_queue.put(("drive_n_cm_forward", 0, 10000))
while True:
    if cmd_queue.empty():
        front_dist = arlo.read_front_ping_sensor()
        time.sleep(0.01)
        if front_dist != -1 and front_dist < 500:
            motor.hard_stop()

            left_dist = arlo.read_left_ping_sensor()
            time.sleep(0.01)
            right_dist = arlo.read_right_ping_sensor()
            time.sleep(0.01)

            if left_dist != -1 and left_dist < 500:
                cmd_queue.put(("turn_90_degrees", 0))
            elif right_dist != -1 and right_dist < 500:
                cmd_queue.put(("turn_90_degrees", 1))
            else:
                cmd_queue.put(("turn_90_degrees", 1))
            cmd_queue.put(("drive_n_cm_forward", 0, 10000))

    time.sleep(0.01)
