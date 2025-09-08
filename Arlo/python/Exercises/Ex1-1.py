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

for _ in range(4):
    cmd_queue.put(("drive_n_cm_forward", 0, 100))
    cmd_queue.put(("turn_90_degrees_left"))


# cmd_queue.put(("turn_n_degrees_left", 360))


time.sleep(9999)