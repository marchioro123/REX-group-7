import time
import queue
import sys
sys.path.append("..")
import robot
from motor_thread import MotorThread

arlo = robot.Robot()

print("Running ...")

cmd_queue = queue.Queue()
motor = MotorThread(arlo, cmd_queue)
motor.start()

for _ in range(4):
    cmd_queue.put(("drive_n_cm_forward", 100))
    cmd_queue.put(("turn_n_degrees_left", 90))


time.sleep(5)