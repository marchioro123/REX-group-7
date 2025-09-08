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


time.sleep(999)