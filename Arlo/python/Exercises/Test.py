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

cmd_queue.put(("drive_n_cm_forward", 2, 250))

time.sleep(9999)