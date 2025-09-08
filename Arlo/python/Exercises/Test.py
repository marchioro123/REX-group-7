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

cmd_queue.put(("turn_n_degrees_left", 360))

time.sleep(9999)