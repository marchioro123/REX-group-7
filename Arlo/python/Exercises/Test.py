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


front_dist = arlo.read_front_ping_sensor()
cmd_queue.put(("turn_90_degrees", 0))
print(front_dist)

time.sleep(9999)
