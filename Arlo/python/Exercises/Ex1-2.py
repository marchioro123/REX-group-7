import time
import queue
import sys
import threading
sys.path.append("..")
import robot
from motor_thread import MotorThread
SERIAL_LOCK = threading.Lock()

arlo = robot.Robot()

cmd_queue = queue.Queue()
motor = MotorThread(arlo, cmd_queue, serial_lock=SERIAL_LOCK)
motor.start()

for _ in range(72):
    cmd_queue.put(("turn_n_degrees", 5))
    time.sleep(2)

time.sleep(999)