import time
import queue
import sys
import threading
sys.path.append("..")
import robot
from motor_thread import MotorThread

arlo = robot.Robot()
SERIAL_LOCK = threading.Lock()

cmd_queue = queue.Queue()
motor = MotorThread(arlo, cmd_queue, serial_lock=SERIAL_LOCK)
motor.start()


cmd_queue.put(("turn_n_degrees", 90))


time.sleep(9999)

# hello
