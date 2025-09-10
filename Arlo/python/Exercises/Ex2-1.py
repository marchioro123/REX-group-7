import time
import queue
import sys
sys.path.append("..")
import robot
from motor_thread import MotorThread

SAFE_DISTANCE = 500 

arlo = robot.Robot()
cmd_queue = queue.Queue()
result_queue = queue.Queue()

motor = MotorThread(arlo, cmd_queue)
motor.start()

cmd_queue.put(("drive_n_cm_forward", 0, 10000))
time.sleep(0.01)


while True:
    if cmd_queue.empty():
        # Ask MotorThread to read front sensor
        cmd_queue.put(("read_front_ping_sensor",))

        try:
            name, front_dist = result_queue.get(timeout=1)
            if name == "front_ping":
                print(f"Front distance: {front_dist} mm")

                if front_dist != -1 and front_dist < SAFE_DISTANCE:
                    # Stop immediately
                    cmd_queue.put(("hard_stop",))
                    time.sleep(0.1)

                    # Check left sensor
                    cmd_queue.put(("read_left_ping_sensor",))
                    left_name, left_dist = result_queue.get(timeout=1)

                    # Check right sensor
                    cmd_queue.put(("read_right_ping_sensor",))
                    right_name, right_dist = result_queue.get(timeout=1)

                    print(f"Left: {left_dist}, Right: {right_dist}")

                    # Decide where to go
                    if left_dist != -1 and left_dist > SAFE_DISTANCE:
                        print("Turning left...")
                        cmd_queue.put(("turn_90_degrees", 0))
                        time.sleep(0.5)
                        cmd_queue.put(("drive_n_cm_forward", 0, 10000))

                    elif right_dist != -1 and right_dist > SAFE_DISTANCE:
                        print("Turning right...")
                        cmd_queue.put(("turn_90_degrees", 1))
                        time.sleep(0.5)
                        cmd_queue.put(("drive_n_cm_forward", 0, 10000))

                    else:
                        print("Blocked in all directions")
                        break

        except queue.Empty:
            print("Sensor read timeout")

    time.sleep(0.01)

# while True:
    # if cmd_queue.empty():
        # front_dist = arlo.read_front_ping_sensor()
        # time.sleep(0.01)
        # if front_dist != -1 and front_dist < 500:
            # motor.hard_stop()

            # left_dist = arlo.read_left_ping_sensor()
            # time.sleep(0.01)
            # right_dist = arlo.read_right_ping_sensor()
            # time.sleep(0.01)

            # if left_dist != -1 and left_dist < 500:
            #     cmd_queue.put(("turn_90_degrees", 0))
            # elif right_dist != -1 and right_dist < 500:
            #     cmd_queue.put(("turn_90_degrees", 1))
            # else:
            #     cmd_queue.put(("turn_90_degrees", 1))
            # time.sleep(0.01)
            # cmd_queue.put(("drive_n_cm_forward", 0, 10000))
            # time.sleep(0.01)

    # time.sleep(0.01)
