from time import sleep, time
from collections import deque
import sys
sys.path.append("..")
import robot


arlo = robot.Robot()


def move_straight(l, speed = "slow"):
    if speed == "slow":
        print(arlo.go_diff(41, 40, 1, 1))
        duration = l*0.039
    if speed == "medium":
        print(arlo.go_diff(86, 83, 1, 1))
        duration = l*0.0162
    if speed == "fast":
        print(arlo.go_diff(127, 115, 1, 1))
        duration = l*0.0105

    wait_till = time() + duration
    return wait_till

def rotate_left(alpha):
    # computes time to rotate, assuming 90 degrees = 0.37 second
    duration = 0.37

    print(arlo.go_diff(105, 100, 0, 1))
    wait_till = time() + duration
    return wait_till

def rotate_right(alpha):
    # computes time to rotate, assuming 90 degrees = 0.73 second
    duration = alpha/90*0.73

    print(arlo.go_diff(68, 64, 0, 1))
    wait_till = time() + duration
    return wait_till

que = deque()
#que.extend([(move_straight, 50), (rotate_left, 90)]*4)

que.extend([(move_straight, 50)])


wait_till = time()
while que:
    next_command, parameter = que.popleft()
    wait_till = next_command(parameter)
    while time() < wait_till:
        front_dist = arlo.read_front_ping_sensor()
        if front_dist != -1 and front_dist < 500:
            print(front_dist)
            arlo.stop()
            input()
        sleep(0.01)

