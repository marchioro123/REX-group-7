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

def rotate_left(alpha, should_sleep = False):
    # computes time to rotate, assuming 90 degrees = 0.37 second
    duration = 0.37/90*alpha

    print(arlo.go_diff(105, 100, 0, 1))
    if should_sleep == True:
        sleep(duration)
    else:
        wait_till = time() + duration
    return wait_till

def rotate_right(alpha, should_sleep = False):
    # !TODO check constant
    duration = 0.37/90*alpha

    print(arlo.go_diff(105, 100, 0, 1))
    if should_sleep == True:
        sleep(duration)
    else:
        wait_till = time() + duration
    return wait_till

def is_too_close(distance, sensitivity):
    return (distance != -1 and distance < sensitivity)

que = deque()
#que.extend([(move_straight, 50), (rotate_left, 90)]*4)

que.extend([(move_straight, 100)])


wait_till = time()
check_sensors = True
while que:
    next_command, parameter = que.popleft()
    if next_command == move_straight:
        check_sensors = True
    else:
        check_sensors = False
    wait_till = next_command(parameter)
    while time() < wait_till:
        front_dist = arlo.read_front_ping_sensor()
        front_is_close = is_too_close(front_dist, 500)
        left_dist = arlo.read_left_ping_sensor()
        left_is_close = is_too_close(left_dist, 500)
        left_is_very_close = is_too_close(left_dist, 50)
        right_dist = arlo.read_right_ping_sensor()
        right_is_close = is_too_close(right_dist, 500)
        right_is_very_close = is_too_close(right_dist, 50)
        if (front_is_close or left_is_very_close or right_is_very_close):
            print("turning!")
            print(front_dist, left_dist, right_dist)
            if front_is_close:
                if left_is_close and not right_is_close:
                    rotate_right(60, True)
                elif not left_is_close and right_is_close:
                    rotate_left(60, True)
                else:
                    rotate_left(120, True)
            elif left_is_very_close and right_is_very_close:
                rotate_left(120, True)
            elif left_is_very_close:
                rotate_right(60, True)
            elif right_is_very_close:
                rotate_left(60, True)
            break
        else:
            sleep(0.05)
    if not que:
        que.extend([(move_straight, 100)])

