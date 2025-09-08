from time import sleep, time

from collections import deque


def move_straight(l):
    # computes time to go straight, assuming 1 meter = 1.8 second
    time = l*1.8

    print(arlo.go_diff(68, 64, 1, 1))
    wait_till = time() + time
    return wait_till

def rotate_left(alpha):
    # computes time to rotate, assuming 90 degrees = 0.71 second
    time = alpha/90*0.71

    print(arlo.go_diff(68, 64, 0, 1))
    wait_till = time() + time
    return wait_till

que = deque()
que.extend([(move_straight, 1), (rotate_left, 90)]*4)


wait_till = time()
while que:
    next_command, parameter = que.popleft()
    wait_till = next_command(parameter)
    while time() < wait_till:
        #just checking sensors
        sleep(0.01)

