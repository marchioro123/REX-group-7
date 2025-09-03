from time import sleep

import sys
sys.path.append("/home/pi/Desktop/REX-group-7/Arlo/python")
import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

sleep(1)

# send a go_diff command to drive forward
forwardLeftSpeed = 69
forwardRightSpeed = 64

turnLeftSpeed = 64
turnRightSpeed = 64

while (1):
    print(arlo.go_diff(forwardLeftSpeed, forwardRightSpeed, 1, 1))
    sleep(1.8)
    print(arlo.go_diff(turnLeftSpeed, turnRightSpeed, 0, 1))
    sleep(0.71)



# x = arlo.read_right_wheel_encoder()

# y = arlo.read_left_wheel_encoder()