from time import sleep

import sys
sys.path.append("/home/pi/Desktop/REX-group-7/Arlo/python")
import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

sleep(1)


leftSpeed = 69
rightSpeed = 64



while (1):
    print(arlo.go_diff(40, 122, 1, 1))
    sleep(4)
    # print(arlo.go_diff(127, 40, 1, 1))
    # sleep(4)



# while (1):
#     print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
#     sleep(1.8)
#     print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
#     sleep(0.71)

