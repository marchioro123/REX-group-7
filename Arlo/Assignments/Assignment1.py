from time import sleep

import sys
sys.path.append("/home/pi/Desktop/REX-group-7/Arlo/python")
import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

sleep(1)

leftPercentageSpeed = 1.083
speed = 64



while (1):
    print(arlo.go_diff(round(40), 115, 1, 1))
    sleep(4.3)
    print(arlo.go_diff(round(115*leftPercentageSpeed), 40, 1, 1))
    sleep(4.3)



# while (1):
#     print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
#     sleep(1.8)
#     print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
#     sleep(0.71)

