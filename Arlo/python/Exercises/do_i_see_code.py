import sys
from time import sleep

sys.path.append("..")
import robot
from camera import cam, find_corner_coordinates, detector
from motor_thread import MotorThread

arlo = robot.Robot()



image = cam.capture_array("main")

corners, ids, rejected = detector.detectMarkers(image)
if ids is not None:
    print("found!")
    rvecs, tvecs, _ = find_corner_coordinates(corners)
    print(tvecs)
