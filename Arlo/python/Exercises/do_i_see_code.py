import sys
from time import sleep

sys.path.append("..")
import robot
from camera import cam, find_corner_coordinates, detector
from motor_thread import MotorThread

arlo = robot.Robot()


found_end = False
while found_end is False:
    image = cam.capture_array("main")

    corners, ids, rejected = detector.detectMarkers(image)
    if ids is None:
        arlo.go_diff(40,40,1,0)
        print("rotato")
        sleep(0.1)
    else:
        print("found!")
        rvecs, tvecs, _ = find_corner_coordinates(corners)
        print(tvecs)
    
