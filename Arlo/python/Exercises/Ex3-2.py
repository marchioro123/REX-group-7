import sys
from time import sleep
import cv2

sys.path.append("..")
import robot
from camera import cam, find_corner_coordinates, detector
from motor_thread import MotorThread

arlo = robot.Robot()

image = cam.capture_array("main")
corners, ids, rejected = detector.detectMarkers(image)
rvecs, tvecs, _ = find_corner_coordinates(corners)
print(tvecs)







found_end = False
while found_end is False:
    image = cam.capture_array("main")

    corners, ids, rejected = detector.detectMarkers(image)
    if ids is None:
        arlo.go_diff(40,40,1,0)
        print("rotato")
        sleep(0.2)
        arlo.stop()
        sleep(0.2)
    else:
        arlo.stop()
        print("found!")
        rvecs, tvecs, _ = find_corner_coordinates(corners)
        print(tvecs)
        x,_,z = [tvecs[0,0,i] for i in range(3)]
        if x > z/5:
            print("rotated to the right")
            arlo.go_diff(40,40,1,0)
            sleep(0.1)
            arlo.stop()
            input()
        elif x < -z/5:
            print("rotated to the left")
            arlo.go_diff(40,40,0,1)
            sleep(0.1)
            arlo.stop()
            input()
        else:
            print("i am not rotating now")
            if z > 0.5:
                print("i am too far away")
                arlo.go_diff(40,40,1,1)
                sleep(0.3)
                arlo.stop()
                input()
            else:
                found_end = True

arlo.stop()
print("i am at the end!")
       
    
