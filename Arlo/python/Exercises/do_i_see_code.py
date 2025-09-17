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
        arlo.stop()
        print("found!")
        rvecs, tvecs, _ = find_corner_coordinates(corners)
        print(tvecs)
        x,_,z = [tvecs[0,0,i] for i in range(3)]
        if x > z/4:
            arlo.go_diff(40,40,1,0)
            sleep(0.1)
            arlo.stop()
            input()
        elif x < -z/4:
            arlo.go_diff(40,40,0,1)
            sleep(0.1)
            arlo.stop()
            input()
        else:
            if z > 0.3:
                arlo.go_diff(40,40,1,1)
                sleep(0.25)
                arlo.stop()
                input()
            else:
                found_end = True

arlo.stop()
print("i am at the end!")

        
    
