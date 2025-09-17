import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import time
from pprint import *

import sys
import numpy as np
from time import sleep

sys.path.append("..")
import robot
from motor_thread import MotorThread

arlo = robot.Robot()

import picamera2
""" try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1) """


print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
imageSize = (1640, 1232)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit),
                                                            "ScalerCrop": (0,0,3280,2464)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

focus_length = 1315
camera_matrix = np.array([[focus_length, 0, 1640/2],
                          [0, focus_length, 1232/2],
                          [0,   0,   1]], dtype=np.float32)
dist_coeffs = np.zeros((5,))
marker_length = 0.145

time.sleep(1)  # wait for camera to setup

image = cam.capture_array("main")

corners, ids, rejected = detector.detectMarkers(image)
if ids is not None:
    print("found!")
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
    print(tvecs)
