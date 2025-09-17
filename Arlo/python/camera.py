import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import time
import numpy as np

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)


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

# Prepares the detector for our used ARUCO codes
aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# Sets the 
focus_length = 1316
camera_matrix = np.array([[focus_length, 0, imageSize[0]/2],
                          [0, focus_length, imageSize[1]/2],
                          [0,   0,   1]], dtype=np.float32)
dist_coeffs = np.zeros((5,))
marker_length = 0.145

def find_corner_coordinates(corners):
    return aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

