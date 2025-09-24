def should_stop(front_dist, left_dist, right_dist):        
    return (
        (front_dist != -1 and front_dist < 500) or 
        (left_dist != -1 and left_dist < 200) or 
        (right_dist != -1 and right_dist < 200)
    )


import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np

imageSize = (1640, 1232)

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