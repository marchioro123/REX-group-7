import math

def should_stop(front_dist, left_dist, right_dist):        
    return (
        (front_dist != -1 and front_dist < 300) or 
        (left_dist != -1 and left_dist < 150) or 
        (right_dist != -1 and right_dist < 150)
    )

def calculate_distance(x1, y1, x2, y2): 
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx**2 + dy**2)

def calculate_turn_angle(x1, y1, angle_deg, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    target_bearing = (90 - math.degrees(math.atan2(dy, dx))) % 360
    current_bearing = angle_deg % 360
    return (target_bearing - current_bearing + 180) % 360 - 180

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

def simplify_path(path, occupancy_map):
    """Greedy path pruning to reduce unnecessary points."""
    simplified = [path[0]]
    i = 0
    while i < len(path) - 1:
        # Try to jump as far as possible along the path
        for j in range(len(path)-1, i, -1):
            if is_collision_free(simplified[-1], path[j], occupancy_map):
                simplified.append(path[j])
                i = j
                break
    return simplified

def is_collision_free(p1, p2, occupancy_map):
    """Check if the straight line between p1 and p2 is free of obstacles."""
    distance = np.linalg.norm(np.array(p2) - np.array(p1))
    steps = int(np.ceil(distance / (occupancy_map.resolution)))  # More conservative
    for t in np.linspace(0, 1, steps):
        point = (1 - t) * np.array(p1) + t * np.array(p2)
        if occupancy_map.in_collision(point):
            return False
    return True