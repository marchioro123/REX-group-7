import cv2
import numpy as np
import time
import math
from math import atan2, degrees
import robot
from time import sleep

LANDMARK_IDS = [1, 2, 3, 4]
VISIT_DISTANCE_M = 0.40
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE_M = 0.16
FOCAL_PX = 900.0
K = np.array([[FOCAL_PX, 0, 640],
              [0, FOCAL_PX, 360],
              [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)


SPEED_FWD = 52
SPEED_TURN = 45
SPEED_SLOW = 44

STOP_DISTANCE_M = 0.45
CLEAR_DISTANCE_M = 0.60
BEARING_OK_DEG = 6.0


def bearing_deg_from_tvec(tvec):
    x, y, z = tvec.ravel()
    if z == 0: z = 1e-6
    return degrees(atan2(x, z))

def distance_from_tvec(tvec):
    return float(np.linalg.norm(tvec))

arlo = robot.Robot()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
parameters = cv2.aruco.DetectorParameters_create()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

order = LANDMARK_IDS + [LANDMARK_IDS[0]]
idx = 0
visited = set()

print("[INFO] Race start!")
start_time = time.time()
MAX_RACE_TIME = 15 * 60

while idx < len(order):
    if time.time() - start_time > MAX_RACE_TIME:
        print("[INFO] Time limit reached!")
        break

    front_left = arlo.read_left_ping_sensor() / 100.0
    front_right = arlo.read_right_ping_sensor() / 100.0
    front = min(front_left, front_right)
    if front < STOP_DISTANCE_M:
        print("[AVOID] Obstacle detected!")
        arlo.stop()
        arlo.go_diff(SPEED_TURN, SPEED_TURN, 0, 1)
        sleep(0.8)
        arlo.stop()
        continue

    ok, frame = cap.read()
    if not ok:
        continue

    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        arlo.go_diff(SPEED_TURN, SPEED_TURN, 1, 0)
        sleep(0.6)
        arlo.stop()
        continue

    ids = ids.flatten().tolist()
    if order[idx] not in ids:
        arlo.go_diff(SPEED_TURN, SPEED_TURN, 0, 1)
        sleep(0.6)
        arlo.stop()
        continue

    i = ids.index(order[idx])
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[i]], MARKER_SIZE_M, K, DIST_COEFFS
    )
    tvec = tvecs[0][0]
    dist = distance_from_tvec(tvec)
    bearing = bearing_deg_from_tvec(tvec)

    print(f"[VIS] L{order[idx]} dist={dist:.2f}m bearing={bearing:+.1f}°")

    if abs(bearing) > BEARING_OK_DEG:
        if bearing > 0:
            arlo.go_diff(SPEED_TURN, SPEED_TURN, 1, 0)
        else:
            arlo.go_diff(SPEED_TURN, SPEED_TURN, 0, 1)
        sleep(0.15)
        arlo.stop()
        continue

    if dist > VISIT_DISTANCE_M:
        arlo.go_diff(SPEED_FWD, SPEED_FWD, 1, 1)
        sleep(0.2)
        arlo.stop()
        continue

    arlo.stop()
    print(f"[OK] Visited L{order[idx]}")
    visited.add(order[idx])
    idx += 1
    sleep(1.0)

arlo.stop()
cap.release()
print(f"[DONE] Race finished. Visited {len(visited)} landmarks in {time.time() - start_time:.1f}s.")
