import sys
from time import sleep

import matplotlib as plt

sys.path.append("..")
import robot
from camera import cam, find_corner_coordinates, detector


image = cam.capture_array("main")
corners, ids, rejected = detector.detectMarkers(image)
rvecs, tvecs, _ = find_corner_coordinates(corners)

x_es = []
z_es = []
for code in tvecs:
    x, _, z = code[0]
    print(x,z)
    x_es.append(x)
    z_es.append(z)

plt.scatter(x_es,z_es)
plt.show()