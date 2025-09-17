import sys
from time import sleep

import matplotlib.pyplot as plt

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

maximum_absolute_value = max(abs(x) for x in x_es)

plt.scatter(x_es,z_es, color = "blue")
plt.scatter([0],[0], s=1000,color="black")
plt.xlim(-maximum_absolute_value-1,maximum_absolute_value+1)
plt.ylim(bottom=0)
plt.savefig("plot.png")