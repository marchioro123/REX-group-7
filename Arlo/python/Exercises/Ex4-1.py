import sys
from time import sleep
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from math import sin, cos

sys.path.append("..")
import robot
from camera import cam, find_corner_coordinates, detector


image = cam.capture_array("main")
corners, ids, rejected = detector.detectMarkers(image)
rvecs, tvecs, _ = find_corner_coordinates(corners)


_, graph = plt.subplots(figsize=(5, 5))

x_es = []
z_es = []
for i in range(len(ids)):
    x, y, z = tvecs[i][0]
    x_dir, _, z_dir = rvecs[i][0]
    print(corners[i])
    print(ids[i])
    print(x,y,z)
    print(rvecs[i])
    print("--------------")
    x_es.append(x)
    z_es.append(z)
    graph.scatter(x, z, color = "blue")
    graph.quiver(x,z, -sin(z_dir), -cos(z_dir))

maximum_absolute_value = max(abs(x) for x in x_es)

circle = patches.Circle((0, -0.225), radius=0.225, color='black', fill=True)
graph.add_patch(circle)

edge_x = np.linspace(-maximum_absolute_value-0.5, maximum_absolute_value+0.5, 100)
edge_y = abs(edge_x*1.75)
graph.plot(edge_x, edge_y,color="red")

graph.set_xlim(-maximum_absolute_value-0.5,maximum_absolute_value+0.5)
graph.set_ylim(-0.3,max(z_es)+1)

graph.set_aspect('equal', adjustable='box')
plt.savefig("map.png")