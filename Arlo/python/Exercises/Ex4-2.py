import sys
from time import sleep
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from math import sin, cos

sys.path.append("..")
from camera import cam, find_corner_coordinates, detector

from RRT.grid_occ import GridOccupancyMap
np.set_printoptions(threshold=sys.maxsize)

DISTANCE_TO_CENTER = 0.115
BOX_RADIUS = 0.17
ROBOT_RADIUS = 0.2250

"""
# BOX
DISTANCE_TO_CENTER = 0.115
BOX_RADIUS = 0.17
"""

image = cam.capture_array("main")
corners, ids, rejected = detector.detectMarkers(image)
rvecs, tvecs, _ = find_corner_coordinates(corners)


_, graph = plt.subplots(figsize=(5, 5))

x_es = []
z_es = []
obstacle_centers = []
for i in range(len(ids)):
    x, y, z = tvecs[i][0]
    x_dir, _, z_dir = rvecs[i][0]
    print(ids[i])
    print(x,y,z)
    print(rvecs[i])
    print("--------------")
    x_es.append(x)
    z_es.append(z)
    graph.scatter(x, z, color = "blue")
    graph.quiver(x,z, sin(z_dir/2), -cos(z_dir/2))
    obstacle_center = (x-sin(z_dir/2)*DISTANCE_TO_CENTER, z+cos(z_dir/2)*DISTANCE_TO_CENTER)
    obstacle_centers.append(obstacle_center)
    obstacle_circle = patches.Circle(obstacle_center, radius=BOX_RADIUS, color='red', fill=True)
    graph.add_patch(obstacle_circle)

maximum_absolute_value = max(abs(x) for x in x_es)

circle = patches.Circle((0, -0.225), radius=0.225, color='black', fill=True)
graph.add_patch(circle)

edge_x = np.linspace(-maximum_absolute_value-0.5, maximum_absolute_value+0.5, 100)
edge_y = abs(edge_x*1.75)
graph.plot(edge_x, edge_y,color="blue")

graph.set_xlim(-maximum_absolute_value-0.5,maximum_absolute_value+0.5)
graph.set_ylim(-0.3,max(z_es)+1)

graph.set_aspect('equal', adjustable='box')
plt.savefig("map.png")



# --------------------------

map = GridOccupancyMap(low=(-maximum_absolute_value-0.5, -0.3), high=(maximum_absolute_value+0.5, max(z_es)+1), res=0.01)
for i in range(map.n_grids[0]):
    for j in range(map.n_grids[1]):
        centroid = np.array([map.map_area[0][0] + map.resolution * (i+0.5), map.map_area[0][1] + map.resolution * (j+0.5)])
        for o in obstacle_centers:
            if np.linalg.norm(centroid - o) <= BOX_RADIUS + ROBOT_RADIUS:
                map.grid[i, j] = 1
                break

map.draw_map()
plt.savefig("Occupancy_grid.png")