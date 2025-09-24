import sys
from time import sleep
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from math import sin, cos
import cv2

sys.path.append("..")
from utils import find_corner_coordinates, detector

from path_planning.grid_occ import GridOccupancyMap
import path_planning.robot_models as robot_models
from path_planning.rrt import RRT

np.set_printoptions(threshold=sys.maxsize)
from matplotlib.animation import FFMpegWriter

DISTANCE_TO_CENTER = 0.115
BOX_RADIUS = 0.17
ROBOT_RADIUS = 0.2250

"""
# BOX
DISTANCE_TO_CENTER = 0.115
BOX_RADIUS = 0.17
"""

image = cv2.imread("../../../Images/5_codes.png")
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

path_res = 0.05
map = GridOccupancyMap(low=(-maximum_absolute_value-0.5, -0.3), high=(maximum_absolute_value+0.5, max(z_es)+1), res=path_res)
for i in range(map.n_grids[0]):
    for j in range(map.n_grids[1]):
        centroid = np.array([map.map_area[0][0] + map.resolution * (i+0.5), map.map_area[0][1] + map.resolution * (j+0.5)])
        for o in obstacle_centers:
            if np.linalg.norm(centroid - o) <= BOX_RADIUS + ROBOT_RADIUS:
                map.grid[i, j] = 1
                break

map.draw_map()
plt.savefig("Occupancy_grid.png")

robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   #

rrt = RRT(
    start=[0, 0],
    goal=[0, 1.9],
    robot_model=robot,
    map=map,
    expand_dis=0.2,
    path_resolution=path_res,
    )

show_animation = True
metadata = dict(title="RRT Test")
writer = FFMpegWriter(fps=15, metadata=metadata)
fig = plt.figure()

with writer.saving(fig, "rrt_test.mp4", 100):
    path = rrt.planning(animation=show_animation, writer=writer)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
            writer.grab_frame()