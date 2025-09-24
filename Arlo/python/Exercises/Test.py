import sys
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
from math import sin, cos, sqrt
from matplotlib.animation import FFMpegWriter

sys.path.append("..")
from utils import find_corner_coordinates, detector

from path_planning.grid_occ import GridOccupancyMap
from path_planning.robot_models import PointMassModel
from path_planning.rrt import RRT

DISTANCE_TO_CENTER = 0.1
BOX_RADIUS = 0.18
ROBOT_RADIUS = 0.2250

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
    n = int(np.ceil(np.linalg.norm(np.array(p2)-np.array(p1))/occupancy_map.resolution))
    for t in np.linspace(0, 1, n):
        point = np.array(p1)*(1-t) + np.array(p2)*t
        # Convert coordinates to grid indices
        i = int((point[0] - occupancy_map.map_area[0][0]) / occupancy_map.resolution)
        j = int((point[1] - occupancy_map.map_area[0][1]) / occupancy_map.resolution)
        # Clamp indices
        i = max(0, min(occupancy_map.n_grids[0]-1, i))
        j = max(0, min(occupancy_map.n_grids[1]-1, j))
        if occupancy_map.grid[i, j] == 1:
            return False
    return True

# Load image and detect obstacles
image = cv2.imread("../../../Images/3_boxes.png")
corners, ids, rejected = detector.detectMarkers(image)
rvecs, tvecs, _ = find_corner_coordinates(corners)

x_es = []
z_es = []
obstacle_centers = []
for i in range(len(ids)):
    x, y, z = tvecs[i][0]
    x_dir, _, z_dir = rvecs[i][0]
    x_es.append(x)
    z_es.append(z)
    obstacle_center = (x-sin(z_dir/2)*DISTANCE_TO_CENTER, z+cos(z_dir/2)*DISTANCE_TO_CENTER)
    obstacle_centers.append(obstacle_center)

maximum_absolute_value = max(abs(x) for x in x_es)
edge_x = np.linspace(-maximum_absolute_value-0.5, maximum_absolute_value+0.5, 100)
edge_y = abs(edge_x*1.75)


# Create occupancy grid
path_res = 0.01
map = GridOccupancyMap(low=(-maximum_absolute_value-0.5, -0.3), high=(maximum_absolute_value+0.5, max(z_es)+1), res=path_res)
for i in range(map.n_grids[0]):
    for j in range(map.n_grids[1]):
        centroid = np.array([map.map_area[0][0] + map.resolution * (i+0.5), map.map_area[0][1] + map.resolution * (j+0.5)])
        for o in obstacle_centers:
            if np.linalg.norm(centroid - o) <= BOX_RADIUS + ROBOT_RADIUS:
                map.grid[i, j] = 1
                break


# Initialize robot and RRT
robot = PointMassModel(ctrl_range=[-path_res, path_res])

rrt = RRT(
    start=[0, 0],
    goal=[0, 3],
    robot_model=robot,
    map=map,
    expand_dis=0.2,
    path_resolution=path_res,
    ) 

# Run RRT and save animation
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
        print(path)
        print(f"Original path points: {len(path)}")
        simple_path = simplify_path(path, map)
        print(f"Simplified path points: {len(simple_path)}")

        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in simple_path], [y for (x, y) in simple_path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Needed for Mac
            plt.show()
            writer.grab_frame()
