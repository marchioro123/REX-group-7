import cv2
import particle
import camera
import numpy as np
import time
import threading
import queue
import math
import random
from math import sin, cos
from timeit import default_timer as timer
import sys
from scipy.stats import norm
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

# Flags
showGUI = False  # Whether or not to open GUI windows
onRobot = True  # Whether or not we are running on the Arlo robot

def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot

if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../Arlo/python")

try:
    import robot
    from motor_thread import MotorThread
    from utils import calculate_distance, calculate_turn_angle, simplify_path, should_stop
    from path_planning.grid_occ import GridOccupancyMap
    from path_planning.robot_models import PointMassModel
    from path_planning.rrt import RRT
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False

# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

DISTANCE_TO_CENTER = 0.1
BOX_RADIUS = 0.18
ROBOT_RADIUS = 0.25

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [8, 7, 3, 10]
landmarks = {
    8: (0.0, 0.0),  # Coordinates for landmark 1
    7: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    10: (400.0, 300.0)  # Coordinates for landmark 4
}
visit_order = [8, 7, 3, 10, 8]

landmark_colors = [CRED, CGREEN, CBLUE, CMAGENTA] # Colors used when drawing the landmarks

seen = {i: False for i in landmarkIDs}


def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
         ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)

def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(np.random.uniform(-40, 440), np.random.uniform(-40, 340), np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles

# Main program #
try:
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)


    # Initialize particles
    num_particles = 2000
    particles = initialize_particles(num_particles)

    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec

    # Initialize the robot (XXX: You do this)

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    draw_world(est_pose, particles, world)

    print("Opening and initializing camera")
    if isRunningOnArlo():
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(1, robottype='macbookpro', useCaptureThread=False)

    arlo = robot.Robot()
    SERIAL_LOCK = threading.Lock()
    cmd_queue = queue.Queue()
    motor = MotorThread(arlo, cmd_queue, serial_lock=SERIAL_LOCK)
    motor.start()

    times_turned = 0
    reached_target_once = False

    while visit_order:        
        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)

        best_distances = dict()
        best_angles = dict()

        if not isinstance(objectIDs, type(None)):
            # List detected objects
            for i in range(len(objectIDs)):
                obj_id = objectIDs[i]
                if (obj_id not in landmarkIDs):
                    continue

                print("Object ID = ", obj_id, ", Distance = ", dists[i], ", angle = ", angles[i]*180/np.pi)
                seen[obj_id] = True
                if (obj_id in best_distances.keys()):
                    best_distances[obj_id] = (best_distances[obj_id] + dists[i] + 22.5) / 2
                    best_angles[obj_id] = (best_angles[obj_id] + angles[i]) / 2             
                if (obj_id not in best_distances.keys()):
                    best_distances[obj_id] = dists[i] + 22.5
                    best_angles[obj_id] = angles[i]

            # Compute particle weights
            for p in particles:
                x, y = p.getX(), p.getY()
                if (x < -60) or (x > 460) or (y < -60) or (y > 360):
                    p.setWeight(0.0)
                else:
                    p.setWeight(1.0)

            for box_id in best_distances.keys():
                if (box_id not in landmarkIDs):
                    continue
                for p in particles:
                    weight = p.getWeight()
                    p.setWeight( norm.pdf( p.distFrom(landmarks[box_id][0], landmarks[box_id][1]) , loc=best_distances[box_id], scale=10) * weight )

            for box_id in best_distances.keys():
                if (box_id not in landmarkIDs):
                    continue
                Lx, Ly = landmarks[box_id]
                for p in particles:
                    weight = p.getWeight()
                    dist_from = p.distFrom(Lx, Ly)
                    dir_landmark = np.array(((Lx - p.getX()) / dist_from, (Ly - p.getY()) / dist_from))
                    dir_particle = np.array((np.cos(p.getTheta()), np.sin(p.getTheta())))
                    dir_orth_particle = np.array((- np.sin(p.getTheta()), np.cos(p.getTheta())))
                    theta = np.sign(dir_landmark @ dir_orth_particle) * np.arccos(dir_landmark @ dir_particle)

                    p.setWeight( norm.pdf(best_angles[box_id] - theta, loc=0, scale=5 * math.pi / 180) * weight )

            total_weight = np.sum([p.getWeight() for p in particles])

            if (total_weight != 0):
                for p in particles:
                    p.setWeight( p.getWeight() / total_weight )
            else:
                print("ANGLE WEIGHT WAS 0")
                for p in particles:
                    p.setWeight( 1 / num_particles )
    

            # Resampling
            indices = np.random.default_rng().choice(
                range(len(particles)),
                size=num_particles,
                replace=True,
                p=[p.getWeight() for p in particles]
            )
            particles = [particles[i].copy() for i in indices]

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)


        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
        print("predX = ", est_pose.getX(), ", predY = ", est_pose.getY(), ", predTheta = ", est_pose.getTheta()*180/np.pi)

        particles = particles[:-100] + initialize_particles(100)
        for p in particles:
            p.setWeight(1.0 / len(particles))

        # # XXX: Make the robot drive
        seen_next_target = objectIDs is not None and visit_order[0] in objectIDs
        seen_two_boxes = sum(seen.values()) >= 2
        done_full_rotation = times_turned > 10

        if seen_next_target or seen_two_boxes or done_full_rotation:
            if seen_next_target:
                turn_angle = -best_angles[visit_order[0]]*180/np.pi
                print(f"Turn {turn_angle:.2f}°")
                #input()
                particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
                particle.add_uncertainty(particles, 0, (turn_angle/20)*math.pi / 180)
                cmd_queue.put(("turn_n_degrees", turn_angle))
                while (not motor.has_started() or motor.is_turning()):
                    time.sleep(0.02)
                motor.clear_has_started()
            elif seen_two_boxes or done_full_rotation:
                target_x, target_y = landmarks[visit_order[0]]
                pos_x, pos_y, est_theta = est_pose.getX(), est_pose.getY(), est_pose.getTheta()
                turn_angle = calculate_turn_angle(pos_x, pos_y, (90.0 - math.degrees(est_theta)) % 360.0, target_x, target_y)
                print(f"Turn {turn_angle:.2f}°")
                #input()
                particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
                particle.add_uncertainty(particles, 0, (turn_angle/20)*math.pi / 180)
                cmd_queue.put(("turn_n_degrees", turn_angle))
                while (not motor.has_started() or motor.is_turning()):
                    time.sleep(0.02)
                motor.clear_has_started()

            for k in seen:
                seen[k] = False
            times_turned = 0

            time.sleep(1)
            colour = cam.get_next_frame()
            objectIDs, dists, angles = cam.detect_aruco_objects(colour)
            print(f"Saw IDs: {objectIDs}")
            target_x = None
            target_y = None
            obstacle_centers = []

            if not isinstance(objectIDs, type(None)):
                for i in range(len(objectIDs)):
                    x, _, z = cam.tvecs[i][0]
                    x_dir, _, z_dir = cam.rvecs[i][0]
                    if objectIDs[i] == visit_order[0]:
                        target_x = x #add small const? DISTANCE_TO_CENTER?
                        target_y = z
                        seen_next_target = True
                        continue
                    elif not done_full_rotation:
                        obstacle_center = (x-sin(z_dir/2)*DISTANCE_TO_CENTER, z+cos(z_dir/2)*DISTANCE_TO_CENTER) #x_dir?
                        obstacle_centers.append(obstacle_center)

            print("Creating Occupancy Map")
            path_res = 0.05
            occ_map = GridOccupancyMap(low=(-1.5, -0.3), high=(1.5, 5.3), res=path_res)
            for i in range(occ_map.n_grids[0]):
                for j in range(occ_map.n_grids[1]):
                    centroid = np.array([occ_map.map_area[0][0] + occ_map.resolution * (i+0.5), occ_map.map_area[0][1] + occ_map.resolution * (j+0.5)])
                    for o in obstacle_centers:
                        if np.linalg.norm(centroid - o) <= BOX_RADIUS + ROBOT_RADIUS:
                            occ_map.grid[i, j] = 1
                            break
            occ_map.draw_map()
            plt.savefig("Occupancy_grid.png")

            robot_model = PointMassModel(ctrl_range=[-path_res, path_res])
            pos_x, pos_y = est_pose.getX(), est_pose.getY()

            rrt = RRT(
                start=np.array([0, 0]),
                goal = np.array([ 
                    target_x if target_x is not None else 0, #add small const? DISTANCE_TO_CENTER?
                    (
                        target_y if target_y is not None else (
                            best_distances[visit_order[0]] / 100
                            if visit_order[0] in best_distances
                            else calculate_distance(
                                pos_x / 100, pos_y / 100,
                                landmarks[visit_order[0]][0] / 100,
                                landmarks[visit_order[0]][1] / 100
                            )
                        )
                    ) - 0.35
                ]),
                robot_model=robot_model,
                map=occ_map,
                expand_dis=1,
                path_resolution=path_res,
                ) 
            print("Calculating path")
            path = rrt.planning(animation=False)
            if path is None:
                print("Cannot find path")
            else:
                print("found path!!")
                print(path)
                simple_path = simplify_path(path, occ_map)
                print("simple path")
                print(simple_path)

                pos_x, pos_y, angle = 0, 0, 0
                aborted = False
                last_index = len(simple_path) - 2
                for i, (target_x, target_y) in enumerate(reversed(simple_path[:-1])):
                    turn_angle = calculate_turn_angle(pos_x, pos_y, angle, target_x, target_y)
                    distance = calculate_distance(pos_x, pos_y, target_x, target_y) * 100 #rtt planning is in meters
                    print(f"Turn {turn_angle:.2f}°, then go {distance:.3f} cm forward")
                    #input()
                    particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
                    particle.add_uncertainty(particles, 0, (turn_angle/20)*math.pi / 180)
                    cmd_queue.put(("turn_n_degrees", turn_angle))
                    while (not motor.has_started() or motor.is_turning()):
                        time.sleep(0.02)
                    motor.clear_has_started()


                    particle.move_particles_forward(particles, distance)
                    particle.add_uncertainty(particles, distance/100, 0)
                    cmd_queue.put(("drive_n_cm_forward", 0, distance))
                    front_dist = 0
                    left_dist = 0
                    right_dist = 0
                    while (not motor.has_started() or motor.is_driving_forward()):
                        with SERIAL_LOCK:
                            front_dist = arlo.read_front_ping_sensor()
                            left_dist = arlo.read_left_ping_sensor()
                            right_dist = arlo.read_right_ping_sensor()
                        if should_stop(front_dist, left_dist, right_dist):
                            t = motor.get_wait_until()
                            timenow = time.monotonic()
                            motor.hard_stop()
                            aborted = True
                            if i==last_index and t - timenow < 1.3 and t > 0:
                                aborted = False
                            if t > 0:
                                leftover_dist = (t- timenow) /0.042
                                particle.move_particles_forward(particles, -leftover_dist)
                                particle.add_uncertainty(particles, (distance-leftover_dist)/100, 0)
                            print(t - timenow)
                            print("Emergency stop!!")
                            motor.clear_has_started()
                            break
                        time.sleep(0.02)
                    motor.clear_has_started()

                    if aborted:
                        object_left = left_dist != -1 and left_dist < 300
                        object_right = right_dist != -1 and right_dist < 300
                        wiggle_angle = 20 if object_left else -20
                        full_wiggle = False
                        num_wiggles = 0
                        while not full_wiggle:
                            num_wiggles = num_wiggles+1
                            print("try wiggle")
                            particle.move_particles(particles, 0, 0, -math.radians(wiggle_angle))
                            particle.add_uncertainty(particles, 0, (wiggle_angle/20)*math.pi / 180)
                            cmd_queue.put(("turn_n_degrees", wiggle_angle))
                            while (not motor.has_started() or motor.is_turning()):
                                time.sleep(0.02)
                            motor.clear_has_started()

                            full_wiggle = True
                            particle.move_particles_forward(particles, 45)
                            particle.add_uncertainty(particles, 3, 0)
                            cmd_queue.put(("drive_n_cm_forward", 0, 45))
                            while (not motor.has_started() or motor.is_driving_forward()):
                                with SERIAL_LOCK:
                                    front_dist = arlo.read_front_ping_sensor()
                                    left_dist = arlo.read_left_ping_sensor()
                                    right_dist = arlo.read_right_ping_sensor()
                                if should_stop(front_dist, left_dist, right_dist, True):
                                    motor.hard_stop()
                                    full_wiggle = False
                                    particle.move_particles_forward(particles, -25)
                                    particle.add_uncertainty(particles, 10, 0)
                                    print("Emergency wiggle stop!!")
                                    break
                                time.sleep(0.02)
                            motor.clear_has_started()
                            if full_wiggle:
                                particle.move_particles(particles, 0, 0, -math.radians(-(num_wiggles*wiggle_angle)))
                                particle.add_uncertainty(particles, 0, (num_wiggles*wiggle_angle/20)*math.pi / 180)
                                cmd_queue.put(("turn_n_degrees", -(num_wiggles*wiggle_angle)))
                                while (not motor.has_started() or motor.is_turning()):
                                    time.sleep(0.02)
                                motor.clear_has_started()
                        break

                    pos_x, pos_y = target_x, target_y
                    angle = (angle + turn_angle) % 360

                if not aborted:
                    reached = visit_order[0]
                    if seen_next_target:              #not reached_target:
                        visit_order.pop(0)
                        print(f"Reached target {reached}")
                        reached_target_once = False
                    elif reached_target_once:
                        saw_target = False
                        for _ in range(10):
                            colour = cam.get_next_frame()
                            objectIDs, dists, angles = cam.detect_aruco_objects(colour)
                            if visit_order[0] in objectIDs:
                                saw_target = True
                                break
                            turn_angle = 35
                            print(f"Turn {turn_angle} degrees")
                            particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
                            particle.add_uncertainty(particles, 0, (turn_angle/20)*math.pi / 180)
                            cmd_queue.put(("turn_n_degrees", turn_angle))
                            while (not motor.has_started() or motor.is_turning()):
                                time.sleep(0.02)
                            motor.clear_has_started()
                            time.sleep(1)
                        if not saw_target:
                            visit_order.pop(0)
                            print(f"Reached target {reached}")
                        reached_target_once = False
                    else:
                        reached_target_once = True

                    if visit_order:
                        print(f"Next target: {visit_order[0]}")
                    else:
                        print("No more targets!")
                time.sleep(1)
        else:
            turn_angle = 35
            print(f"Turn {turn_angle} degrees")
            particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
            particle.add_uncertainty(particles, 0, (turn_angle/20)*math.pi / 180)
            cmd_queue.put(("turn_n_degrees", turn_angle))
            while (not motor.has_started() or motor.is_turning()):
                time.sleep(0.02)
            motor.clear_has_started()
            times_turned += 1
            time.sleep(1)

        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)
    
            # Show frame
            cv2.imshow(WIN_RF1, colour)

            # Show world
            cv2.imshow(WIN_World, world)
    
  
finally: 
    # Make sure to clean up even if an exception occurred
    
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()

