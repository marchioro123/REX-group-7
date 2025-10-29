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
ROBOT_RADIUS = 0.7

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 7, 4, 5]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    7: (0.0, 300.0),  # Coordinates for landmark 2
    4: (400.0, 0.0),  # Coordinates for landmark 3
    5: (400.0, 300.0)  # Coordinates for landmark 4
}
visit_order = [1, 7, 4, 5, 1]

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

    while visit_order:        
        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)

        best_distances = dict()
        best_angles = dict()

        if not isinstance(objectIDs, type(None)):
            #particle.add_uncertainty(particles, 2, 2*math.pi / 180)
            
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
            # XXX: You do this
            for p in particles:
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
            # print([p.getWeight() for p in particles])
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

        if seen_next_target or seen_two_boxes:
            if seen_next_target:
                turn_angle = -best_angles[visit_order[0]]*180/np.pi
                print(f"Turn {turn_angle:.2f}°")
             #   input()
                # if (abs(turn_angle) > 5):
                cmd_queue.put(("turn_n_degrees", turn_angle))
                particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
                while (not motor.has_started() or motor.is_turning()):
                    time.sleep(0.02)
                motor.clear_has_started()
            elif seen_two_boxes:
                target_x, target_y = landmarks[visit_order[0]]
                pos_x, pos_y, est_theta = est_pose.getX(), est_pose.getY(), est_pose.getTheta()
            
                turn_angle = calculate_turn_angle(pos_x, pos_y, (90.0 - math.degrees(est_theta)) % 360.0, target_x, target_y)
                print(f"Turn {turn_angle:.2f}°")
             #   input()
                # if (abs(turn_angle) > 5):
                cmd_queue.put(("turn_n_degrees", turn_angle))
                particle.move_particles(particles, 0, 0, -math.radians(turn_angle))
                while (not motor.has_started() or motor.is_turning()):
                    time.sleep(0.02)
                motor.clear_has_started()

            #particle.add_uncertainty(particles, 0, 7*math.pi / 180)
            for k in seen:
                seen[k] = False
            
            time.sleep(1)
            colour = cam.get_next_frame()
            objectIDs, dists, angles = cam.detect_aruco_objects(colour)
            print(f"Saw IDs: {objectIDs}")
            x_es = []
            z_es = []
            target_x = None
            target_y = None
            obstacle_centers = []

            if not isinstance(objectIDs, type(None)):
                for i in range(len(objectIDs)):
                    print("inside the loop")
                    print(i)
                    x, _, z = cam.tvecs[i][0]
                    x_dir, _, z_dir = cam.rvecs[i][0]
                    print(visit_order[0])
                    if objectIDs[i] == visit_order[0]:
                        target_x = x
                        target_y = z
                        x_es.append(x)
                        z_es.append(z)
                        continue
                    x_es.append(x)
                    z_es.append(z)
                    obstacle_center = (x-sin(z_dir/2)*DISTANCE_TO_CENTER, z+cos(z_dir/2)*DISTANCE_TO_CENTER)
                    obstacle_centers.append(obstacle_center)

            # maximum_absolute_value = max((abs(x) for x in x_es), default=2)
            # z_max = max(z_es, default=4)
            print("Creating Occupancy Map")
            path_res = 0.05
            occ_map = GridOccupancyMap(low=(-3.5, -0.3), high=(3.5, 5.3), res=path_res)
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
                    target_x if target_x is not None else 0,
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
                    ) - 0.3
                ]),
                robot_model=robot_model,
                map=occ_map,
                expand_dis=1,
                path_resolution=path_res,
                ) 
            # show_animation = True
            # metadata = dict(title="RRT Test")
            # writer = FFMpegWriter(fps=15, metadata=metadata)
            # fig = plt.figure()

            print("Calculating path")
            # with writer.saving(fig, "rrt_test.mp4", 100):
            path = rrt.planning(animation=False)
                #path = rrt.planning(animation=show_animation, writer=writer)

            if path is None:
                print("Cannot find path")
            else:
                print("found path!!")
                print(path)
                simple_path = simplify_path(path, occ_map)
                print("simple path")
                print(simple_path)
                # if show_animation:
                #     rrt.draw_graph()
                #     plt.plot([x for (x, y) in path], [y for (x, y) in path], '-b')
                #     plt.plot([x for (x, y) in simple_path], [y for (x, y) in simple_path], '-r')
                #     plt.grid(True)
                #     plt.pause(0.01)  # Need for Mac
                #     plt.show()
                #     writer.grab_frame()

                pos_x, pos_y, angle = 0, 0, 0
                aborted = False
                last_index = len(simple_path) - 2
                for i, (target_x, target_y) in enumerate(simple_path[:-1]):
                    turn_angle = calculate_turn_angle(pos_x, pos_y, angle, target_x, target_y)
                    distance = calculate_distance(pos_x, pos_y, target_x, target_y) * 100 #rtt planning is in meters
                    print(f"Turn {turn_angle:.2f}°, then go {distance:.3f} cm forward")
                  #  input()
                    # if (abs(turn_angle) > 5):
                    cmd_queue.put(("turn_n_degrees", turn_angle))
                    while (not motor.has_started() or motor.is_turning()):
                        time.sleep(0.02)
                    motor.clear_has_started()

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
                           # with SERIAL_LOCK:
                           
                            t = motor.get_wait_until()
                            motor.hard_stop()
                            print(left_dist)
                            print(right_dist)
                            print(front_dist)
                            aborted = True
                            timenow = time.monotonic()
                            if i==last_index and t - timenow < 1.5 and t > 0:
                                print("Target..")
                                aborted = False
                            print("Emergency stop!!")
                            print(t - timenow)
                            # particle.move_particles(particles, (target_x-pos_x)/2, (target_y-pos_y)/2, -(math.radians(turn_angle)/2))
                            # particle.add_uncertainty(particles, distance/2, (turn_angle/2)*math.pi / 180)
                            break
                        time.sleep(0.02)
                    motor.clear_has_started()

                    # particle.move_particles(particles, target_x-pos_x, target_y-pos_y, -math.radians(turn_angle))
                    if aborted:
                        # with SERIAL_LOCK:
                        #     left_dist = arlo.read_left_ping_sensor()
                        #     right_dist = arlo.read_right_ping_sensor()
                        object_left = left_dist != -1 and left_dist < 300
                        object_right = right_dist != -1 and right_dist < 300
                        if object_left and not object_right:
                            print(f"left sensor")
                          #  input()
                            cmd_queue.put(("turn_n_degrees", 45))
                            cmd_queue.put(("drive_n_cm_forward", 0, 30))
                            cmd_queue.put(("turn_n_degrees", -45))
                        elif not object_left:
                            print(f"right sensor")
                         #   input()
                            cmd_queue.put(("turn_n_degrees", -45))
                            cmd_queue.put(("drive_n_cm_forward", 0, 30))
                            cmd_queue.put(("turn_n_degrees", 45))
                        else:
                            print(f"backwards sensor")
                            cmd_queue.put(("drive_n_cm_forward", 0, -20))

                        while (not motor.has_started() or motor.is_turning() or motor.is_driving_forward()):
                            time.sleep(0.02)
                        motor.clear_has_started()
                        break

                    # particle.add_uncertainty(particles, 7, 7*math.pi / 180)
                    pos_x, pos_y = target_x, target_y
                    angle = (angle + turn_angle) % 360


            # print("Checking if target is close enough")
            # input()
            # colour = cam.get_next_frame()
            # objectIDs, dists, angles = cam.detect_aruco_objects(colour)
            # if not isinstance(objectIDs, type(None)):
            #     if visit_order[0] in objectIDs:
            #         idx = list(objectIDs).index(visit_order[0])
            #         print(f"Reached target {visit_order[0]} (distance {dists[idx]:.1f} cm)")
            #         if dists[idx] < 40.0:
            #             print(f"Reached target {visit_order.pop(0)} (distance {dists[idx]:.1f} cm) — next target: {visit_order[0]}")
                particles = initialize_particles(num_particles)
                times_turned = 0
                if not aborted:
                    reached = visit_order.pop(0)
                    print(f"Reached target {reached}")
                    if visit_order:  # check if there's a next target
                        print(f"Next target: {visit_order[0]}")
                    else:
                        print("No more targets!")
              #  input()

        else:
            if times_turned > 10:
                turn_angle = random.uniform(-180, 180)
                print(f"Random turn {turn_angle:.2f}°, then go {50:.3f} cm forward")
             #   input()
                # if (abs(turn_angle) > 5):
                cmd_queue.put(("turn_n_degrees", turn_angle))
                while (not motor.has_started() or motor.is_turning()):
                    time.sleep(0.02)
                motor.clear_has_started()
                cmd_queue.put(("drive_n_cm_forward", 0, 50))
                aborted = False
                while (not motor.has_started() or motor.is_turning() or motor.is_driving_forward()):
                    with SERIAL_LOCK:
                        front_dist = arlo.read_front_ping_sensor()
                        left_dist = arlo.read_left_ping_sensor()
                        right_dist = arlo.read_right_ping_sensor()
                    if should_stop(front_dist, left_dist, right_dist):
                        motor.hard_stop()
                        aborted = True
                        break
                    time.sleep(0.02)
                motor.clear_has_started()

                particles = initialize_particles(num_particles)
                for k in seen:
                    seen[k] = False

                if not aborted:
                    times_turned = 0

            else:
                turn_angle = 35
                print(f"Turn {turn_angle} degrees")
                cmd_queue.put(("turn_n_degrees", turn_angle))
                particle.move_particles(particles, 0, 0, -math.radians(turn_angle))

                while (not motor.has_started() or motor.is_turning()):
                    time.sleep(0.02)
                motor.clear_has_started()
                particle.add_uncertainty(particles, 0, 5*math.pi / 180)
                print("Finished turning")
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

