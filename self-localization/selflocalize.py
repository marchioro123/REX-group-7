import cv2
import particle
import camera
import numpy as np
import time
import threading
import queue
import math
from timeit import default_timer as timer
import sys
from scipy.stats import norm

# Flags
showGUI = True  # Whether or not to open GUI windows
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
    from utils import calculate_distance, calculate_turn_angle
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

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [10, 6]
landmarks = {
    10: (0.0, 0.0),  # Coordinates for landmark 1
    6: (200.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks

seen = {
    10: False,
    6: False
}

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
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
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
        cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(1, robottype='macbookpro', useCaptureThread=False)

    arlo = robot.Robot()
    SERIAL_LOCK = threading.Lock()
    cmd_queue = queue.Queue()
    motor = MotorThread(arlo, cmd_queue, serial_lock=SERIAL_LOCK)
    motor.start()

    while True:

        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
            elif action == ord('x'): # Backwards
                velocity -= 4.0
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
            elif action == ord('d'): # Right
                angular_velocity -= 0.2
        
        # # Use motor controls to update particles
        # # XXX: Make the robot drive
        # # XXX: You do this
        if all(seen.values()):
          # time.sleep(1000)
          
            target_x, target_y = (landmarks[6][0] + landmarks[10][0]) / 2, (landmarks[6][1] + landmarks[10][1]) / 2
            pos_x, pos_y, est_theta = est_pose.getX(), est_pose.getY(), est_pose.getTheta()
           
            turn_angle = calculate_turn_angle(pos_x, pos_y, (90.0 - math.degrees(est_theta)) % 360.0, target_x, target_y)
            distance = calculate_distance(pos_x, pos_y, target_x, target_y)
            print(f"Turn {turn_angle:.2f}°, then go {distance:.3f} cm forward")
            input()

            cmd_queue.put(("turn_n_degrees", turn_angle))
            cmd_queue.put(("drive_n_cm_forward", 0, distance))

            particle.move_particles(particles, target_x-pos_x, target_y-pos_y, -math.radians(turn_angle))

            for k in seen:
                seen[k] = False

            while (not motor.has_started() or motor.is_turning() or motor.is_driving_forward()):
                time.sleep(0.1)
            motor.clear_has_started()

            particle.add_uncertainty(particles, 10, 10*math.pi / 180)
            print("Stopped at target")

            input()

        else:
            print("Turn 50 degrees")
            turn_angle = 50
            cmd_queue.put(("turn_n_degrees", turn_angle))
            particle.move_particles(particles, 0, 0, -math.radians(turn_angle))

            while (not motor.has_started() or motor.is_turning() or motor.is_driving_forward()):
                time.sleep(0.1)
            motor.clear_has_started()
            particle.add_uncertainty(particles, 0, 10*math.pi / 180)
            print("Finished turning")

        time.sleep(1)

        for j in range(1):
            # time.sleep(1)
            # Fetch next frame
            colour = cam.get_next_frame()
            
            # Detect objects
            objectIDs, dists, angles = cam.detect_aruco_objects(colour)

            best_distances = dict()
            best_angles = dict()

            if not isinstance(objectIDs, type(None)):
                particle.add_uncertainty(particles, 2, 2*math.pi / 180)
                
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
                # XXX: Do something for each detected object - remember, the same ID may appear several times

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
                    k=0
                    for p in particles:
                        
                        weight = p.getWeight()
                        dist_from = p.distFrom(landmarks[box_id][0], landmarks[box_id][1])

                        dir_landmark = np.array(((Lx - p.getX()) / dist_from, (Ly - p.getY()) / dist_from))
                        dir_particle = np.array((np.cos(p.getTheta()), np.sin(p.getTheta())))
                        dir_orth_particle = np.array((- np.sin(p.getTheta()), np.cos(p.getTheta())))

                        theta = np.sign(dir_landmark @ dir_orth_particle) * np.arccos(dir_landmark @ dir_particle)

                     #   if (k==0):
                      #  print(dir_landmark @ dir_particle)

                        p.setWeight( norm.pdf(best_angles[box_id] - theta, loc=0, scale=5 * math.pi / 180) * weight )
                        k=k+1

                total_weight = np.sum([p.getWeight() for p in particles])

                if (total_weight != 0):
                    for p in particles:
                        p.setWeight( p.getWeight() / total_weight )
                else:
                    print("ANGLE WEIGHT WAS 0")
                    for p in particles:
                        p.setWeight( 1 / num_particles )
        

                # Resampling
                # XXX: You do this
            #  print([p.getWeight() for p in particles])
                indices = np.random.default_rng().choice(
                    range(len(particles)),
                    size=num_particles-100,
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
            particles = particles + initialize_particles(100)


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

