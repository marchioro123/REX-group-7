import time
import statistics
#import matplotlib.pyplot as plt
import sys
sys.path.append("..")
import robot

arlo = robot.Robot()

def get_measurements(count=5, delay=3):
    measurements = []
    for _ in range(count):
        front_dist = arlo.read_front_ping_sensor()
        measurements.append(front_dist)
        time.sleep(delay)
    return measurements

measurements = get_measurements()
std_dev = statistics.stdev(measurements)
print(f"Measurements: {measurements} | stddev: {std_dev:.2f}")


# Wall
# 20cm
# 50cm
# 100cm
# 180cm
# 250cm

