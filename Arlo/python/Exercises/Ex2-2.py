import time
import statistics
import sys
sys.path.append("..")
import robot

arlo = robot.Robot()

def get_measurements(count=5, delay=3):
    measurements = []
    for _ in range(count):
        front_dist = arlo.read_front_ping_sensor()
        print(f"Measurement: {front_dist}")
        measurements.append(front_dist)
        time.sleep(delay)
    return measurements

measurements = get_measurements()
std_dev = statistics.stdev(measurements)
print(f"Measurements: {measurements} | stddev: {std_dev:.2f}")



#Plots
# import matplotlib.pyplot as plt
# distances = [300, 250, 180, 100, 50, 20, 10, 5]
# stddevs_table = [1.64, 0.45, 1.64, 0.55, 0.00, 0.45, 0.55, 0.45]
# stddevs_cardboard = [0.45, 3.13, 0.45, 0.55, 0.84, 0.55, 0.55, 1.10]

# distances_sponge = [50, 20, 10, 5]
# stddevs_sponge = [1.34, 0.55, 0.45, 0.55]

# plt.plot(distances, stddevs_table, label="Table")
# plt.plot(distances, stddevs_cardboard, label="Cardboard")
# plt.plot(distances_sponge, stddevs_sponge, label="Sponge")
# plt.xlabel("Distance (cm)")
# plt.ylabel("Standard Deviation")
# plt.grid(True)
# plt.legend()
# plt.show()