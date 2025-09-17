import numpy as np
import matplotlib.pyplot as plt

# Data
Z = np.array([20, 50, 100, 180, 250, 300])   # distance in cm
x = np.array([979, 391, 140, 104, 76, 62])   # image size in pixels
X = 14.5                                     # physical size of object in cm

# Compute f for each measurement
f_values = x * Z / X

# Means
mean_all = np.mean(f_values)
std_all = np.std(f_values, ddof=1)

# Robust mean (exclude outlier at 100 cm)
mask_inliers = np.array([True, True, False, True, True, True])
f_robust = f_values[mask_inliers]
mean_robust = np.mean(f_robust)
std_robust = np.std(f_robust, ddof=1)

# --- Plot f vs Z with labels ---
plt.figure(figsize=(7,5))
plt.scatter(Z, f_values, color="blue", label="Estimated f")

# Add labels with values
for z, f in zip(Z, f_values):
    plt.text(z, f+15, f"{f:.1f}", ha="center", fontsize=9, color="black")

# Mean lines
plt.axhline(mean_all, color="red", linestyle="--", label=f"Mean (all): {mean_all:.1f}")
plt.axhline(mean_robust, color="green", linestyle="--", label=f"Mean (robust): {mean_robust:.1f}")

plt.xlabel("Distance Z (cm)")
plt.ylabel("Focal length f (pixels)")
plt.title("Estimated focal length at different distances")
plt.legend()
plt.grid(True)
plt.show()
