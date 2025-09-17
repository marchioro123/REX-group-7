import numpy as np
import matplotlib.pyplot as plt

Z = np.array([20, 50, 100, 180, 250, 300])   # distances in cm
x = np.array([979, 391, 190, 104, 76, 62])   # measured image size in pixels
X = 14.5                                     # physical object size in cm

# Focal length estimates
f_values = x * Z / X

# Mean and std for all points
mean_all = np.mean(f_values)
std_all = np.std(f_values, ddof=1)

# Robust mean/std excluding the 100 cm outlier
mask_inliers = np.array([True, True, False, True, True, True])
f_robust = f_values[mask_inliers]
mean_robust = np.mean(f_robust)
std_robust = np.std(f_robust, ddof=1)

print("All points: mean = %.2f px, std = %.2f px" % (mean_all, std_all))
print("Robust subset: mean = %.2f px, std = %.2f px" % (mean_robust, std_robust))

# Regression slope for x vs 1/Z
invZ = 1 / Z
slope = np.sum(invZ * x) / np.sum(invZ**2)

# -----------------------------
# Plot 1: x vs 1/Z with regression line
# -----------------------------
plt.figure(figsize=(7,5))
plt.scatter(invZ, x, color="blue", label="Measurements")
# Add labels
for i, (iz, val) in enumerate(zip(invZ, x)):
    plt.text(iz, val+20, f"{val:.0f}", ha="center", fontsize=9)
# Regression line
xline = np.linspace(0, invZ.max()*1.05, 100)
yline = slope * xline
plt.plot(xline, yline, 'r-', label="Regression fit")
plt.xlabel("1 / Z (1/cm)")
plt.ylabel("Image size x (pixels)")
plt.title("Image size vs inverse distance")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# Plot 2: f vs Z with means
# -----------------------------
plt.figure(figsize=(7,5))
plt.scatter(Z, f_values, color="blue", label="Estimated f")
# Add labels for f values
for z, f in zip(Z, f_values):
    plt.text(z, f+15, f"{f:.1f}", ha="center", fontsize=9)
# Mean lines
plt.axhline(mean_all, color="red", linestyle="--", label=f"Mean (all): {mean_all:.1f}")
plt.axhline(mean_robust, color="green", linestyle="--", label=f"Mean (robust): {mean_robust:.1f}")
plt.xlabel("Distance Z (cm)")
plt.ylabel("Focal length f (pixels)")
plt.title("Estimated focal length at different distances")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# Plot 3: Residuals of regression
# -----------------------------
pred_x = slope * invZ
residuals = x - pred_x

plt.figure(figsize=(7,5))
plt.scatter(Z, residuals, color="purple", label="Residuals")
# Add labels
for z, r in zip(Z, residuals):
    plt.text(z, r+2, f"{r:.1f}", ha="center", fontsize=9)
plt.axhline(0, color="black", linestyle="--")
plt.xlabel("Distance Z (cm)")
plt.ylabel("Residual (pixels)")
plt.title("Residuals of regression model")
plt.legend()
plt.grid(True)
plt.show()
