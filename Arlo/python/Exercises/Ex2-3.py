import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

# Define target density p(x) (mixture of Gaussian model)
def p(x):
    return (0.3 * norm.pdf(x, loc=2.0, scale=1.0) +
            0.4 * norm.pdf(x, loc=5.0, scale=2.0) +
            0.3 * norm.pdf(x, loc=9.0, scale=1.0))


# SIR uniform distribution
def sir_uniform(k, rng=np.random.default_rng()):

    # 1. uniform samples from [0, 15]
    samples = rng.uniform(0, 15, size=k)

    # 2. Compute unnormalized weights
    weights = p(samples)

    # 3. Normalize
    weightsN = weights / np.sum(weights)

    # 4. Resample with replacement
    resampled = rng.choice(samples, size=k, replace=True, p=weightsN)
    return resampled

# Plot results for k = 20, 100, 1000
ks = [20, 100, 1000]
x = np.linspace(0, 15, 1000)
px = p(x)

for k in ks:
    # Histogram as probability density uniform
    resampled = sir_uniform(k)
    plt.figure(figsize=(7,4))
    plt.hist(resampled, bins=30, density=True, alpha=0.6, label=f"SIR resampled (k={k})")
    plt.plot(x, px, "r-", lw=2, label="True distribution p(x)")
    plt.xlabel("x")
    plt.ylabel("Density")
    plt.title(f"SIR with uniform proposal, k={k}")
    plt.legend()
    plt.show()



# SIR normal distribution with N(5,4)
def sir_norm(k, rng=np.random.default_rng()):

    # 1. Proposal: normal instead of uniform
    samples = rng.normal(loc=5, scale=2, size=k)  # N(5, 4)

    # 2. Compute unnormalized weights
    weights = p(samples)

    # 3. Normalize
    weightsN = weights / np.sum(weights)

    # 4. Resample with replacement
    resampled = rng.choice(samples, size=k, replace=True, p=weightsN)
    return resampled

# Plot results for k = 20, 100, 1000
ks = [20, 100, 1000]
x = np.linspace(0, 15, 1000)
px = p(x)

for k in ks:
    # Histogram as probability density normal distrubution
    resampled = sir_norm(k)
    plt.figure(figsize=(7,4))
    plt.hist(resampled, bins=30, density=True, alpha=0.6, label=f"SIR resampled (k={k})")
    plt.plot(x, px, "r-", lw=2, label="True distribution p(x)")
    plt.xlabel("x")
    plt.ylabel("Density")
    plt.title(f"SIR with normal proposal, k={k}")
    plt.legend()
    plt.show()
