import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import math


def density_function(x,u,s):
     return (1 / (math.sqrt(2*math.pi) * s)) * math.e**(-0.5 * ((x - u)**2 / s**2))


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


# SIR normal distribution with N(5,4)
def sir_norm(k, rng=np.random.default_rng()):

    # 1. Proposal: normal instead of uniform 
    samples = rng.normal(loc=5, scale=2, size=k)  # N(μ=5, σ=2)

    # 2. Compute unnormalized weights:
    #    w(x) = p(x) / q(x), where q(x) ~ N(5,2)
    q_vals = np.array([density_function(x, 5, 2) for x in samples])
    weights = p(samples) / q_vals

    # 3. Normalize
    weightsN = weights / np.sum(weights)

    # 4. Resample with replacement
    resampled = rng.choice(samples, size=k, replace=True, p=weightsN)
    return resampled


# Plot results for k = 20, 100, 1000
ks = [20, 100, 1000, 10000]
x = np.linspace(-2, 15, 1000)
px = p(x)

for k in ks:
    # Histogram as probability density uniform
    resampled1 = sir_uniform(k)
    resampled2 = sir_norm(k)
    plt.figure(figsize=(7,4))
    plt.hist(resampled1, bins=30, density=True, alpha=0.6, label=f"SIR resampled uniform (k={k})")
    plt.hist(resampled2, bins=30, density=True, alpha=0.6, label=f"SIR resampled normalized (k={k})", color="orange")
    plt.plot(x, px, "r-", lw=2, label="True distribution p(x)")
    plt.xlabel("x")
    plt.ylabel("Density")
    plt.title(f"SIR with uniform and normalized proposal, k={k}")
    plt.legend()
    plt.show()
