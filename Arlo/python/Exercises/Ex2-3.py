import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import math


def density_function(x,u,s):
    return (1/((2*math.pi)**(1/2)*s) * math.e**((-1/2)*(x-u)**2)/(s**2))


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
    samples = rng.normal(loc=5, scale=2, size=k)  # N(5, 4)

    # 2. Compute unnormalized weights
    what_this = norm.pdf(samples, loc=5, scale=2)
    
    #print(np.array([density_function(x,5,2) for x in samples]))
    normalizing_weights = np.array([density_function(x,5,2) for x in samples])
    normalizing_weights = normalizing_weights / np.sum(normalizing_weights)
    print("array start", p(samples), "array_end")
    print("normalizing_weights")
    print(normalizing_weights)
    print(np.sum(normalizing_weights))
    print("what this")
    print(what_this)
    print("")
    weights = p(samples) / normalizing_weights
    print(weights)


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
    plt.hist(resampled2, bins=30, density=True, alpha=0.6, label=f"SIR resampled normalized (k={k})")
    plt.plot(x, px, "r-", lw=2, label="True distribution p(x)")
    plt.xlabel("x")
    plt.ylabel("Density")
    plt.title(f"SIR with uniform and normalized proposal, k={k}")
    plt.legend()
    plt.show()


""""
# SIR normal distribution with N(5,4)
def sir_norm(k, rng=np.random.default_rng()):

    # 1. Proposal: normal instead of uniform 
    samples = rng.normal(loc=5, scale=2, size=k)  # N(5, 4)

    # 2. Compute unnormalized weights
    weights = p(samples) / norm.pdf(samples, loc=5, scale=2)

    # 3. Normalize
    weightsN = weights / np.sum(weights)

    # 4. Resample with replacement
    resampled = rng.choice(samples, size=k, replace=True, p=weightsN)
    return resampled

# Plot results for k = 20, 100, 1000
ks = [20, 100, 1000]
x = np.linspace(-2, 15, 1000)
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
"""