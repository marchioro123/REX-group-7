import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm


def p(x):
    return (0.3 * norm.pdf(x, loc=2.0, scale=1.0) +
            0.4 * norm.pdf(x, loc=5.0, scale=2.0) +
            0.3 * norm.pdf(x, loc=9.0, scale=1.0))


# SIR uniform distribution
def sir_uniform(k, rng=np.random.default_rng()):
    samples = rng.uniform(0, 15, size=k)
    weights = p(samples)
    weightsN = weights / np.sum(weights)
    return rng.choice(samples, size=k, replace=True, p=weightsN)


# SIR normal distribution - N(5,4)
def sir_normal(k, rng=np.random.default_rng()):
    samples = rng.normal(loc=5, scale=2, size=k)
    weights = p(samples) / norm.pdf(samples, loc=5, scale=2)
    weightsN = weights / np.sum(weights)
    return rng.choice(samples, size=k, replace=True, p=weightsN)


# Plot results for k = 20, 100, 1000, 10000
ks = [20, 100, 1000, 10000]
x = np.linspace(-2, 15, 1000)
px = p(x)

for k in ks:
    # Generate resampled data
    resampled_uniform = sir_uniform(k)
    resampled_normal = sir_normal(k)

    # ---- Plot for uniform proposal ----
    plt.figure(figsize=(7, 4))
    plt.hist(resampled_uniform, bins=50, density=True, alpha=0.6, label=f"SIR resampled uniform (k={k})")
    plt.plot(x, px, "r-", lw=2, label="True distribution p(x)")
    plt.xlabel("x")
    plt.ylabel("Probability Density")
    plt.title(f"SIR with uniform proposal, k={k}")
    plt.legend()
    plt.show()

    # ---- Plot for normal proposal ----
    plt.figure(figsize=(7, 4))
    plt.hist(resampled_normal, color="orange", bins=50, density=True, alpha=0.6, label=f"SIR resampled normal (k={k})")
    plt.plot(x, px, "r-", lw=2, label="True distribution p(x)")
    plt.xlabel("x")
    plt.ylabel("Probability Density")
    plt.title(f"SIR with normal proposal, k={k}")
    plt.legend()
    plt.show()



