# Project 5 Implementation Part Two
# Anders Smitterberg
# Progress Munoriarwa

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

import os
import yaml
import rclpy
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from scipy.optimize import curve_fit

# ---------------------------------------------------------------------------
# Configuration - The manual says LDS1 but this is actually the LDS2
# ---------------------------------------------------------------------------

rclpy.init()

BAG_FILES = [
    ("../data/rosbag_0_5m", 0.5),
    ("../data/rosbag_1m",   1.0),
    ("../data/rosbag_2m",   2.0),
]

TARGET_ANGLE = 0.0   # radians — 0 = forward-facing beam
ANGLE_WINDOW = 0.1   # radians to average over
RANGE_MAX    = 8000

os.makedirs("figures", exist_ok=True)
os.makedirs("../results", exist_ok=True)

# results dict carries data between sections: {true_dist: {"data": ..., "mean": ..., ...}}
results = {}


# ---------------------------------------------------------------------------
# Section 1: Bag Reading
# ---------------------------------------------------------------------------

for bag_path, true_dist in BAG_FILES:
    print(f"Reading {bag_path}  (z* = {true_dist} m)...", end=" ", flush=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    topic_types = {tm.name: get_message(tm.type) for tm in reader.get_all_topics_and_types()}

    all_ranges = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != "/scan":
            continue
        msg = deserialize_message(raw, topic_types[topic])

        # TODO: compute centre index for TARGET_ANGLE
        # index = int((TARGET_ANGLE - msg.angle_min) / msg.angle_increment)

        # TODO: compute half-window and clamp to valid range
        # half = int((ANGLE_WINDOW * 0.5) / msg.angle_increment)
        # i0 = max(index - half, 0)
        # i1 = min(index + half, len(msg.ranges) - 1)

        # TODO: extract, filter invalid readings, append to all_ranges
        # window = np.array(msg.ranges[i0:i1+1], dtype=float)
        # valid  = np.isfinite(window) & (window >= msg.range_min) & (window <= msg.range_max)
        # all_ranges.extend(window[valid].tolist())

    results[true_dist] = {"data": np.array(all_ranges)}
    print(f"{len(all_ranges)} measurements")


# ---------------------------------------------------------------------------
# Section 2: Parameter Estimation (2.3)
# ---------------------------------------------------------------------------

print("\n--- Parameter Estimation ---")
print(f"{'z*(m)':>6} {'N':>6} {'Mean(m)':>9} {'Bias(m)':>9} {'sigma_hit(m)':>13} {'Outliers':>10} {'Shorts':>8}")
print("-" * 65)

for true_dist, r in sorted(results.items()):
    data = r["data"]

    # TODO: compute stats and store back into results[true_dist]
    # mean         = np.mean(data)
    # sigma_hit    = np.std(data)
    # bias         = mean - true_dist
    # outlier_rate = np.mean(np.abs(data - mean) > 3 * sigma_hit)
    # short_rate   = np.mean(data < true_dist - 3 * sigma_hit)

    # results[true_dist].update({
    #     "mean": mean, "sigma_hit": sigma_hit, "bias": bias,
    #     "outlier_rate": outlier_rate, "short_rate": short_rate,
    # })

    # print(f"{true_dist:6.2f} {len(data):6d} {mean:9.4f} {bias:9.4f} {sigma_hit:13.4f} "
    #       f"{outlier_rate:9.2%} {short_rate:7.2%}")
    pass


# ---------------------------------------------------------------------------
# Section 3: Histogram Analysis (2.2)
# ---------------------------------------------------------------------------

print("\n--- Histograms ---")

for true_dist, r in sorted(results.items()):
    data = r["data"]

    plt.figure()

    # TODO: histogram
    # plt.hist(data, bins=60, density=True, alpha=0.6, label="measurements")

    # TODO: Gaussian fit overlay
    # mu, sigma = stats.norm.fit(data)
    # x = np.linspace(data.min(), data.max(), 500)
    # plt.plot(x, stats.norm.pdf(x, mu, sigma), label=f"Gaussian  σ={sigma:.4f}")

    # TODO: vertical line at true distance
    # plt.axvline(true_dist, color="red", linestyle="--", label=f"z*={true_dist} m")

    plt.title(f"z* = {true_dist} m  (N={len(data)})")
    plt.xlabel("Range (m)")
    plt.ylabel("Density")
    plt.legend()
    fname = f"figures/hist_{str(true_dist).replace('.', 'p')}m.png"
    plt.savefig(fname, dpi=200)
    plt.close()
    print(f"  Saved {fname}")


# ---------------------------------------------------------------------------
# Section 4: sigma_hit vs Distance (2.4)
# ---------------------------------------------------------------------------

print("\n--- sigma_hit vs Distance ---")

distances  = np.array(sorted(results.keys()))
sigma_hits = np.array([results[d]["sigma_hit"] for d in distances])
biases     = np.array([results[d]["bias"]      for d in distances])

# TODO: fit linear model  sigma_hit(z*) = sigma0 + sigma1 * z*
# popt, _ = curve_fit(lambda z, s0, s1: s0 + s1 * z, distances, sigma_hits)
# sigma0, sigma1 = popt
# print(f"  sigma_hit(z*) = {sigma0:.5f} + {sigma1:.5f} * z*")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))

# TODO: scatter sigma_hits vs distances + linear fit on ax1
# TODO: scatter biases vs distances + axhline(0) on ax2

ax1.set_xlabel("True distance z* (m)")
ax1.set_ylabel("sigma_hit (m)")
ax1.set_title("sigma_hit vs Distance")
ax1.grid(True, alpha=0.3)
ax2.set_xlabel("True distance z* (m)")
ax2.set_ylabel("Bias = mean - z* (m)")
ax2.set_title("Systematic Bias vs Distance")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("figures/sigma_vs_distance.png", dpi=200)
plt.close()
print("  Saved figures/sigma_vs_distance.png")


# ---------------------------------------------------------------------------
# Section 5: Beam Model Component Analysis
# ---------------------------------------------------------------------------

print("\n--- Beam Model Weights ---")
print(f"{'z*(m)':>6}  {'z_hit':>8}  {'z_short':>8}  {'z_max':>8}  {'z_rand':>8}")
print("-" * 48)

for true_dist, r in sorted(results.items()):
    data      = r["data"]
    sigma_hit = r["sigma_hit"]

    # TODO: compute weights
    # z_hit   = np.mean(np.abs(data - true_dist) <= 3 * sigma_hit)
    # z_short = np.mean(data < true_dist - 3 * sigma_hit)
    # z_max   = np.mean(data >= RANGE_MAX)
    # z_rand  = max(0.0, 1.0 - z_hit - z_short - z_max)

    # print(f"{true_dist:6.2f}  {z_hit:8.4f}  {z_short:8.4f}  {z_max:8.4f}  {z_rand:8.4f}")
    pass


# ---------------------------------------------------------------------------
# Section 6: Normality Tests (Q-Q plots + Shapiro-Wilk)
# ---------------------------------------------------------------------------

print("\n--- Normality Tests ---")
print(f"{'z*(m)':>6}  {'Shapiro W':>10}  {'p-value':>10}  {'Normal?':>10}")
print("-" * 44)

fig, axes = plt.subplots(1, len(results), figsize=(5 * len(results), 4))

for ax, (true_dist, r) in zip(axes, sorted(results.items())):
    data      = r["data"]
    sigma_hit = r["sigma_hit"]

    # TODO: isolate p_hit region
    # hit_data = data[np.abs(data - true_dist) <= 3 * sigma_hit]

    # TODO: Shapiro-Wilk (subsample to 5000 max)
    # sample = hit_data if len(hit_data) <= 5000 else np.random.choice(hit_data, 5000, replace=False)
    # W, p = stats.shapiro(sample)
    # print(f"{true_dist:6.2f}  {W:10.6f}  {p:10.6f}  {'Yes' if p > 0.05 else 'No':>10}")

    # TODO: Q-Q plot
    # stats.probplot(hit_data, dist="norm", plot=ax)

    ax.set_title(f"Q-Q  z*={true_dist} m")

plt.tight_layout()
plt.savefig("figures/qq_plots.png", dpi=200)
plt.close()
print("  Saved figures/qq_plots.png")


# ---------------------------------------------------------------------------
# Section 7: Save Results to YAML
# ---------------------------------------------------------------------------

print("\n--- Saving Results ---")

summary = {
    "uncertainty_model": {
        # TODO: replace None with sigma0 and sigma1 from section 4
        "sigma0": None,
        "sigma1": None,
        "description": "sigma_hit(z*) = sigma0 + sigma1 * z*",
    },
    "per_distance": {
        # TODO: populate from results dict
        # str(true_dist): {"mean": r["mean"], "sigma_hit": r["sigma_hit"], ...}
    },
}

with open("../results/calibration_results.yaml", "w") as f:
    yaml.dump(summary, f, default_flow_style=False, sort_keys=False)

print("  Saved ../results/calibration_results.yaml")

rclpy.shutdown()
