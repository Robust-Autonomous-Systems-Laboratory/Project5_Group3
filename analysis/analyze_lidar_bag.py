# Project 5 Implementation Part Two
# Anders Smitterberg
# Progress Munoriarwa

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

import os
import rclpy
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Configuration - The manual says LDS1 but this is actually the LDS2
# ---------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))

rclpy.init()

BAG_FILES = [
    (os.path.join(HERE, "../data/rosbag_0_5m"), 0.5),
    (os.path.join(HERE, "../data/rosbag_1m"),   1.0),
    (os.path.join(HERE, "../data/rosbag_2m"),   2.0),
]

TARGET_ANGLE = 0.0   # radians — 0 = forward-facing beam
ANGLE_WINDOW = 0.1   # radians to average over

os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)

results = {}


# ---------------------------------------------------------------------------
# Section 1 (2.1): Bag Reading — Extract Relevant Beams
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

        index = int((TARGET_ANGLE - msg.angle_min) / msg.angle_increment)

        half = int((ANGLE_WINDOW * 0.5) / msg.angle_increment)
        i0 = max(index - half, 0)
        i1 = min(index + half, len(msg.ranges) - 1)

        window = np.array(msg.ranges[i0:i1+1], dtype=float)
        valid  = np.isfinite(window) & (window >= msg.range_min) & (window <= msg.range_max)
        all_ranges.extend(window[valid].tolist())

    results[true_dist] = {"data": np.array(all_ranges)}
    print(f"{len(all_ranges)} measurements")

# ---------------------------------------------------------------------------
# Section 2 (2.2): Histogram Analysis
# ---------------------------------------------------------------------------

print("\n--- Histograms ---")

for true_dist, r in sorted(results.items()):
    data = r["data"]

    plt.figure()

    from scipy import stats as scipy_stats
    plt.hist(data, bins=60, density=True, alpha=0.6, label="measurements")

    mu, sigma = scipy_stats.norm.fit(data)
    x = np.linspace(data.min(), data.max(), 500)
    plt.plot(x, scipy_stats.norm.pdf(x, mu, sigma), label=f"Gaussian  Sigma={sigma:.4f}")

    plt.axvline(true_dist, color="red", linestyle="--", label=f"z*={true_dist} m")

    plt.title(f"z* = {true_dist} m  (N={len(data)})")
    plt.xlabel("Range (m)")
    plt.ylabel("Density")
    plt.legend()
    fname = os.path.join(HERE, f"figures/hist_{str(true_dist).replace('.', '_')}m.png")
    plt.savefig(fname, dpi=200)
    plt.close()
    print(f"  Saved {fname}")


# ---------------------------------------------------------------------------
# Section 3 (2.3): Parameter Estimation — sigma_hit and Bias
# ---------------------------------------------------------------------------

print("\n--- Parameter Estimation ---")
print(f"{'z*(m)':>6} {'N':>6} {'Mean(m)':>9} {'Bias(m)':>9} {'sigma_hit(m)':>13} {'Outliers':>10}")
print("-" * 60)

for true_dist, r in sorted(results.items()):
    data = r["data"]

    mean      = np.mean(data)
    sigma_hit = np.std(data)
    bias      = mean - true_dist
    
    # outliers > 3 sigma
    outliers = np.sum(np.abs(data - mean) > 3 * sigma_hit)

    results[true_dist].update({"mean": mean, "sigma_hit": sigma_hit, "bias": bias, "outliers": outliers})

    print(f"{true_dist:6.2f} {len(data):6d} {mean:9.4f} {bias:9.4f} {sigma_hit:13.4f} {outliers:10d}")


# ---------------------------------------------------------------------------
# Section 4 (2.4): Model Validation — sigma_hit vs Distance
# ---------------------------------------------------------------------------

print("\n--- Model Validation ---")

distances = np.array(sorted(results.keys()))

sigma_hits = np.array([results[d]["sigma_hit"] for d in distances])
biases     = np.array([results[d]["bias"]      for d in distances])

# fit line
popt = np.polyfit(distances, sigma_hits, 1)
sigma_1, sigma_0 = popt
print(f"  sigma_hit(z*) = {sigma_0:.5f} + {sigma_1:.5f} * z*")

# check for short or infinite
for true_dist, r in sorted(results.items()):
    data      = r["data"]

# are there any shorts? short defined as less than 3 std away from mean
    short_rate = np.mean(data < (true_dist - 3 * r["sigma_hit"]))
    results[true_dist]["short_rate"] = short_rate
    print(f"  z*={true_dist:3.1f}m  short_rate={short_rate:.2%}")

plt.figure()
plt.scatter(distances, sigma_hits, color="blue", s=80, label="Estimated sigma_hit")

z_range = np.linspace(0, 2.5, 100)
plt.plot(z_range, sigma_0 + sigma_1 * z_range, "r--", 
         label=f"Fit: {sigma_0:.4f} + {sigma_1:.4f}z*")

plt.xlabel("True distance (m)")
plt.ylabel("sigma_hit (m)")
plt.title("sigma_hit vs Distance")
plt.legend()
plt.grid(True, alpha=0.3)
plt.ylim(0, max(sigma_hits) * 1.2)
plt.savefig(os.path.join(HERE, "figures/sigma_vs_distance.png"), dpi=200)
plt.close()
print("  Saved figures/sigma_vs_distance.png")

rclpy.shutdown()
