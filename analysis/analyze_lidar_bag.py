# Project 5 Implementation
# Anders Smitterberg
# Progress Munoriarwa


# ---------------------------------------------------------------------------
# Configuration - The manual says LDS1 but I do not believe this is the LDS1
# ---------------------------------------------------------------------------

BAG_FILES = [
    ("../data/rosbag_0_5m", 0.5),
    ("../data/rosbag_1m",   1.0),
    ("../data/rosbag_2m",   2.0),
]

TARGET_ANGLE  = 0.0   # radians — 0 = forward-facing beam
ANGLE_WINDOW  = 0.1   # radians — angular width to average over
RANGE_MAX     = 3.5   # This is from the LDS1 but I do not know if ours is the same


# ---------------------------------------------------------------------------
# Section 1: Bag Reader
# ---------------------------------------------------------------------------

# Progress I believe you made this earlier.

# ---------------------------------------------------------------------------
# Section 2: Parameter Estimation (2.3)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Section 3: Histogram Analysis (2.2)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Section 4: sigma_hit vs Distance (2.4)
# ---------------------------------------------------------------------------