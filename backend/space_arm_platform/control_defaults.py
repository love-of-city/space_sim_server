"""Shared SARM teleoperation defaults with no simulator dependencies."""

# Preserve the original scripted-grasp pose for historical/regression scenes.
LEGACY_PREGRASP = (
    0.0,
    -0.1790243,
    0.2159404,
    -0.0368382,
    0.0,
    0.0,
    0.01875,
    0.01875,
)

# A collision-free alternate arm posture selected for balanced Cartesian
# teleoperation.  Around this pose the six-axis chain can produce the default
# 0.05 m/s translation along +/-X, +/-Y and +/-Z while holding tool attitude
# within the configured joint-speed limits.  The tool remains within roughly
# one centimetre and nine degrees of the legacy PREGRASP tool pose.
BALANCED_TELEOP_HOME = (
    -0.82309,
    -0.42807,
    0.44472,
    -1.62499,
    -0.92330,
    4.79592,
    0.01875,
    0.01875,
)

BALANCED_TELEOP_PROFILE = "teleop-balanced-v1"
# New scenes select a body-frame elbow-high branch offline, then persist it.
# The v1 profile and saved instances remain unchanged for reproducibility.
ELBOW_UP_TELEOP_PROFILE = "teleop-elbow-up-v1"
ZERO_START_TELEOP_PROFILE = "teleop-zero-prepare-v1"
DEFAULT_RANDOMIZATION_PROFILE = ZERO_START_TELEOP_PROFILE
DEFAULT_OPERATING_JOINT_DEG = (0.0, -67.6, -86.6, 143.2, -85.5, 0.0)
ZERO_TELEOP_HOME = (0.0,) * 6 + (0.01875, 0.01875)

# Keep randomized starts inside the validated well-conditioned neighborhood.
BALANCED_TELEOP_JOINT_SPANS = (
    0.0300,
    0.02625,
    0.02625,
    0.0225,
    0.0300,
    0.0300,
    0.0100,
    0.0100,
)
