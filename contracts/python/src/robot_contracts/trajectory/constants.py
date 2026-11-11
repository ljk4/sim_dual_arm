"""Versioned trajectory bounds and numeric tolerances shared by all endpoints."""
MAX_JOINTS = 64
MAX_SEGMENTS = 128
MAX_HORIZON_NS = 3_000_000_000
POSITION_ATOL = 1e-9
VELOCITY_ATOL = 1e-9
ACCELERATION_ATOL = 1e-9
RELATIVE_TOLERANCE = 1e-9
LIMIT_ATOL = 1e-9
LIMIT_RTOL = 1e-9
FOLLOWER_ALGORITHM_VERSION = "bounded-target-follower/ruckig-0.15.3/v1"
