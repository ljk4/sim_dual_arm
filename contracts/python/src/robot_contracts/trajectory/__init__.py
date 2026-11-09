"""Trajectory math: cubic splines, bounded-follower plans and accepted records.

Exports are resolved lazily so modules outside this package may import a
submodule without triggering the package-wide import graph.
"""

from __future__ import annotations

import importlib
from typing import Any

_EXPORTS = {
    **{name: ".spline" for name in (
        "ACCELERATION_ATOL", "LIMIT_ATOL", "LIMIT_RTOL", "MAX_HORIZON_NS", "MAX_JOINTS",
        "MAX_SEGMENTS", "POSITION_ATOL", "RELATIVE_TOLERANCE", "VELOCITY_ATOL",
        "CubicSegment", "JointExtremes", "ReferenceExtremes", "ReferenceInputs",
        "ReferencePlan", "ReferenceSample", "check_continuous_limits",
        "compute_reference_extremes", "validate_reference_inputs",
        "validate_velocity_boundaries", "verify_reference_plan")},
    **{name: ".follower" for name in (
        "FOLLOWER_ALGORITHM_VERSION", "FollowerLimitFlags", "NormalizedFollowerPositions",
        "PreparedFollowerPlan", "compute_follower_deviations",
        "estimate_raw_terminal_velocities", "normalize_follower_positions",
        "restrict_terminal_velocities")},
    **{name: ".follower_reference" for name in (
        "DIGEST_HEX_LENGTH", "FOLLOWER_NORMALIZATION_VERSION",
        "FOLLOWER_REFERENCE_DIGEST_VERSION", "FOLLOWER_REFERENCE_KINDS",
        "FOLLOWER_SAMPLER_VERSION", "FollowerAppliedSample", "FollowerReference",
        "FollowerValidationLimits", "compute_follower_reference_digest",
        "follower_reference_digest_payload", "validate")},
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(module_name, __name__), name)
