"""Cross-end trajectory data and identity validation only."""
import importlib

_EXPORTS = {
    "FollowerLimitFlags": ".follower",
    "NormalizedFollowerPositions": ".follower",
    "follower_reference_digest_payload": ".follower_reference",
    "compute_follower_reference_digest": ".follower_reference",
    "FollowerAppliedSample": ".follower_reference",
    "FollowerReference": ".follower_reference",
    "FollowerValidationLimits": ".follower_reference",
    "CubicSegment": ".spline",
    "ReferenceSample": ".spline",
    "ReferencePlan": ".spline",
    "JointExtremes": ".spline",
    "ReferenceExtremes": ".spline",
    "MAX_JOINTS": ".constants",
    "MAX_SEGMENTS": ".constants",
    "MAX_HORIZON_NS": ".constants",
    "POSITION_ATOL": ".constants",
    "VELOCITY_ATOL": ".constants",
    "ACCELERATION_ATOL": ".constants",
    "RELATIVE_TOLERANCE": ".constants",
    "LIMIT_ATOL": ".constants",
    "LIMIT_RTOL": ".constants",
    "FOLLOWER_ALGORITHM_VERSION": ".constants",
    "DIGEST_HEX_LENGTH": ".follower_reference",
    "FOLLOWER_NORMALIZATION_VERSION": ".follower_reference",
    "FOLLOWER_REFERENCE_DIGEST_VERSION": ".follower_reference",
    "FOLLOWER_REFERENCE_KINDS": ".follower_reference",
    "FOLLOWER_SAMPLER_VERSION": ".follower_reference",
}
__all__ = sorted(_EXPORTS)
def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    return getattr(importlib.import_module(_EXPORTS[name], __name__), name)
