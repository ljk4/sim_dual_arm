"""Segmentation visibility counting shared by scene checks.

This module deliberately has no MuJoCo dependency. It validates a segmentation
image and counts pixels whose ``(objid, objtype)`` pair matches a declared geom.
"""

from __future__ import annotations

__all__ = [
    "GEOM_OBJECT_TYPE",
    "VisibilityError",
    "count_visible_pixels",
]

# mjtObj.mjOBJ_GEOM is 5 in the fixed MuJoCo SDK.
GEOM_OBJECT_TYPE = 5
SEGMENTATION_OBJID_CHANNEL = 0
SEGMENTATION_OBJTYPE_CHANNEL = 1
MAX_PIXELS = 4096 * 4096


class VisibilityError(ValueError):
    """A segmentation image or visibility input is not usable."""


def count_visible_pixels(
    segmentation: object,
    geom_id: int,
    *,
    object_type: int = GEOM_OBJECT_TYPE,
) -> int:
    """Count pixels whose segmentation pair is exactly (objid, objtype)."""
    if isinstance(geom_id, bool) or not isinstance(geom_id, int) or geom_id < 0:
        raise VisibilityError("geom_id must be a non-negative integer")
    if isinstance(object_type, bool) or not isinstance(object_type, int) or object_type < 0:
        raise VisibilityError("object_type must be a non-negative integer")
    import numpy as np

    try:
        array = segmentation if isinstance(segmentation, np.ndarray) else np.asarray(segmentation)
    except ValueError as error:
        raise VisibilityError("segmentation shape is not readable") from error
    if array.ndim != 3 or array.shape[2] != 2:
        raise VisibilityError(
            "segmentation must have shape (height, width, 2) with (objid, objtype); "
            f"got {array.shape}"
        )
    height, width, _channels = array.shape
    if height <= 0 or width <= 0 or height * width > MAX_PIXELS:
        raise VisibilityError(
            f"segmentation resolution is outside the supported bound: {array.shape}"
        )
    if array.dtype.kind not in ("i", "u"):
        raise VisibilityError(f"segmentation dtype must be integer, got {array.dtype}")
    mask = (array[:, :, SEGMENTATION_OBJTYPE_CHANNEL] == object_type) & (
        array[:, :, SEGMENTATION_OBJID_CHANNEL] == geom_id
    )
    return int(np.count_nonzero(mask))
