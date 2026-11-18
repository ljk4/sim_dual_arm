"""Segmentation visibility counting shared by scene checks.

This module deliberately has no MuJoCo dependency.  It validates a
segmentation image and counts pixels whose ``(objid, objtype)`` pair matches a
declared geom.  Scene-specific layout configuration lives with the scenario.
"""

from __future__ import annotations

import math

__all__ = [
    "GEOM_OBJECT_TYPE",
    "VisibilityError",
    "count_visible_pixels",
    "yaw_quaternion",
]

# mjtObj.mjOBJ_GEOM is 5 in the fixed MuJoCo SDK.  Keeping the value here lets
# the pure-python checks run on a host without MuJoCo.
GEOM_OBJECT_TYPE = 5
# MuJoCo's Renderer returns a segmentation image whose last axis is
# (objid, objtype); column order is part of the SDK contract, not a guess.
SEGMENTATION_OBJID_CHANNEL = 0
SEGMENTATION_OBJTYPE_CHANNEL = 1
MAX_PIXELS = 4096 * 4096


class VisibilityError(ValueError):
    """A segmentation image or visibility input is not usable."""


def yaw_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """Return the world Z yaw quaternion in MuJoCo's wxyz order."""
    if isinstance(yaw_deg, bool) or not isinstance(yaw_deg, int | float):
        raise VisibilityError("yaw_deg must be a finite number")
    yaw = math.radians(float(yaw_deg))
    if not math.isfinite(yaw):
        raise VisibilityError("yaw_deg must be finite")
    half = yaw / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _shape(segmentation: object) -> tuple[int, int, int]:
    shape = getattr(segmentation, "shape", None)
    if shape is not None:
        try:
            dimensions = tuple(int(value) for value in shape)
        except (TypeError, ValueError) as error:
            raise VisibilityError("segmentation shape is not readable") from error
    else:
        try:
            dimensions = (
                len(segmentation),  # type: ignore[arg-type]
                len(segmentation[0]),  # type: ignore[index]
                len(segmentation[0][0]),  # type: ignore[index]
            )
        except (TypeError, IndexError) as error:
            raise VisibilityError("segmentation must be a 3-dimensional image") from error
    if len(dimensions) != 3 or dimensions[2] != 2:
        raise VisibilityError(
            "segmentation must have shape (height, width, 2) with (objid, objtype); "
            f"got {dimensions}"
        )
    height, width, _channels = dimensions
    if height <= 0 or width <= 0 or height * width > MAX_PIXELS:
        raise VisibilityError(
            f"segmentation resolution is outside the supported bound: {dimensions}"
        )
    return height, width, 2


def _is_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


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
    _shape(segmentation)
    try:
        import numpy as np
    except ImportError:
        np = None  # type: ignore[assignment]
    if np is not None and isinstance(segmentation, np.ndarray):
        if segmentation.ndim != 3 or segmentation.shape[2] != 2:
            raise VisibilityError("segmentation must be a 2-channel image")
        if segmentation.dtype.kind not in ("i", "u"):
            raise VisibilityError(f"segmentation dtype must be integer, got {segmentation.dtype}")
        mask = (segmentation[:, :, SEGMENTATION_OBJTYPE_CHANNEL] == object_type) & (
            segmentation[:, :, SEGMENTATION_OBJID_CHANNEL] == geom_id
        )
        return int(np.count_nonzero(mask))
    height, width, _channels = _shape(segmentation)
    count = 0
    for row in range(height):
        row_values = segmentation[row]  # type: ignore[index]
        if len(row_values) != width:
            raise VisibilityError("segmentation rows have inconsistent widths")
        for column in range(width):
            cell = row_values[column]
            if len(cell) != 2:
                raise VisibilityError("segmentation pixels must contain (objid, objtype)")
            obj_id = cell[SEGMENTATION_OBJID_CHANNEL]
            obj_type = cell[SEGMENTATION_OBJTYPE_CHANNEL]
            if not _is_integer(obj_id) or not _is_integer(obj_type):
                raise VisibilityError("segmentation ids and types must be integers")
            if obj_id == geom_id and obj_type == object_type:
                count += 1
    return count
