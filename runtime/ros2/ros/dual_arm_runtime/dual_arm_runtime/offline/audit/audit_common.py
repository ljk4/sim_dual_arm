"""Shared bounds, camera validation and numeric helpers for MCAP audits."""

from __future__ import annotations

import math

from robot_contracts.models import ContractError

_MAX_FEEDBACK_PER_COMMAND = 4096
_MAX_METADATA_ENTRIES = 1_000_000
_MAX_COMMANDS = 4096
_MAX_EVENTS = 100_000
_TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
_TERMINAL_EVENT_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})


def record_error(message: str) -> ContractError:
    return ContractError("RECORDING_ERROR", message)


def close(left: float, right: float, tolerance: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=tolerance)


def vectors_close(left: tuple[float, ...], right: tuple[float, ...], tolerance: float) -> bool:
    return len(left) == len(right) and all(
        close(a, b, tolerance) for a, b in zip(left, right, strict=True)
    )


def validate_camera_frame(message: object, camera: object) -> None:
    """Validate one recorded camera frame against its declared profile entry."""
    name = camera.name  # type: ignore[attr-defined]
    image = message
    if (image.width, image.height) != (camera.width, camera.height):  # type: ignore[attr-defined]
        raise record_error(f"Camera dimensions differ: {name}")
    if image.encoding != "rgb8" or len(image.data) != image.width * image.height * 3:
        raise record_error(f"Invalid RGB frame: {name}")
    if image.step != image.width * 3:
        raise record_error(f"Camera RGB step differs: {name}")
    if image.frame_id != camera.frame_id:  # type: ignore[attr-defined]
        raise record_error(f"Camera frame_id differs: {name}")
    camera_info = message
    if (camera_info.info_width, camera_info.info_height) != (camera.width, camera.height):  # type: ignore[attr-defined]
        raise record_error(f"CameraInfo dimensions differ: {name}")
    if camera_info.info_frame_id != camera.frame_id:  # type: ignore[attr-defined]
        raise record_error(f"CameraInfo frame_id differs: {name}")
    if message.calibration_digest != camera.calibration_digest:  # type: ignore[attr-defined]
        raise record_error(f"Camera calibration differs: {name}")
