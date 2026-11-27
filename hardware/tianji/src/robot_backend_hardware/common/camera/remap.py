"""Camera undistortion and centered MuJoCo-pinhole FOV remapping."""
from __future__ import annotations

import math
from typing import Sequence

import cv2
import numpy as np
from pydantic import Field

from robot_backend_hardware.common.config import DeviceSettings


class PinholeRemapConfig(DeviceSettings):
    """Configuration for undistorting and resampling camera frames to simulation pinhole."""

    enabled: bool = True
    target_width: int = Field(default=320, gt=0)
    target_height: int = Field(default=240, gt=0)
    target_fovy_deg: float = Field(gt=1.0, lt=179.0)


def build_pinhole_matrix(width: int, height: int, fovy_deg: float) -> np.ndarray:
    """Build a centered pinhole camera matrix matching MuJoCo vertical FOV."""
    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid target image dimensions: {width}x{height}")
    if not (1.0 < fovy_deg < 179.0):
        raise ValueError(f"target_fovy_deg out of range (1, 179): {fovy_deg}")
    fy = float(height) / (2.0 * math.tan(math.radians(float(fovy_deg)) / 2.0))
    return np.array(
        [
            [fy, 0.0, float(width) / 2.0],
            [0.0, fy, float(height) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


class PinholeRemapper:
    """Precomputes OpenCV undistort+rectify maps to match a centered MuJoCo pinhole."""

    def __init__(
        self,
        source_camera_matrix: np.ndarray | Sequence[Sequence[float]],
        distortion_coeffs: np.ndarray | Sequence[float] | None,
        *,
        target_width: int,
        target_height: int,
        target_fovy_deg: float,
    ) -> None:
        K_src = np.asarray(source_camera_matrix, dtype=np.float64)
        if K_src.shape != (3, 3):
            raise ValueError(f"Expected 3x3 source_camera_matrix, got {K_src.shape}")
        if distortion_coeffs is None:
            dist = np.zeros(5, dtype=np.float64)
        else:
            dist = np.asarray(list(distortion_coeffs), dtype=np.float64).reshape(-1)
            if dist.size == 0:
                dist = np.zeros(5, dtype=np.float64)

        self._source_camera_matrix = K_src
        self._distortion_coeffs = dist
        self._target_width = int(target_width)
        self._target_height = int(target_height)
        self._target_fovy_deg = float(target_fovy_deg)
        self._target_camera_matrix = build_pinhole_matrix(
            self._target_width,
            self._target_height,
            self._target_fovy_deg,
        )
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self._source_camera_matrix,
            self._distortion_coeffs,
            np.eye(3, dtype=np.float64),
            self._target_camera_matrix,
            (self._target_width, self._target_height),
            cv2.CV_16SC2,
        )

    @property
    def target_width(self) -> int:
        return self._target_width

    @property
    def target_height(self) -> int:
        return self._target_height

    @property
    def target_fovy_deg(self) -> float:
        return self._target_fovy_deg

    @property
    def target_camera_matrix(self) -> np.ndarray:
        return self._target_camera_matrix.copy()

    def remap(self, frame: np.ndarray) -> np.ndarray:
        """Undistort and resample a frame into the configured pinhole output geometry."""
        remapped = cv2.remap(
            frame,
            self._map1,
            self._map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        return np.ascontiguousarray(remapped)

    def describe_intrinsics(self, base: dict) -> dict:
        """Return updated camera intrinsics metadata reflecting the remapped output."""
        updated = dict(base)
        updated["raw_width"] = base.get("width")
        updated["raw_height"] = base.get("height")
        updated["raw_camera_matrix"] = base.get("camera_matrix")
        updated["raw_distortion_coefficients"] = base.get("distortion_coefficients")
        updated["width"] = self._target_width
        updated["height"] = self._target_height
        updated["fovy_deg"] = self._target_fovy_deg
        updated["calibration_status"] = "pinhole_remapped"
        updated["camera_matrix"] = self._target_camera_matrix.tolist()
        updated["distortion_model"] = "pinhole_remapped"
        updated["distortion_coefficients"] = [0.0, 0.0, 0.0, 0.0, 0.0]
        return updated
