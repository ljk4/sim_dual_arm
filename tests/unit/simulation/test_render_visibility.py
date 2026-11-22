from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.render import visibility  # noqa: E402


def test_pixel_count_uses_exact_objid_and_objtype_not_colors():
    segmentation = [
        [[7, visibility.GEOM_OBJECT_TYPE], [7, visibility.GEOM_OBJECT_TYPE], [8, 5]],
        [[7, 1], [4, 2], [7, visibility.GEOM_OBJECT_TYPE]],
    ]
    assert visibility.count_visible_pixels(segmentation, 7) == 3
    assert visibility.count_visible_pixels(segmentation, 8) == 1
    assert visibility.count_visible_pixels(segmentation, 4) == 0

    with pytest.raises(visibility.VisibilityError, match=r"shape .*2\)|2-channel"):
        visibility.count_visible_pixels([[[1, 2, 3]]], 1)
    with pytest.raises(visibility.VisibilityError, match="integer"):
        visibility.count_visible_pixels([[[1.0, 5.0]]], 1)
