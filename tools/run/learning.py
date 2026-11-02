#!/usr/bin/env python3
"""Public learning entry point: convert, train, budget and bundle commands.

Source paths are explicit: this candidate's learning/run_logging packages and
the pinned read-only LeRobot source.  The ROS runtime, its NumPy 1 environment
and ``rclpy`` are never added to this process.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
_CANDIDATE_PATHS = (
    ROOT / "components/learning/src",
    ROOT / "libraries/run_logging/src",
)
for _path in _CANDIDATE_PATHS:
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

_SOURCE = os.environ.get(
    "LEARNING_SOURCE",
    str(ROOT / "third_party/src/lerobot-act-v0.6.1/src"),
)
if Path(_SOURCE).is_dir() and _SOURCE not in sys.path:
    sys.path.insert(0, _SOURCE)

from dual_arm_learning.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
