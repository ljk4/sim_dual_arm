"""Small shared audit helpers used by the one-shot S2 checks.

These functions have no scene, rendering or ROS dependency.  Keeping them in
one place avoids the new kinematics entry copying the accepted scene-check
provenance and atomic-write logic.
"""

from __future__ import annotations

import hashlib
import json
from datetime import UTC, datetime
from pathlib import Path

from robot_contracts.io import sha256_file, write_json_atomic

__all__ = [
    "code_provenance",
    "python_source_manifest",
    "sha256_file",
    "utc_now",
    "write_json_atomic",
]


def utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def python_source_manifest(directory: Path) -> dict[str, object]:
    """Hash a flat source package independently of its mount location."""
    files = {path.name: sha256_file(path) for path in sorted(directory.glob("*.py"))}
    if not files:
        raise OSError(f"no Python sources in {directory}")
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {
        "directory": str(directory),
        "files": files,
        "sha256": hashlib.sha256(encoded).hexdigest(),
    }


def code_provenance() -> dict[str, object]:
    import robot_run_logging

    return {
        "robot_backend_sim": python_source_manifest(Path(__file__).resolve().parents[2]),
        "robot_run_logging": python_source_manifest(
            Path(robot_run_logging.__file__).resolve().parent
        ),
    }
