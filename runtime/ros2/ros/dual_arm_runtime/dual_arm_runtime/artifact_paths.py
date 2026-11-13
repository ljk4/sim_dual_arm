"""Single source for ``var/<scenario-id>`` runtime artifact roots.

Every runtime output lives under ``<base>/<scenario-id>/{datasets,models,logs,
diagnostics}`` (container ``/data/<scenario-id>/...``).  Direct configuration
mode has no scenario id and uses the reserved ``_direct`` segment instead of
silently falling back to a real scenario.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

DIRECT_SEGMENT = "_direct"
_SCENARIO_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")


def scenario_segment(scenario: str | None) -> str:
    """Return the safe directory segment for one scenario selection."""
    if scenario is None:
        return DIRECT_SEGMENT
    if not _SCENARIO_PATTERN.fullmatch(scenario) or ".." in scenario:
        raise ValueError(
            "scenario id must contain 1..128 safe ASCII characters and no path traversal"
        )
    return scenario


def container_data_root() -> Path:
    return Path(os.environ.get("DATA_ROOT", "/data"))


def scenario_root(base: Path, scenario: str | None) -> Path:
    return Path(base) / scenario_segment(scenario)


def logs_root(base: Path, scenario: str | None) -> Path:
    return scenario_root(base, scenario) / "logs"


def diagnostics_root(base: Path, scenario: str | None) -> Path:
    return scenario_root(base, scenario) / "diagnostics"


def runs_root(scenario: str | None) -> Path:
    return logs_root(container_data_root(), scenario) / "runs"


def recordings_root(scenario: str | None) -> Path:
    return logs_root(container_data_root(), scenario) / "recordings"


def default_backend_log(scenario: str | None) -> Path:
    return logs_root(container_data_root(), scenario) / "backend" / "teacher-backend" / "log"


__all__ = [
    "DIRECT_SEGMENT",
    "container_data_root",
    "default_backend_log",
    "diagnostics_root",
    "logs_root",
    "recordings_root",
    "runs_root",
    "scenario_root",
    "scenario_segment",
]
