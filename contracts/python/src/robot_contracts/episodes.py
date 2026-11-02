"""Immutable episode result values and explicit manifest version selection.

This module is deliberately independent of ROS, ML and the runtime package so
that offline tools, tests and future consumers share one vocabulary for the three
independent result dimensions introduced by episode manifest v2.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

from robot_contracts.models import ContractError

EPISODE_SCHEMA_VERSION = 2
NATIVE_EPISODE_SCHEMA_VERSION = 3
POLICY_EPISODE_SCHEMA_VERSION = 4
SUPPORTED_EPISODE_SCHEMA_VERSIONS = (1, 2, 3, 4)
_VERSION_DIRECTORIES = {1: "v1", 2: "v2", 3: "v3", 4: "v4"}


class RuntimeOutcome(StrEnum):
    """What happened to the runtime workflow, never whether the task succeeded."""

    COMPLETED = "completed"
    STOPPED = "stopped"
    INTERRUPTED = "interrupted"
    FAILED = "failed"


class TaskOutcome(StrEnum):
    """Task-level judgement; mock collection currently never evaluates it."""

    NOT_EVALUATED = "not_evaluated"
    SUCCEEDED = "succeeded"
    TIMED_OUT = "timed_out"
    FAILED = "failed"


@dataclass(frozen=True)
class RecordingResult:
    """Recording integrity dimension of one finished episode."""

    complete: bool
    failure_reason: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.complete, bool):
            raise TypeError("recording complete must be a boolean")
        if not isinstance(self.failure_reason, str):
            raise TypeError("recording failure reason must be a string")
        if self.complete and self.failure_reason:
            raise ValueError("a complete recording cannot carry a failure reason")
        if not self.complete and not self.failure_reason:
            raise ValueError("an incomplete recording must carry a reason")


@dataclass(frozen=True)
class RuntimeResult:
    """Runtime dimension of one finished episode."""

    outcome: RuntimeOutcome
    reason: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, RuntimeOutcome):
            raise TypeError("runtime outcome must be a RuntimeOutcome")
        if not isinstance(self.reason, str):
            raise TypeError("runtime reason must be a string")
        if self.outcome is RuntimeOutcome.COMPLETED and self.reason:
            raise ValueError("a completed runtime cannot carry a failure reason")
        if self.outcome is not RuntimeOutcome.COMPLETED and not self.reason:
            raise ValueError("a non-completed runtime must carry a reason")

    @classmethod
    def completed(cls) -> RuntimeResult:
        return cls(RuntimeOutcome.COMPLETED, "")


@dataclass(frozen=True)
class TaskResult:
    """Task dimension of one finished episode."""

    outcome: TaskOutcome = TaskOutcome.NOT_EVALUATED
    reason: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, TaskOutcome):
            raise TypeError("task outcome must be a TaskOutcome")
        if not isinstance(self.reason, str):
            raise TypeError("task reason must be a string")


def manifest_schema_path(schema_root: Path, version: int) -> Path:
    """Select the episode manifest schema by explicit version mapping.

    ``schema_root`` is normally ``contracts/schemas``.  A version-specific
    directory (``.../v1`` .. ``.../v4``) is accepted only when it matches the
    requested version; user data never supplies a path fragment.
    """
    if isinstance(version, bool) or version not in _VERSION_DIRECTORIES:
        raise ContractError(
            "UNSUPPORTED_SCHEMA_VERSION",
            f"episode manifest version {version!r} is not supported",
        )
    directory = _VERSION_DIRECTORIES[version]
    if schema_root.name in _VERSION_DIRECTORIES.values():
        if schema_root.name != directory:
            raise ContractError(
                "UNSUPPORTED_SCHEMA_VERSION",
                f"schema directory {schema_root.name} does not match version {version}",
            )
        return schema_root / "episode_manifest.schema.json"
    return schema_root / directory / "episode_manifest.schema.json"
