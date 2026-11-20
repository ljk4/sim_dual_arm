"""Immutable episode results and the canonical manifest schema path."""

from __future__ import annotations

from enum import StrEnum
from pathlib import Path

from pydantic import model_validator

from .validation import StrictContractModel, contract_error


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


class RecordingResult(StrictContractModel):
    """Recording integrity dimension of one finished episode."""

    complete: bool
    failure_reason: str = ""

    @model_validator(mode="after")
    def _consistent(self) -> RecordingResult:
        if self.complete and self.failure_reason:
            raise contract_error("failure_reason", "a complete recording cannot carry a reason")
        if not self.complete and not self.failure_reason:
            raise contract_error("failure_reason", "an incomplete recording must carry a reason")
        return self


class RuntimeResult(StrictContractModel):
    """Runtime dimension of one finished episode."""

    outcome: RuntimeOutcome
    reason: str = ""

    @model_validator(mode="after")
    def _consistent(self) -> RuntimeResult:
        completed = self.outcome is RuntimeOutcome.COMPLETED
        if completed and self.reason:
            raise contract_error("reason", "a completed runtime cannot carry a failure reason")
        if not completed and not self.reason:
            raise contract_error("reason", "a non-completed runtime must carry a reason")
        return self

    @classmethod
    def completed(cls) -> RuntimeResult:
        return cls(RuntimeOutcome.COMPLETED, "")


class TaskResult(StrictContractModel):
    """Task dimension of one finished episode."""

    outcome: TaskOutcome = TaskOutcome.NOT_EVALUATED
    reason: str = ""


def manifest_schema_path(schema_root: Path) -> Path:
    """Return the episode manifest schema in the shared schema directory."""
    return schema_root / "episode_manifest.schema.json"
