"""Generic immutable task state shared by every simulation task plugin."""

from __future__ import annotations

import json
from typing import Annotated, Any

from pydantic import Field, model_validator

from .models import RunContext
from .validation import NonEmptyStr, NonNegativeInt, StrictContractModel, contract_error

__all__ = ["TaskState"]

_INT64_MAX = 2**63 - 1


class TaskState(StrictContractModel):
    """One immutable task fact; task-specific values live in ``metrics``."""

    context: RunContext | None = None
    state_seq: Annotated[NonNegativeInt, Field(le=_INT64_MAX)]
    sample_time_ns: Annotated[NonNegativeInt, Field(le=_INT64_MAX)]
    task_epoch: Annotated[NonNegativeInt, Field(le=_INT64_MAX)]
    task_type: NonEmptyStr
    phase: NonEmptyStr
    succeeded: bool = False
    terminal: bool = False
    failure_reason: str = Field(default="", max_length=512)
    plan_digest: str = ""
    task_config_digest: str = ""
    metrics: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _terminal(self) -> TaskState:
        if self.succeeded and not self.terminal:
            raise contract_error("succeeded", "a succeeded task must be terminal")
        if self.failure_reason and self.succeeded:
            raise contract_error("failure_reason", "a succeeded task cannot carry a failure")
        return self

    def metric(self, name: str, default: Any = None) -> Any:
        return self.metrics.get(name, default)

    def metrics_json(self) -> str:
        """Return the canonical JSON object used on the ROS wire."""
        try:
            return json.dumps(
                self.metrics, sort_keys=True, separators=(",", ":"), allow_nan=False
            )
        except (TypeError, ValueError) as error:
            raise ValueError("task metrics are not canonical JSON") from error
