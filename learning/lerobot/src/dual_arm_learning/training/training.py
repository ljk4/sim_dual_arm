"""Public ACT training entry point."""

from __future__ import annotations

from typing import Any

from ..console_progress import ConsoleProgress
from .engine import (
    _create_context,
    _finalize_training,
    _prepare_training_context,
    _raise_training_outcome,
    _run_training_steps,
    _TrainingContext,
)
from .manifest import verify_dataset_manifest
from .support import (  # noqa: F401 - public re-exports
    DatasetAudit,
    TrainingError,
    TrainingInterrupted,
    build_act_config,
    build_policy,
    build_processors,
    build_report,
    loader,
    masked_l1,
    validation_loss,
)


def train(
    config: Any,
    *,
    allow_synthetic: bool = False,
    max_steps: int | None = None,
    progress: ConsoleProgress | None = None,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Run one mode and write one self-contained model package."""
    context: _TrainingContext = _create_context(
        config,
        allow_synthetic,
        max_steps,
        progress=progress,
        selection=selection,
        overrides=overrides,
    )
    try:
        _prepare_training_context(context)
        _run_training_steps(context)
    except TrainingInterrupted as error:
        context.final_reason = f"{error.reason}_save_requested"
        context.interrupted = error
    except BaseException as error:
        context.final_reason = "failed"
        context.original_error = error
    finally:
        if progress is not None:
            progress.phase("保存模型与训练报告")
        report = _finalize_training(context)
    _raise_training_outcome(context, report)
    return report


__all__ = [
    "DatasetAudit",
    "TrainingError",
    "TrainingInterrupted",
    "build_act_config",
    "build_policy",
    "build_processors",
    "build_report",
    "loader",
    "masked_l1",
    "train",
    "validation_loss",
    "verify_dataset_manifest",
]
