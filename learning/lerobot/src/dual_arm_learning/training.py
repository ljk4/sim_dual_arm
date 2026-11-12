"""Public ACT training entry point and compatibility re-exports.

Dataset admission lives in :mod:`training_manifest`; the budgeted context, AMP
step loop and finalization live in :mod:`training_engine`; model, loader,
validation and report support lives in :mod:`training_support`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .budget import BudgetExceeded
from .console_progress import ConsoleProgress
from .training_engine import (
    _create_context,
    _finalize_training,
    _prepare_training_context,
    _raise_training_outcome,
    _run_training_steps,
    _TrainingContext,
)
from .training_manifest import (  # noqa: F401 - private re-export used by tests
    sample_filter as _sample_filter,
)
from .training_manifest import (
    verify_dataset_manifest,
)
from .training_support import (  # noqa: F401 - public re-exports
    BUNDLE_FORMAT,
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

# Private aliases kept for callers that patch the historical names.
_loader = loader
_validation = validation_loss


def train(
    config: Any,
    *,
    allow_synthetic: bool = False,
    resume_run: Path | None = None,
    max_steps: int | None = None,
    progress: ConsoleProgress | None = None,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Run one mode with a stable run id and one unique budget segment."""
    context: _TrainingContext = _create_context(
        config,
        allow_synthetic,
        resume_run,
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
    except BudgetExceeded as error:
        context.final_reason = "budget_exhausted"
        if context.ticket is None:
            context.original_error = error
    except BaseException as error:
        context.final_reason = "failed"
        context.original_error = error
    finally:
        if progress is not None:
            progress.phase("保存检查点与训练报告")
        report = _finalize_training(context)
    _raise_training_outcome(context, report)
    return report


__all__ = [
    "DatasetAudit",
    "TrainingError",
    "build_act_config",
    "masked_l1",
    "train",
    "verify_dataset_manifest",
]
