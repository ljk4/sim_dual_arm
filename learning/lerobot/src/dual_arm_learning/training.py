"""Public ACT training entry point.

Dataset admission lives in :mod:`training_manifest`; the training context and
step loop live in :mod:`training_engine`; model, loader, validation and report
support lives in :mod:`training_support`.
"""

from __future__ import annotations

from typing import Any

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
    "build_act_config",
    "masked_l1",
    "train",
    "verify_dataset_manifest",
]
