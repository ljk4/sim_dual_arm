"""Budgeted ACT training context, step loop and checkpoint finalization."""

from __future__ import annotations

import json
import random
import signal
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .budget import BudgetLedger, BudgetTicket
from .checkpoints import load_checkpoint, save_checkpoint, validate_checkpoint_binding
from .config import TrainingConfig
from .console_progress import ConsoleProgress
from .sample_filter import SampleFilterMask
from .training_manifest import sample_filter as _sample_filter
from .training_manifest import verify_dataset_manifest
from .training_support import (
    DatasetAudit,
    MetricsWriter,
    TrainingError,
    TrainingInterrupted,
    build_policy,
    build_processors,
    build_report,
)
from .training_support import (
    emit as _emit,
)
from .training_support import (
    load_dataset as _dataset,
)
from .training_support import (
    loader as _loader,
)
from .training_support import (
    make_logger as _make_logger,
)
from .training_support import (
    new_run_id as _new_run_id,
)
from .training_support import (
    new_segment_id as _new_segment_id,
)
from .training_support import (
    prepare_dir as _prepare_dir,
)
from .training_support import (
    process_batch as _process_batch,
)
from .training_support import (
    sample_keys as _sample_keys,
)
from .training_support import (
    validation_loss as _validation,
)


@dataclass
class _TrainingContext:
    config: TrainingConfig
    audit: DatasetAudit
    sample_filter: SampleFilterMask | None
    run_id: str
    run_root: Path
    segment_id: str
    instance_id: str
    logger: Any
    metrics: MetricsWriter
    budget: BudgetLedger
    ticket: BudgetTicket | None
    started: float
    steps_total: int
    resume: dict[str, Any] | None
    progress: ConsoleProgress | None = None
    selection: dict[str, Any] | None = None
    overrides: dict[str, Any] | None = None
    stop_signal: int | None = None
    final_reason: str = "completed"
    interrupted: TrainingInterrupted | None = None
    original_error: BaseException | None = None
    save_records: list[dict[str, Any]] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    policy: Any = None
    optimizer: Any = None
    scheduler: Any = None
    pre: Any = None
    post: Any = None
    train_dataset: Any = None
    validation_dataset: Any = None
    policy_loader: Any = None
    iterator: Iterator[Any] | None = None
    step: int = 0
    best_val: float | None = None
    loaded_backbone: int = 0
    weights_sha256: str = ""
    previous_sigint: Any = None
    previous_sigterm: Any = None


def _resume_state(
    config: TrainingConfig, resume_run: Path | None
) -> tuple[str, Path, dict[str, Any] | None]:
    if resume_run is None:
        run_id = _new_run_id(config)
        run_root = config.checkpoint.root / run_id
        _prepare_dir(run_root)
        return run_id, run_root, None
    run_root = resume_run.resolve()
    if not run_root.is_dir():
        raise TrainingError(f"resume run directory is missing: {run_root}")
    latest = run_root / "latest.pt"
    if not latest.is_file():
        raise TrainingError(f"resume run has no latest.pt: {run_root}")
    checkpoint = load_checkpoint(latest)
    validate_checkpoint_binding(checkpoint, config)
    if checkpoint.get("config_sha256") != config.sha256:
        raise TrainingError("resume checkpoint was written with a different configuration")
    if checkpoint.get("mode") != config.mode:
        raise TrainingError("resume checkpoint mode differs")
    return run_root.name, run_root, checkpoint


def _create_context(
    config: TrainingConfig, allow_synthetic: bool, resume_run: Path | None, max_steps: int | None,
    *, progress: ConsoleProgress | None = None,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> _TrainingContext:
    if config.mode == "full" and allow_synthetic:
        raise TrainingError("full mode never accepts synthetic source data")
    audit = verify_dataset_manifest(
        config, allow_synthetic=allow_synthetic, require_full=config.mode == "full"
    )
    sample_filter = _sample_filter(config, audit)
    run_id, run_root, checkpoint = _resume_state(config, resume_run)
    segment_id = _new_segment_id(run_id)
    logger, _ = _make_logger(config, run_id, segment_id)
    context = _TrainingContext(
        config=config,
        audit=audit,
        sample_filter=sample_filter,
        run_id=run_id,
        run_root=run_root,
        segment_id=segment_id,
        instance_id=segment_id,
        logger=logger,
        metrics=MetricsWriter(run_root),
        budget=BudgetLedger(
            config.budget.ledger, config.budget.lock, config.budget.limit_seconds
        ),
        ticket=None,
        started=time.monotonic(),
        steps_total=(
            config.mode_spec.steps
            if max_steps is None
            else min(config.mode_spec.steps, max_steps)
        ),
        resume=checkpoint,
        progress=progress,
        selection=selection,
        overrides=overrides,
    )
    context.previous_sigint = signal.signal(signal.SIGINT, _request_stop(context))
    context.previous_sigterm = signal.signal(signal.SIGTERM, _request_stop(context))
    return context


def _request_stop(context: _TrainingContext):
    def request(signum: int, _frame: object) -> None:
        if context.stop_signal is None:
            context.stop_signal = signum

    return request


def _begin_budget(context: _TrainingContext) -> None:
    config = context.config
    mode = config.mode_spec
    context.ticket = context.budget.begin(
        run_id=context.run_id,
        segment_id=context.segment_id,
        mode=config.mode,
        reserved_seconds=mode.reserve_seconds,
        config_sha256=config.sha256,
        source=str(config.source),
        instance_id=context.instance_id,
    )
    _emit(
        context.logger,
        "training.start",
        mode=config.mode,
        seed=config.seed,
        device=config.device,
        steps=context.steps_total,
        budget_run_id=context.run_id,
        budget_segment_id=context.segment_id,
        budget_instance_id=context.instance_id,
        budget_reserved_s=mode.reserve_seconds,
        budget_used_before_s=context.ticket.used_before,
        resumed=context.resume is not None,
    )


def _seed_everything(config: TrainingConfig) -> None:
    import torch

    torch.manual_seed(config.seed)
    np.random.seed(config.seed)
    random.seed(config.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(config.seed)
    if config.deterministic:
        torch.use_deterministic_algorithms(True, warn_only=True)


def _prepare_datasets(context: _TrainingContext) -> None:
    config = context.config
    mode = config.mode_spec
    train_episodes = mode.episodes
    if train_episodes is not None:
        if max(train_episodes) >= len(context.audit.train_episodes):
            raise TrainingError("mode episode list is outside the training split")
        train_episodes = tuple(train_episodes)
    validation_episodes = tuple(range(len(context.audit.validation_episodes)))
    context.train_dataset = _dataset(
        config, context.audit.train_root, train_episodes, training=True
    )
    if context.sample_filter is not None:
        if len(context.train_dataset) != context.sample_filter.source_frames:
            raise TrainingError("filtered training dataset size differs from the sample filter")
        if max(context.sample_filter.eligible_indices) >= len(context.train_dataset):
            raise TrainingError("sample filter contains an out-of-range anchor")
    if mode.validation:
        context.validation_dataset = _dataset(
            config, context.audit.validation_root, validation_episodes, training=False
        )


def _restore_checkpoint(context: _TrainingContext) -> None:
    import torch

    checkpoint = context.resume
    if checkpoint is None:
        return
    context.policy.load_state_dict(checkpoint["model"], strict=True)
    context.optimizer.load_state_dict(checkpoint["optimizer"])
    if checkpoint.get("scheduler") is not None:
        context.scheduler.load_state_dict(checkpoint["scheduler"])
    rng = checkpoint.get("rng") or {}
    if "python" in rng:
        random.setstate(rng["python"])
    if "numpy" in rng:
        np.random.set_state(rng["numpy"])
    if "torch" in rng:
        torch.set_rng_state(rng["torch"])
    if torch.cuda.is_available() and "cuda" in rng:
        torch.cuda.set_rng_state_all(rng["cuda"])
    context.step = int(checkpoint.get("step", 0))
    context.best_val = checkpoint.get("best_val_l1")


def _prepare_model(context: _TrainingContext) -> None:
    import torch

    config = context.config
    context.policy, act_config, context.loaded_backbone, context.weights_sha256 = build_policy(
        config
    )
    context.pre, context.post = build_processors(config, context.train_dataset, act_config)
    context.optimizer = torch.optim.AdamW(
        context.policy.parameters(), lr=config.lr, weight_decay=config.weight_decay
    )
    context.scheduler = torch.optim.lr_scheduler.ConstantLR(
        context.optimizer, factor=1.0, total_iters=max(1, context.steps_total)
    )
    _restore_checkpoint(context)


def _emit_model_ready(context: _TrainingContext) -> None:
    config = context.config
    sample_filter = context.sample_filter
    mode = config.mode_spec
    train_episodes = mode.episodes
    if train_episodes is not None:
        train_episodes = tuple(train_episodes)
    _emit(
        context.logger,
        "training.model_ready",
        batch_size=config.batch_size,
        loaded_backbone_tensors=context.loaded_backbone,
        weights_sha256=context.weights_sha256,
        train_frames=len(context.train_dataset),
        train_source_frames=len(context.train_dataset),
        train_eligible_frames=(
            len(context.train_dataset)
            if sample_filter is None
            else len(sample_filter.eligible_indices)
        ),
        train_excluded_frames=(0 if sample_filter is None else sample_filter.excluded_frames),
        sample_filter_digest=("" if sample_filter is None else sample_filter.digest),
        sample_filter_sha256=("" if sample_filter is None else sample_filter.file_sha256),
        validation_frames=(
            0 if context.validation_dataset is None else len(context.validation_dataset)
        ),
        train_episodes=len(context.audit.train_episodes)
        if train_episodes is None
        else len(train_episodes),
        validation_episodes=(
            0 if context.validation_dataset is None else len(context.audit.validation_episodes)
        ),
        parameters=sum(parameter.numel() for parameter in context.policy.parameters()),
        resumed=context.resume is not None,
    )


def _prepare_training_context(context: _TrainingContext) -> None:
    config = context.config
    _begin_budget(context)
    if context.progress is not None:
        context.progress.message(f"训练目录：{context.run_root}")
        context.progress.phase("初始化模型和数据")
    _seed_everything(config)
    _prepare_datasets(context)
    _prepare_model(context)
    _emit_model_ready(context)
    context.policy_loader = _loader(
        context.train_dataset,
        config,
        training=True,
        indices=(
            None if context.sample_filter is None else context.sample_filter.eligible_indices
        ),
    )
    context.iterator = iter(context.policy_loader)
    context.policy.train()
    if context.progress is not None:
        context.progress.phase("训练", context.steps_total, unit="step", initial=context.step)


def _signal_reason(context: _TrainingContext) -> TrainingInterrupted:
    reason = "sigterm" if context.stop_signal == signal.SIGTERM else "sigint"
    return TrainingInterrupted(reason)


def _amp_settings(config: TrainingConfig) -> tuple[bool, Any, Any]:
    import torch

    enabled = bool(
        config.use_amp and config.device.startswith("cuda") and torch.cuda.is_available()
    )
    dtype = torch.bfloat16 if config.amp_dtype == "bfloat16" else torch.float16
    scaler = torch.amp.GradScaler(
        "cuda", enabled=enabled and config.amp_dtype == "float16"
    )
    return enabled, dtype, scaler


def _train_step(context: _TrainingContext, amp: tuple[bool, Any, Any]) -> dict[str, Any]:
    import torch

    config = context.config
    if context.stop_signal is not None:
        raise _signal_reason(context)
    assert context.ticket is not None
    context.ticket.check(config.budget.stop_margin_seconds)
    try:
        batch = next(context.iterator)
    except StopIteration:
        context.iterator = iter(context.policy_loader)
        batch = next(context.iterator)
    step_started = time.monotonic()
    processed = _process_batch(context.pre, batch, _sample_keys(config))
    context.optimizer.zero_grad(set_to_none=True)
    amp_enabled, amp_dtype, scaler = amp
    with torch.amp.autocast(device_type="cuda", dtype=amp_dtype, enabled=amp_enabled):
        loss, loss_metrics = context.policy.forward(processed)
    if not torch.isfinite(loss):
        raise TrainingError(f"non-finite training loss at step {context.step}")
    scaler.scale(loss).backward()
    scaler.unscale_(context.optimizer)
    grad_norm = torch.nn.utils.clip_grad_norm_(
        context.policy.parameters(), config.grad_clip_norm
    )
    if not torch.isfinite(grad_norm):
        raise TrainingError(f"non-finite gradient norm at step {context.step}")
    scaler.step(context.optimizer)
    scaler.update()
    context.scheduler.step()
    context.step += 1
    if context.stop_signal is not None:
        raise _signal_reason(context)
    return {
        "loss": float(loss.detach().cpu()),
        "l1_loss": float(loss_metrics.get("l1_loss", 0.0)),
        "kl_loss": float(loss_metrics.get("kld_loss", 0.0)),
        "grad_norm": float(grad_norm.detach().cpu()),
        "step_seconds": time.monotonic() - step_started,
    }


def _step_row(context: _TrainingContext, metrics: dict[str, Any]) -> dict[str, Any]:
    import torch

    vram_alloc = vram_reserved = 0.0
    if torch.cuda.is_available():
        vram_alloc = torch.cuda.max_memory_allocated() / (1024 * 1024)
        vram_reserved = torch.cuda.max_memory_reserved() / (1024 * 1024)
    assert context.ticket is not None
    return {
        "step": context.step,
        "phase": "train",
        "unit": "step",
        **metrics,
        "lr": float(context.optimizer.param_groups[0]["lr"]),
        "elapsed_seconds": time.monotonic() - context.started,
        "vram_allocated_mb": vram_alloc,
        "vram_reserved_mb": vram_reserved,
        "budget_remaining_s": context.ticket.remaining_seconds,
    }


def _save_checkpoint(context: _TrainingContext, kind: str, reason: str) -> None:
    if context.progress is not None:
        context.progress.detail(f"保存 {kind} 检查点")
    record = save_checkpoint(
        context.run_root / f"{kind}.pt",
        config=context.config,
        policy=context.policy,
        optimizer=context.optimizer,
        scheduler=context.scheduler,
        step=context.step,
        best_val=context.best_val,
        reason=reason,
        run_id=context.run_id,
        segment_id=context.segment_id,
        instance_id=context.instance_id,
    )
    context.save_records.append(record)
    _emit(context.logger, "training.checkpoint", path=record["path"], step=context.step)


def _maybe_checkpoint(context: _TrainingContext) -> None:
    config = context.config
    due = context.step % config.save_every_steps == 0 or context.step == context.steps_total
    if due and config.checkpoint.keep_latest:
        reason = "periodic" if context.step % config.save_every_steps == 0 else "final"
        _save_checkpoint(context, "latest", reason)


def _maybe_validate(context: _TrainingContext) -> None:
    config = context.config
    if context.validation_dataset is None:
        return
    if not (context.step % config.val_every_steps == 0 or context.step == context.steps_total):
        return
    validation = _validation(
        context.policy,
        context.pre,
        context.validation_dataset,
        config,
        max_batches=config.val_batches,
        progress=context.progress,
    )
    validation.update({"step": context.step, "phase": "validation", "unit": "step"})
    context.metrics.write(validation)
    _emit(context.logger, "training.validation", **validation)
    if context.best_val is None or validation["val_l1"] < context.best_val:
        context.best_val = validation["val_l1"]
        if config.checkpoint.keep_best:
            _save_checkpoint(context, "best", "best_validation")


def _run_training_steps(context: _TrainingContext) -> None:
    config = context.config
    amp = _amp_settings(config)
    while context.step < context.steps_total:
        metrics = _train_step(context, amp)
        if context.progress is not None:
            context.progress.update(context.step, detail=f"loss={metrics['loss']:.5f}")
        if context.step % config.log_every_steps == 0 or context.step == 1:
            row = _step_row(context, metrics)
            context.metrics.write(row)
            _emit(context.logger, "training.step", **row)
        _maybe_checkpoint(context)
        _maybe_validate(context)
        if context.progress is not None:
            detail = f"loss={metrics['loss']:.5f}"
            if context.best_val is not None:
                detail += f" · best_val={context.best_val:.5f}"
            context.progress.detail(detail)
        assert context.ticket is not None
        if context.ticket.remaining_seconds <= config.budget.stop_margin_seconds:
            context.final_reason = "budget_near_exhausted"
            break
    if context.stop_signal is not None and context.final_reason == "completed":
        raise _signal_reason(context)


def _save_final_checkpoints(context: _TrainingContext) -> None:
    config = context.config
    if context.policy is None or context.optimizer is None or context.scheduler is None:
        return
    if config.checkpoint.keep_latest:
        try:
            _save_checkpoint(context, "latest", context.final_reason)
        except BaseException as error:
            context.errors.append(
                f"latest checkpoint save failed: {type(error).__name__}: {error}"
            )
    if config.checkpoint.keep_final:
        try:
            _save_checkpoint(context, "final", context.final_reason)
        except BaseException as error:
            context.errors.append(
                f"final checkpoint save failed: {type(error).__name__}: {error}"
            )


def _close_logging(context: _TrainingContext) -> None:
    event_reason = "failed" if context.errors else context.final_reason
    if context.original_error is not None:
        try:
            context.logger.emit(
                "training.exception",
                level="ERROR",
                exception=context.original_error,
                fields={"reason": context.final_reason, "step": context.step},
            )
        except BaseException as error:
            context.errors.append(f"exception event failed: {type(error).__name__}: {error}")
    else:
        try:
            _emit(
                context.logger,
                "training.end",
                reason=event_reason,
                step=context.step,
                segment_id=context.segment_id,
                instance_id=context.instance_id,
            )
        except BaseException as error:
            context.errors.append(f"training.end event failed: {type(error).__name__}: {error}")
    try:
        context.logger.close()
    except BaseException as error:
        context.errors.append(f"logger close failed: {type(error).__name__}: {error}")


def _peak_vram() -> tuple[float, float]:
    import torch

    if not torch.cuda.is_available():
        return 0.0, 0.0
    return (
        torch.cuda.max_memory_allocated() / (1024 * 1024),
        torch.cuda.max_memory_reserved() / (1024 * 1024),
    )


def _report_payload(context: _TrainingContext) -> dict[str, Any]:
    config = context.config
    reason = "failed" if context.errors else context.final_reason
    peak_alloc, peak_reserved = _peak_vram()
    return build_report(
        config=config,
        audit=context.audit,
        run_id=context.run_id,
        segment_id=context.segment_id,
        instance_id=context.instance_id,
        reason=reason,
        step=context.step,
        steps_total=context.steps_total,
        device=config.device,
        policy=context.policy,
        loaded_backbone=context.loaded_backbone,
        started=context.started,
        best_val=context.best_val,
        save_records=context.save_records,
        errors=context.errors,
        peak_alloc=peak_alloc,
        peak_reserved=peak_reserved,
        sample_filter=context.sample_filter,
        weights_sha256=context.weights_sha256,
        selection=context.selection,
        overrides=context.overrides,
    )


def _write_report(context: _TrainingContext, report: dict[str, Any]) -> None:
    try:
        payload = json.dumps(report, indent=2, allow_nan=False, default=str) + "\n"
        (context.run_root / f"training_report-{context.segment_id}.json").write_text(
            payload, encoding="utf-8"
        )
        (context.run_root / "training_report.json").write_text(payload, encoding="utf-8")
    except BaseException as error:
        context.errors.append(f"training report write failed: {type(error).__name__}: {error}")


def _finish_budget(context: _TrainingContext, report: dict[str, Any]) -> None:
    if context.ticket is None:
        return
    reason_for_report = "failed" if context.errors else context.final_reason
    try:
        context.ticket.finish(reason_for_report, reason=context.final_reason)
    except BaseException as error:
        context.errors.append(f"budget finish failed: {type(error).__name__}: {error}")
        report["reason"] = "failed"
        report["errors"] = list(context.errors)
        try:
            payload = json.dumps(report, indent=2, allow_nan=False, default=str) + "\n"
            (context.run_root / f"training_report-{context.segment_id}.json").write_text(payload)
            (context.run_root / "training_report.json").write_text(payload)
        except OSError as report_error:
            context.errors.append(f"failure report update failed: {report_error}")


def _restore_signals(context: _TrainingContext) -> None:
    signal.signal(signal.SIGINT, context.previous_sigint)
    signal.signal(signal.SIGTERM, context.previous_sigterm)


def _finalize_training(context: _TrainingContext) -> dict[str, Any]:
    _save_final_checkpoints(context)
    _close_logging(context)
    report = _report_payload(context)
    _write_report(context, report)
    _finish_budget(context, report)
    _restore_signals(context)
    return report


def _raise_training_outcome(context: _TrainingContext, report: dict[str, Any]) -> None:
    if context.errors:
        message = "; ".join(context.errors)
        if context.original_error is not None:
            context.original_error.add_note(message)
            raise context.original_error
        raise TrainingError(message)
    if context.original_error is not None:
        raise context.original_error
    if context.interrupted is not None:
        raise context.interrupted




__all__ = [
    "_TrainingContext",
    "_create_context",
    "_finalize_training",
    "_prepare_training_context",
    "_raise_training_outcome",
    "_run_training_steps",
]
