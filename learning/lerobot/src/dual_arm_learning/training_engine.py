"""ACT training context, step loop and single-file model finalization."""

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

from .bundle import build_schema, save_model_package
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
    new_model_name,
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
    model_name: str
    model_dir: Path
    log_root: Path
    logger: Any
    metrics: MetricsWriter
    started: float
    steps_total: int
    progress: ConsoleProgress | None = None
    selection: dict[str, Any] | None = None
    overrides: dict[str, Any] | None = None
    stop_signal: int | None = None
    final_reason: str = "completed"
    interrupted: TrainingInterrupted | None = None
    original_error: BaseException | None = None
    errors: list[str] = field(default_factory=list)
    policy: Any = None
    act_config: Any = None
    optimizer: Any = None
    scheduler: Any = None
    pre: Any = None
    post: Any = None
    train_dataset: Any = None
    validation_dataset: Any = None
    policy_loader: Any = None
    iterator: Iterator[Any] | None = None
    step: int = 0
    best_step: int | None = None
    best_val: float | None = None
    model_saved: bool = False
    loaded_backbone: int = 0
    weights_sha256: str = ""
    previous_sigint: Any = None
    previous_sigterm: Any = None


def _create_context(
    config: TrainingConfig,
    allow_synthetic: bool,
    max_steps: int | None,
    *,
    progress: ConsoleProgress | None = None,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> _TrainingContext:
    if config.mode == "full" and allow_synthetic:
        raise TrainingError("full mode never accepts synthetic source data")
    audit = verify_dataset_manifest(
        config, allow_synthetic=allow_synthetic, require_full=config.mode == "full"
    )
    sample_filter = _sample_filter(config, audit)
    model_name = new_model_name(config.checkpoint.root, config.logging.root)
    model_dir = config.checkpoint.root / model_name
    logger, log_root = _make_logger(config, model_name)
    context = _TrainingContext(
        config=config,
        audit=audit,
        sample_filter=sample_filter,
        model_name=model_name,
        model_dir=model_dir,
        log_root=log_root,
        logger=logger,
        metrics=MetricsWriter(log_root),
        started=time.monotonic(),
        steps_total=(
            config.mode_spec.steps if max_steps is None else min(config.mode_spec.steps, max_steps)
        ),
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


def _prepare_model(context: _TrainingContext) -> None:
    import torch

    config = context.config
    context.policy, context.act_config, context.loaded_backbone, context.weights_sha256 = (
        build_policy(config)
    )
    context.pre, context.post = build_processors(
        config, context.train_dataset, context.act_config
    )
    context.optimizer = torch.optim.AdamW(
        context.policy.parameters(), lr=config.lr, weight_decay=config.weight_decay
    )
    context.scheduler = torch.optim.lr_scheduler.ConstantLR(
        context.optimizer, factor=1.0, total_iters=max(1, context.steps_total)
    )


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
        sample_filter_sha256=("" if sample_filter is None else sample_filter.source_sha256),
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
    )


def _prepare_training_context(context: _TrainingContext) -> None:
    config = context.config
    if context.progress is not None:
        context.progress.message(f"模型目录：{context.model_dir}")
        context.progress.phase("初始化模型和数据")
    _seed_everything(config)
    _prepare_datasets(context)
    _prepare_model(context)
    _emit_model_ready(context)
    context.policy_loader = _loader(
        context.train_dataset,
        config,
        training=True,
        indices=(None if context.sample_filter is None else context.sample_filter.eligible_indices),
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
    scaler = torch.amp.GradScaler("cuda", enabled=enabled and config.amp_dtype == "float16")
    return enabled, dtype, scaler


def _train_step(context: _TrainingContext, amp: tuple[bool, Any, Any]) -> dict[str, Any]:
    import torch

    config = context.config
    if context.stop_signal is not None:
        raise _signal_reason(context)
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
    grad_norm = torch.nn.utils.clip_grad_norm_(context.policy.parameters(), config.grad_clip_norm)
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
    return {
        "step": context.step,
        "phase": "train",
        "unit": "step",
        **metrics,
        "lr": float(context.optimizer.param_groups[0]["lr"]),
        "elapsed_seconds": time.monotonic() - context.started,
        "vram_allocated_mb": vram_alloc,
        "vram_reserved_mb": vram_reserved,
    }


def _training_args(context: _TrainingContext) -> dict[str, Any]:
    config = context.config
    selection = context.selection or {}
    model_values = config.model.model_dump(mode="json")
    model_values.pop("weights_path", None)
    training_values = {
        "batch_size": config.batch_size,
        "num_workers": config.num_workers,
        "pin_memory": config.pin_memory,
        "prefetch_factor": config.prefetch_factor,
        "use_amp": config.use_amp,
        "amp_dtype": config.amp_dtype,
        "lr": config.lr,
        "weight_decay": config.weight_decay,
        "grad_clip_norm": config.grad_clip_norm,
        "log_every_steps": config.log_every_steps,
        "val_every_steps": config.val_every_steps,
        "val_batches": config.val_batches,
        "deterministic": config.deterministic,
    }
    return {
        "model_name": "act",
        "trained_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "robot": selection.get("robot", ""),
        "task": selection.get("task", ""),
        "mode": config.mode,
        "seed": config.seed,
        "steps_completed": context.step,
        "best_step": context.best_step,
        "best_val_l1": context.best_val,
        "elapsed_seconds": time.monotonic() - context.started,
        "dataset_root": str(config.data_root),
        "model_hyperparameters": model_values,
        "training_hyperparameters": training_values,
    }


def _schema(context: _TrainingContext) -> tuple[dict, dict]:
    profile = json.loads(context.config.dataset.profile_path.read_text(encoding="utf-8"))
    return build_schema(context.config, profile)


def _save_model(context: _TrainingContext, *, step: int) -> None:
    if context.policy is None or context.act_config is None:
        return
    if context.progress is not None:
        context.progress.detail("保存 model.safetensors")
    observation, action = _schema(context)
    save_model_package(
        context.model_dir,
        policy=context.policy,
        act_config=context.act_config,
        dataset_root=context.config.data_root,
        step=step,
        synthetic=context.audit.synthetic,
        config_sha256=context.config.sha256,
        observation_schema=observation,
        action_schema=action,
        training_args=_training_args(context),
    )
    context.model_saved = True
    _emit(context.logger, "training.model", path=str(context.model_dir), step=step)


def _write_training_args(context: _TrainingContext) -> None:
    if not (context.model_dir / "model.safetensors").is_file():
        return
    import yaml

    (context.model_dir / "training_args.yaml").write_text(
        yaml.safe_dump(_training_args(context), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


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
        context.best_step = context.step
        _save_model(context, step=context.step)


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
        _maybe_validate(context)
        if context.progress is not None:
            detail = f"loss={metrics['loss']:.5f}"
            if context.best_val is not None:
                detail += f" · best_val={context.best_val:.5f}"
            context.progress.detail(detail)
    if context.stop_signal is not None and context.final_reason == "completed":
        raise _signal_reason(context)


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
        model_name=context.model_name,
        model_dir=context.model_dir,
        reason=reason,
        step=context.step,
        steps_total=context.steps_total,
        device=config.device,
        policy=context.policy,
        loaded_backbone=context.loaded_backbone,
        started=context.started,
        best_step=context.best_step,
        best_val=context.best_val,
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
        (context.log_root / "training_report.json").write_text(payload, encoding="utf-8")
    except BaseException as error:
        context.errors.append(f"training report write failed: {type(error).__name__}: {error}")


def _restore_signals(context: _TrainingContext) -> None:
    signal.signal(signal.SIGINT, context.previous_sigint)
    signal.signal(signal.SIGTERM, context.previous_sigterm)


def _save_final_model(context: _TrainingContext) -> None:
    if (context.model_dir / "model.safetensors").is_file():
        return
    if context.final_reason == "failed":
        return
    if context.policy is None or context.act_config is None:
        return
    try:
        _save_model(context, step=context.step)
    except BaseException as error:
        context.errors.append(f"model save failed: {type(error).__name__}: {error}")


def _finalize_training(context: _TrainingContext) -> dict[str, Any]:
    _save_final_model(context)
    try:
        _write_training_args(context)
    except BaseException as error:
        context.errors.append(f"training args write failed: {type(error).__name__}: {error}")
    _close_logging(context)
    report = _report_payload(context)
    _write_report(context, report)
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
