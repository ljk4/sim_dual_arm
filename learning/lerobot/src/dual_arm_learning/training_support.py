"""Model, data-loader, validation and reporting support for ACT training."""

from __future__ import annotations

import csv
import json
import os
import time
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .config import TrainingConfig
from .dataset import configure_local_caches, file_sha256
from .sample_filter import SampleFilterMask


class TrainingError(RuntimeError):
    """Training cannot continue without violating an explicit contract."""


@dataclass(frozen=True)
class DatasetAudit:
    train_root: Path
    validation_root: Path
    train_episodes: tuple[int, ...]
    validation_episodes: tuple[int, ...]
    training_global_indices: tuple[int, ...]
    validation_global_indices: tuple[int, ...]
    train_frames: int
    validation_frames: int
    manifest_sha256: str
    verified_files: int
    synthetic: bool


def masked_l1(prediction: Any, target: Any, action_is_pad: Any) -> tuple[Any, int]:
    """Return masked L1 in normalized space and the number of valid scalars."""
    import torch

    if prediction.shape != target.shape:
        raise TrainingError(f"prediction {tuple(prediction.shape)} != target {tuple(target.shape)}")
    if action_is_pad.shape != target.shape[:2]:
        raise TrainingError("action_is_pad shape does not match prediction")
    mask = (~action_is_pad.bool()).unsqueeze(-1)
    error = torch.abs(prediction - target) * mask
    count = int(mask.sum().item()) * int(target.shape[-1])
    if count == 0:
        raise TrainingError("validation batch has no valid action scalars")
    return error.sum() / count, count


def _load_resnet_weights(policy: Any, config: TrainingConfig) -> tuple[int, str]:
    import torch

    path = config.model.weights_path
    if not path.is_file() and os.environ.get("RESNET18_WEIGHTS"):
        path = Path(os.environ["RESNET18_WEIGHTS"])
    if not path.is_file():
        raise TrainingError(f"official ResNet18 weights are missing: {path}")
    weights_sha256 = file_sha256(path)
    if config.model.weights_sha256 and weights_sha256 != config.model.weights_sha256:
        raise TrainingError("official ResNet18 weights SHA256 does not match the configuration")
    loaded = torch.load(path, map_location="cpu", weights_only=True)
    if isinstance(loaded, dict) and "state_dict" in loaded:
        loaded = loaded["state_dict"]
    if not isinstance(loaded, dict):
        raise TrainingError("ResNet18 weight file did not contain a state dictionary")
    backbone = policy.model.backbone
    expected = backbone.state_dict()
    filtered = {key: value for key, value in loaded.items() if key in expected}
    missing, unexpected = backbone.load_state_dict(filtered, strict=False)
    if missing or unexpected:
        raise TrainingError(
            "strict ResNet18 load failed: "
            f"missing={list(missing)[:4]} unexpected={list(unexpected)[:4]}"
        )
    if len(filtered) != len(expected):
        raise TrainingError("ResNet18 weight file did not cover every backbone tensor")
    return len(filtered), weights_sha256


def build_act_config(config: TrainingConfig, *, device: str | None = None) -> Any:
    """Build the upstream ACTConfig without downloading any backbone weight."""
    from lerobot.configs import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig

    inputs = {"observation.state": PolicyFeature(FeatureType.STATE, (config.dataset.state_dim,))}
    for camera in config.dataset.camera_names:
        inputs[f"observation.images.{camera}"] = PolicyFeature(
            FeatureType.VISUAL, (3, config.dataset.camera_height, config.dataset.camera_width)
        )
    outputs = {"action": PolicyFeature(FeatureType.ACTION, (config.dataset.state_dim,))}
    model = config.model
    return ACTConfig(
        input_features=inputs,
        output_features=outputs,
        chunk_size=config.dataset.chunk_size,
        n_action_steps=config.dataset.chunk_size,
        vision_backbone=model.vision_backbone,
        pretrained_backbone_weights=None,
        dim_model=model.dim_model,
        n_heads=model.n_heads,
        dim_feedforward=model.dim_feedforward,
        n_encoder_layers=model.n_encoder_layers,
        n_decoder_layers=model.n_decoder_layers,
        use_vae=model.use_vae,
        latent_dim=model.latent_dim,
        n_vae_encoder_layers=model.n_vae_encoder_layers,
        dropout=model.dropout,
        kl_weight=model.kl_weight,
        device=config.device if device is None else device,
    )


def build_policy(config: TrainingConfig) -> tuple[Any, Any, int, str]:
    from lerobot.policies.act.modeling_act import ACTPolicy

    act_config = build_act_config(config)
    policy = ACTPolicy(act_config)
    loaded, weights_sha256 = _load_resnet_weights(policy, config)
    policy.to(config.device)
    return policy, act_config, loaded, weights_sha256


def build_processors(
    config: TrainingConfig, train_dataset: Any, act_config: Any
) -> tuple[Any, Any]:
    from lerobot.policies.act.processor_act import make_act_pre_post_processors

    if not hasattr(train_dataset, "meta") or not train_dataset.meta.stats:
        raise TrainingError("training dataset has no meta.stats")
    return make_act_pre_post_processors(act_config, train_dataset.meta.stats)


def load_dataset(
    config: TrainingConfig, root: Path, episodes: Iterable[int] | None, *, training: bool
) -> Any:
    configure_local_caches(config.cache_root, data_root=config.data_root)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    delta = [index / config.dataset.fps for index in range(config.dataset.chunk_size)]
    return LeRobotDataset(
        repo_id=f"local/{config.data_root.name}-{root.name}",
        root=root,
        episodes=None if episodes is None else list(episodes),
        delta_timestamps={"action": delta},
        tolerance_s=1e-4,
        video_backend=config.dataset.video_backend,
        force_cache_sync=False,
    )


def sample_keys(config: TrainingConfig) -> tuple[str, ...]:
    keys = ["observation.state"]
    keys.extend(f"observation.images.{camera}" for camera in config.dataset.camera_names)
    keys.extend(["action", "action_is_pad"])
    return tuple(keys)


def process_batch(pre: Any, batch: dict[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    """Preprocess one already-collated batch and guarantee ACT batch dimensions."""
    filtered = {key: batch[key] for key in keys}
    processed = pre(filtered)
    if processed["action"].dim() == 2:
        processed["action"] = processed["action"].unsqueeze(0)
    if processed["action_is_pad"].dim() == 1:
        processed["action_is_pad"] = processed["action_is_pad"].unsqueeze(0)
    return processed


def loader(
    dataset: Any,
    config: TrainingConfig,
    *,
    training: bool,
    indices: Sequence[int] | None = None,
) -> Any:
    from torch.utils.data import DataLoader, SubsetRandomSampler

    if indices is not None and not training:
        raise TrainingError("a sample filter can only restrict the training loader")
    sampler = None if indices is None else SubsetRandomSampler(list(indices))
    workers = config.num_workers
    return DataLoader(
        dataset,
        batch_size=config.batch_size,
        shuffle=training and sampler is None,
        sampler=sampler,
        num_workers=workers,
        persistent_workers=workers > 0,
        pin_memory=config.pin_memory,
        prefetch_factor=config.prefetch_factor if workers > 0 else None,
        drop_last=False,
    )


def validation_loss(
    policy: Any,
    pre: Any,
    dataset: Any,
    config: TrainingConfig,
    *,
    max_batches: int,
    progress: Any = None,
) -> dict[str, Any]:
    import torch

    loader_ = loader(dataset, config, training=False)
    expected_batches = min(len(loader_), max_batches) if progress is not None else 0
    was_training = bool(policy.training)
    total = 0.0
    count = 0
    batches = 0
    policy.eval()
    try:
        with torch.no_grad():
            for batch in loader_:
                processed = process_batch(pre, batch, sample_keys(config))
                model_batch = {
                    key: value for key, value in processed.items() if key.startswith("observation.")
                }
                prediction = policy.predict_action_chunk(model_batch)
                loss, scalars = masked_l1(
                    prediction, processed["action"], processed["action_is_pad"]
                )
                total += float(loss.detach().cpu()) * scalars
                count += scalars
                batches += 1
                if progress is not None:
                    progress.detail(f"验证 {batches}/{expected_batches} 批")
                if batches >= max_batches:
                    break
    finally:
        policy.train(was_training)
    if count == 0:
        raise TrainingError("validation produced no valid action scalars")
    return {
        "val_l1": total / count,
        "val_batches": batches,
        "val_valid_scalars": count,
        "val_frames": min(len(dataset), batches * config.batch_size),
    }


METRIC_FIELDS = (
    "step",
    "phase",
    "unit",
    "loss",
    "l1_loss",
    "kl_loss",
    "lr",
    "grad_norm",
    "step_seconds",
    "elapsed_seconds",
    "vram_allocated_mb",
    "vram_reserved_mb",
    "val_l1",
    "val_batches",
    "val_valid_scalars",
    "val_frames",
)


class MetricsWriter:
    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self.jsonl = directory / "metrics.jsonl"
        self.csv = directory / "metrics.csv"
        self._csv_ready = self.csv.is_file() and self.csv.stat().st_size > 0

    def write(self, row: dict[str, Any]) -> None:
        payload = json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False)
        with self.jsonl.open("a", encoding="utf-8") as stream:
            stream.write(payload + "\n")
        if not self._csv_ready:
            with self.csv.open("w", encoding="utf-8", newline="") as stream:
                csv.DictWriter(stream, fieldnames=METRIC_FIELDS).writeheader()
            self._csv_ready = True
        with self.csv.open("a", encoding="utf-8", newline="") as stream:
            csv.DictWriter(stream, fieldnames=METRIC_FIELDS, extrasaction="ignore").writerow(row)


def make_logger(config: TrainingConfig, model_name: str) -> tuple[Any, Path]:
    from robot_run_logging import EventLogger, LogSettings

    root = config.logging.root / model_name
    root.mkdir(parents=True, exist_ok=True)
    logger_dir = root / "events"
    if logger_dir.exists():
        raise TrainingError(f"{logger_dir}: event log directory already exists")
    logger = EventLogger(
        logger_dir,
        run_id=model_name,
        component=config.logging.component,
        settings=LogSettings(
            queue_capacity=config.logging.queue_capacity,
            max_event_bytes=config.logging.max_event_bytes,
            max_file_bytes=config.logging.max_file_bytes,
            max_total_bytes=config.logging.max_total_bytes,
            close_timeout_s=config.logging.close_timeout_s,
            flush_interval_s=config.logging.flush_interval_s,
            console=config.logging.console,
        ),
    )
    return logger, root


def emit(logger: Any, event: str, **fields: Any) -> None:
    scalars = {
        key: value
        for key, value in fields.items()
        if value is None or isinstance(value, bool | int | float | str)
    }
    logger.emit(event, fields=scalars)


def new_model_name(*roots: Path) -> str:
    base = time.strftime("act_%Y%m%d_%H%M%S")
    if not roots or not any((root / base).exists() for root in roots):
        return base
    suffix = 2
    while any((root / f"{base}_{suffix}").exists() for root in roots):
        suffix += 1
    return f"{base}_{suffix}"


class TrainingInterrupted(TrainingError):
    """A bounded save was attempted after SIGINT/SIGTERM; exit is non-zero."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(f"training interrupted by {reason}")


def build_report(
    *,
    config: TrainingConfig,
    audit: DatasetAudit,
    model_name: str,
    model_dir: Path,
    reason: str,
    step: int,
    steps_total: int,
    device: str,
    policy: Any,
    loaded_backbone: int,
    started: float,
    best_step: int | None,
    best_val: Any,
    errors: list[str],
    peak_alloc: float,
    peak_reserved: float,
    sample_filter: SampleFilterMask | None = None,
    weights_sha256: str = "",
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    report = {
        "format": "dual-arm-act-training-report-v2",
        "model_name": model_name,
        "model_dir": str(model_dir),
        "mode": config.mode,
        "diagnostic": bool(config.dataset.diagnostic),
        "reason": reason,
        "steps": step,
        "planned_steps": steps_total,
        "device": device,
        "parameters": 0 if policy is None else sum(p.numel() for p in policy.parameters()),
        "loaded_backbone_tensors": loaded_backbone,
        "duration_seconds": time.monotonic() - started,
        "peak_vram_allocated_mb": peak_alloc,
        "peak_vram_reserved_mb": peak_reserved,
        "best_step": best_step,
        "best_val_l1": best_val,
        "dataset": asdict(audit)
        | {
            "train_root": str(audit.train_root),
            "validation_root": str(audit.validation_root),
        },
        "errors": errors,
        "sample_filter": None if sample_filter is None else sample_filter.summary(),
        "input_sha256": {
            "training_config": config.sha256,
            "dataset_manifest": audit.manifest_sha256,
            "profile": file_sha256(config.dataset.profile_path),
            "resnet18_weights": weights_sha256,
        },
        "limits": [
            "model selection uses validation masked L1 only",
            "physical task success is measured by the supervised campaign, not this loss",
            "synthetic fixtures never count as the 50 formal episodes",
        ],
    }
    if selection is not None:
        report["selection"] = selection
    if overrides is not None:
        report["overrides"] = overrides
    return report


__all__ = [
    "DatasetAudit",
    "METRIC_FIELDS",
    "MetricsWriter",
    "TrainingError",
    "TrainingInterrupted",
    "build_act_config",
    "build_policy",
    "build_processors",
    "build_report",
    "emit",
    "load_dataset",
    "loader",
    "make_logger",
    "masked_l1",
    "new_model_name",
    "process_batch",
    "sample_keys",
    "validation_loss",
]
