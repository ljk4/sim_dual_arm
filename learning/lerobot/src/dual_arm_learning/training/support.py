"""Model, data-loader, validation and reporting support for ACT training."""

from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from ..config import TrainingConfig, VisualAugmentationSpec
from ..dataset.common import configure_local_caches, file_sha256
from ..dataset.sample_filter import SampleFilterMask


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
        optimizer_lr=config.lr,
        optimizer_lr_backbone=(config.lr if config.lr_backbone is None else config.lr_backbone),
        device=config.device if device is None else device,
    )


def build_optimizer_param_groups(policy: Any, config: TrainingConfig) -> list[dict[str, Any]]:
    """Split visual backbone parameters into their lower learning-rate group when configured."""
    if config.lr_backbone is None:
        return [{"params": [p for p in policy.parameters() if p.requires_grad], "lr": config.lr}]
    head_params = []
    backbone_params = []
    for name, param in policy.named_parameters():
        if not param.requires_grad:
            continue
        if name.startswith("model.backbone"):
            backbone_params.append(param)
        else:
            head_params.append(param)
    if not backbone_params:
        return [{"params": head_params, "lr": config.lr}]
    groups: list[dict[str, Any]] = []
    if head_params:
        groups.append({"params": head_params, "lr": config.lr})
    groups.append({"params": backbone_params, "lr": config.lr_backbone})
    return groups


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


def _augment_single_camera(images: Any, spec: VisualAugmentationSpec) -> Any:
    import torch
    import torch.nn.functional as F

    batch_size, _channels, height, width = images.shape
    dev = images.device
    x = images

    def _factor(lo: float, hi: float) -> Any:
        return torch.empty((batch_size, 1, 1, 1), device=dev, dtype=torch.float32).uniform_(lo, hi)

    if spec.brightness > 0.0:
        x = (x * _factor(1.0 - spec.brightness, 1.0 + spec.brightness)).clamp_(0.0, 1.0)
    if spec.contrast > 0.0:
        luma = 0.2989 * x[:, 0:1] + 0.5870 * x[:, 1:2] + 0.1140 * x[:, 2:3]
        gray_mean = luma.mean(dim=(-2, -1), keepdim=True)
        factor = _factor(1.0 - spec.contrast, 1.0 + spec.contrast)
        x = (x * factor + gray_mean * (1.0 - factor)).clamp_(0.0, 1.0)
    if spec.saturation > 0.0:
        gray = 0.2989 * x[:, 0:1] + 0.5870 * x[:, 1:2] + 0.1140 * x[:, 2:3]
        factor = _factor(1.0 - spec.saturation, 1.0 + spec.saturation)
        x = (x * factor + gray * (1.0 - factor)).clamp_(0.0, 1.0)
    if spec.hue > 0.0:
        max_angle = spec.hue * 2.0 * math.pi
        theta = torch.empty((batch_size,), device=dev, dtype=torch.float32).uniform_(
            -max_angle, max_angle
        )
        cos_t = theta.cos().view(batch_size, 1, 1)
        sin_t = theta.sin().view(batch_size, 1, 1)
        eye = torch.eye(3, device=dev, dtype=torch.float32).unsqueeze(0)
        uu = torch.full((1, 3, 3), 1.0 / 3.0, device=dev, dtype=torch.float32)
        inv_sqrt3 = 1.0 / math.sqrt(3.0)
        skew = torch.tensor(
            [[0.0, -inv_sqrt3, inv_sqrt3], [inv_sqrt3, 0.0, -inv_sqrt3], [-inv_sqrt3, inv_sqrt3, 0.0]],
            device=dev,
            dtype=torch.float32,
        ).unsqueeze(0)
        rot = cos_t * eye + (1.0 - cos_t) * uu + sin_t * skew
        x = torch.einsum("bij,bjhw->bihw", rot, x).clamp_(0.0, 1.0)
    if spec.blur_prob > 0.0:
        blur_mask = torch.rand((batch_size,), device=dev) < spec.blur_prob
        if blur_mask.any():
            idx = blur_mask.nonzero(as_tuple=False).squeeze(-1)
            n_blur = int(idx.numel())
            sigma = torch.empty((n_blur, 1), device=dev, dtype=torch.float32).uniform_(0.2, 1.0)
            coords = torch.tensor([-1.0, 0.0, 1.0], device=dev, dtype=torch.float32).view(1, 3)
            k1d = torch.exp(-0.5 * (coords / sigma) ** 2)
            k1d = k1d / k1d.sum(dim=-1, keepdim=True)
            k2d = k1d.unsqueeze(2) * k1d.unsqueeze(1)
            weight = k2d.unsqueeze(1).expand(n_blur, 3, 3, 3).reshape(n_blur * 3, 1, 3, 3).contiguous()
            pad_mode = "reflect" if min(height, width) > 1 else "replicate"
            padded = F.pad(x.index_select(0, idx), (1, 1, 1, 1), mode=pad_mode)
            blurred = F.conv2d(
                padded.reshape(1, n_blur * 3, padded.shape[2], padded.shape[3]),
                weight,
                groups=n_blur * 3,
            ).reshape(n_blur, 3, height, width).clamp_(0.0, 1.0)
            x[idx] = blurred
    if spec.noise_std > 0.0:
        x = (x + torch.randn_like(x) * spec.noise_std).clamp_(0.0, 1.0)
    return x


def augment_visual_batch(
    batch: dict[str, Any],
    spec: VisualAugmentationSpec,
    *,
    device: str | None = None,
) -> dict[str, Any]:
    """Apply spatial-preserving photometric, blur, noise and single-camera dropout."""
    if not spec.enabled:
        return batch
    import torch

    image_keys = [
        key
        for key, value in batch.items()
        if key.startswith("observation.images.")
        and isinstance(value, torch.Tensor)
        and value.ndim == 4
        and value.shape[1] == 3
    ]
    if not image_keys:
        return batch
    target_device = (
        torch.device(device)
        if device is not None and device.startswith("cuda") and torch.cuda.is_available()
        else None
    )
    augmented = dict(batch)
    for key in image_keys:
        tensor = batch[key]
        if target_device is not None and tensor.device != target_device:
            x = tensor.to(target_device, dtype=torch.float32, non_blocking=True)
        else:
            x = tensor.to(dtype=torch.float32).clone()
        augmented[key] = _augment_single_camera(x, spec)

    if spec.camera_dropout_prob > 0.0:
        ref = augmented[image_keys[0]]
        batch_size = int(ref.shape[0])
        drop_mask = torch.rand((batch_size,), device=ref.device) < spec.camera_dropout_prob
        if drop_mask.any():
            chosen = torch.randint(0, len(image_keys), (batch_size,), device=ref.device)
            for cam_idx, key in enumerate(image_keys):
                cam_mask = drop_mask & (chosen == cam_idx)
                if cam_mask.any():
                    augmented[key][cam_mask] = 0.0
    return augmented


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
    data_loader: Any = None,
) -> dict[str, Any]:
    import torch

    loader_ = loader(dataset, config, training=False) if data_loader is None else data_loader
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


class MetricsWriter:
    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self.jsonl = directory / "metrics.jsonl"

    def write(self, row: dict[str, Any]) -> None:
        payload = json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False)
        with self.jsonl.open("a", encoding="utf-8") as stream:
            stream.write(payload + "\n")


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
        "model_name": model_name,
        "model_dir": str(model_dir),
        "mode": config.mode,
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
        ],
    }
    if selection is not None:
        report["selection"] = selection
    if overrides is not None:
        report["overrides"] = overrides
    return report


__all__ = [
    "DatasetAudit",
    "MetricsWriter",
    "TrainingError",
    "TrainingInterrupted",
    "augment_visual_batch",
    "build_act_config",
    "build_optimizer_param_groups",
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
