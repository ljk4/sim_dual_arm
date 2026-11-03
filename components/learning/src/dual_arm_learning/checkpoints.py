"""Named checkpoint save/load helpers for ACT training.

The rest of the trainer owns the training loop and finalization; this module
owns the trustworthy torch checkpoint format, atomic replacement, disk-space
checks and resume loading.
"""

from __future__ import annotations

import json
import os
import random
import shutil
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from .config import TrainingConfig
from .dataset import canonical_hash, file_sha256

CHECKPOINT_FORMAT = "dual-arm-act-checkpoint-v1"


class CheckpointError(RuntimeError):
    """A checkpoint cannot be written or trusted."""


def torch_load_trusted(path: Path) -> Any:
    import torch

    try:
        return torch.load(path, map_location="cpu", weights_only=False)
    except Exception as error:
        raise CheckpointError(f"{path}: cannot load trusted checkpoint: {error}") from error


def load_checkpoint(path: Path) -> dict[str, Any]:
    value = torch_load_trusted(path)
    if not isinstance(value, dict) or value.get("format") != CHECKPOINT_FORMAT:
        raise CheckpointError(f"{path}: unknown checkpoint format")
    return value


def training_binding(config: TrainingConfig) -> dict[str, Any]:
    """Bind CLI overrides and the accepted dataset, as well as YAML bytes."""
    effective = json.loads(json.dumps(asdict(config), default=str))
    if config.sample_filter is None:
        # Preserve the exact pre-filter binding so old checkpoints still resume.
        effective.pop("sample_filter", None)
    binding: dict[str, Any] = {
        "effective_configuration": effective,
        "effective_configuration_sha256": canonical_hash(effective),
        "dataset_manifest_sha256": file_sha256(config.data_root / config.manifest_name),
    }
    if config.sample_filter is not None:
        binding["sample_filter_sha256"] = file_sha256(config.sample_filter)
    return binding


def validate_checkpoint_binding(checkpoint: dict[str, Any], config: TrainingConfig) -> None:
    expected = training_binding(config)
    if checkpoint.get("training_binding") != expected:
        raise CheckpointError(
            "checkpoint effective configuration or accepted dataset changed; "
            "use the original configuration, mode and data root"
        )


def checkpoint_document(
    config: TrainingConfig,
    *,
    step: int,
    best_val: Any,
    run_id: str,
    segment_id: str,
    instance_id: str,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    document = {
        "format": CHECKPOINT_FORMAT,
        "mode": config.mode,
        "step": step,
        "seed": config.seed,
        "best_val_l1": best_val,
        "config_sha256": config.sha256,
        "training_binding": training_binding(config),
        "source": str(config.source),
        "dataset_manifest": str(config.data_root / config.manifest_name),
        "budget_run_id": run_id,
        "budget_segment_id": segment_id,
        "budget_instance_id": instance_id,
        "created_wall_time_ns": time.time_ns(),
    }
    if extra:
        document.update(extra)
    return document


def save_checkpoint(
    path: Path,
    *,
    config: TrainingConfig,
    policy: Any,
    optimizer: Any,
    scheduler: Any,
    step: int,
    best_val: Any,
    reason: str,
    run_id: str,
    segment_id: str,
    instance_id: str,
) -> dict[str, Any]:
    import torch

    path.parent.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(path.parent).free
    if free < config.checkpoint.min_free_bytes:
        raise CheckpointError(
            f"{path}: {free} free bytes is below the configured "
            f"{config.checkpoint.min_free_bytes} reserve"
        )
    rng = {
        "python": random.getstate(),
        "numpy": np.random.get_state(),
        "torch": torch.get_rng_state(),
    }
    if torch.cuda.is_available():
        rng["cuda"] = torch.cuda.get_rng_state_all()
    checkpoint = checkpoint_document(
        config,
        step=step,
        best_val=best_val,
        run_id=run_id,
        segment_id=segment_id,
        instance_id=instance_id,
        extra={
            "reason": reason,
            "model": policy.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": None if scheduler is None else scheduler.state_dict(),
            "rng": rng,
        },
    )
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        torch.save(checkpoint, temporary)
        size = temporary.stat().st_size
        if size > free - config.checkpoint.min_free_bytes:
            raise CheckpointError(f"{path}: checkpoint would consume the free-space reserve")
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": file_sha256(path),
        "step": step,
        "reason": reason,
        "run_id": run_id,
        "segment_id": segment_id,
        "instance_id": instance_id,
    }


__all__ = [
    "CHECKPOINT_FORMAT",
    "CheckpointError",
    "checkpoint_document",
    "load_checkpoint",
    "save_checkpoint",
    "torch_load_trusted",
]
