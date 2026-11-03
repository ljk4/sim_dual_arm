"""ACT smoke/overfit/full training with an append-only wall-clock budget.

This module is deliberately lazy about Torch, LeRobot and CUDA imports.  The
configuration and dataset-audit functions remain usable in a small environment;
the GPU training path is entered only after the budget reservation exists.
"""

from __future__ import annotations

import csv
import json
import os
import random
import signal
import time
import uuid
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .budget import BudgetExceeded, BudgetLedger, BudgetTicket
from .checkpoints import load_checkpoint, save_checkpoint, validate_checkpoint_binding
from .config import TrainingConfig
from .dataset import canonical_hash, configure_local_caches, file_sha256, tree_hashes
from .sample_filter import SampleFilterMask, load_filter_mask

BUNDLE_FORMAT = "dual-arm-act-bundle-v1"


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


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise TrainingError(f"{path}: cannot read JSON: {error}") from error
    if not isinstance(value, dict):
        raise TrainingError(f"{path}: expected an object")
    return value


def _episode_indices(split: dict[str, Any]) -> list[int]:
    episodes = split.get("episodes")
    if not isinstance(episodes, list):
        raise TrainingError("manifest split episodes must be an array")
    result: list[int] = []
    for entry in episodes:
        if not isinstance(entry, dict):
            raise TrainingError("manifest episode must be an object")
        index = entry.get("global_success_index")
        if isinstance(index, bool) or not isinstance(index, int):
            raise TrainingError("manifest episode global_success_index must be an integer")
        result.append(index)
    return result


def verify_dataset_manifest(
    config: TrainingConfig,
    *,
    allow_synthetic: bool = False,
    require_full: bool | None = None,
) -> DatasetAudit:
    """Check 45/5 disjointness, fixed validation slots and every declared hash."""
    root = config.data_root
    manifest_path = root / config.manifest_name
    manifest = _load_json(manifest_path)
    if manifest.get("status") != "complete":
        raise TrainingError(f"{manifest_path}: dataset manifest is not complete")
    synthetic = bool(manifest.get("synthetic"))
    if synthetic and (not allow_synthetic or require_full):
        raise TrainingError("refusing synthetic data for the requested training mode")
    splits = manifest.get("splits")
    if not isinstance(splits, dict):
        raise TrainingError("manifest.splits must be an object")
    training = splits.get("training")
    validation = splits.get("validation")
    if not isinstance(training, dict) or not isinstance(validation, dict):
        raise TrainingError("manifest.splits must contain training and validation")
    if training.get("episode_count") != config.dataset.expected_training_episodes:
        raise TrainingError("training split does not contain exactly 45 episodes")
    if validation.get("episode_count") != config.dataset.expected_validation_episodes:
        raise TrainingError("validation split does not contain exactly 5 episodes")
    train_indices = _episode_indices(training)
    validation_indices = _episode_indices(validation)
    if set(train_indices) & set(validation_indices):
        raise TrainingError("training and validation splits overlap")
    if set(validation_indices) != set(config.dataset.validation_positions):
        raise TrainingError("validation split is not at success positions 9/19/29/39/49")
    if set(train_indices) | set(validation_indices) != set(range(50)):
        raise TrainingError("splits do not partition the first 50 successes")
    if len(train_indices) != 45 or len(validation_indices) != 5:
        raise TrainingError("split cardinality is not 45/5")
    profile = manifest.get("profile")
    if not isinstance(profile, dict) or profile.get("semantic_sha256") != canonical_hash(
        _load_json(config.dataset.profile_path)
    ):
        raise TrainingError("manifest profile hash does not match the configured learning profile")
    if profile.get("profile_id") != _load_json(config.dataset.profile_path).get("profile_id"):
        raise TrainingError("manifest profile id does not match")
    if manifest.get("lerobot", {}).get("source_revision") != config.dataset.lerobot_source_revision:
        raise TrainingError("manifest LeRobot revision does not match the pinned source")
    if manifest.get("lerobot", {}).get("source_archive_sha256") != (
        config.dataset.lerobot_source_archive_sha256
    ):
        raise TrainingError("manifest LeRobot source archive does not match")
    processors = manifest.get("processors")
    if not isinstance(processors, dict) or processors.get("training_stats_only") is not True:
        raise TrainingError("manifest does not prove train-only processor statistics")
    fact_records: list[dict[str, Any]] = []
    for split_name, split in (("training", training), ("validation", validation)):
        split_root = root / str(split.get("root", ""))
        if not split_root.is_dir():
            raise TrainingError(f"{split_name} dataset root is missing: {split_root}")
        info = _load_json(split_root / "meta/info.json")
        if info.get("total_episodes") != split.get("episode_count"):
            raise TrainingError(f"{split_name} meta total_episodes disagrees with manifest")
        features = info.get("features")
        if not isinstance(features, dict):
            raise TrainingError(f"{split_name} meta features are missing")
        for camera in config.dataset.camera_names:
            key = f"observation.images.{camera}"
            if key not in features or features[key].get("dtype") != "video":
                raise TrainingError(f"{split_name} is missing video feature {key}")
        if features.get("action", {}).get("shape") != [config.dataset.state_dim]:
            raise TrainingError(f"{split_name} action shape is not 16")
        if features.get("observation.state", {}).get("shape") != [config.dataset.state_dim]:
            raise TrainingError(f"{split_name} observation.state shape is not 16")
        if not (split_root / "meta/stats.json").is_file():
            raise TrainingError(f"{split_name} meta/stats.json is missing")
        if config.verify_hashes:
            declared = split.get("files")
            if not isinstance(declared, dict) or not declared:
                raise TrainingError(f"{split_name} manifest has no file hash inventory")
            actual = tree_hashes(split_root)
            if actual != declared:
                missing = sorted(set(declared) - set(actual))
                extra = sorted(set(actual) - set(declared))
                changed = sorted(
                    name for name in set(actual) & set(declared) if actual[name] != declared[name]
                )
                raise TrainingError(
                    f"{split_name} dataset file hash mismatch: missing={missing[:4]} "
                    f"extra={extra[:4]} changed={changed[:4]}"
                )
        for episode in split.get("episodes", []):
            if not isinstance(episode, dict):
                raise TrainingError(f"{split_name} episode entry is not an object")
            facts = episode.get("source_facts")
            if not isinstance(facts, dict):
                raise TrainingError(f"{split_name} episode has no source task/runtime facts")
            stored_facts_hash = facts.get("facts_sha256")
            payload = {key: value for key, value in facts.items() if key != "facts_sha256"}
            if stored_facts_hash != canonical_hash(payload):
                raise TrainingError(f"{split_name} source facts hash mismatch")
            if (
                payload.get("task", {}).get("outcome") != "succeeded"
                or payload.get("task", {}).get("reason") != "TASK_SUCCEEDED"
            ):
                raise TrainingError(f"{split_name} source task did not succeed")
            if (
                payload.get("runtime", {}).get("outcome") not in {"stopped", "completed"}
                or payload.get("runtime", {}).get("reason") != "TASK_SUCCEEDED"
            ):
                raise TrainingError(f"{split_name} source runtime did not stop successfully")
            if payload.get("recording", {}).get("complete") is not True:
                raise TrainingError(f"{split_name} source recording is incomplete")
            if payload.get("quality_qualified") is not True:
                raise TrainingError(f"{split_name} source quality is not qualified")
            fact_records.append(
                {
                    "attempt_id": episode.get("attempt_id"),
                    "seed": episode.get("seed"),
                    "facts": payload,
                }
            )
            if config.verify_hashes:
                files = episode.get("source_files")
                if not isinstance(files, dict):
                    raise TrainingError(f"{split_name} episode has no source file hashes")
                for _name, record in files.items():
                    if (
                        not isinstance(record, dict)
                        or "path" not in record
                        or "sha256" not in record
                    ):
                        raise TrainingError(f"{split_name} source hash record is malformed")
                    source = Path(str(record["path"]))
                    if not source.is_file() or file_sha256(source) != record["sha256"]:
                        raise TrainingError(f"{split_name} source export hash mismatch: {source}")
    if manifest.get("source_facts_sha256") != canonical_hash(fact_records):
        raise TrainingError("dataset manifest source task/runtime facts hash mismatch")
    if require_full or (require_full is None and config.mode == "full"):
        for split_name, split in (("training", training), ("validation", validation)):
            for episode in split.get("episodes", []):
                if int(episode.get("frame_count", 0)) < config.dataset.chunk_size:
                    raise TrainingError(
                        f"full training refuses {split_name} episode shorter than one action chunk"
                    )
    return DatasetAudit(
        train_root=root / str(training["root"]),
        validation_root=root / str(validation["root"]),
        train_episodes=tuple(range(len(train_indices))),
        validation_episodes=tuple(range(len(validation_indices))),
        training_global_indices=tuple(train_indices),
        validation_global_indices=tuple(validation_indices),
        train_frames=int(training.get("frame_count", 0)),
        validation_frames=int(validation.get("frame_count", 0)),
        manifest_sha256=file_sha256(manifest_path),
        verified_files=(
            len(training.get("files", {})) + len(validation.get("files", {}))
            if config.verify_hashes
            else 0
        ),
        synthetic=synthetic,
    )


def _sample_filter(config: TrainingConfig, audit: DatasetAudit) -> SampleFilterMask | None:
    if config.sample_filter is None:
        return None
    if config.mode != "full":
        raise TrainingError("data.sample_filter is only supported by full training")
    mask = load_filter_mask(config.sample_filter)
    if mask.document["dataset"]["manifest"]["sha256"] != audit.manifest_sha256:
        raise TrainingError("sample filter was generated from a different dataset manifest")
    if mask.source_frames != audit.train_frames:
        raise TrainingError("sample filter source frame count differs from the training split")
    return mask


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


def _load_resnet_weights(policy: Any, config: TrainingConfig) -> int:
    import torch

    path = config.model.weights_path
    if not path.is_file() and os.environ.get("RESNET18_WEIGHTS"):
        path = Path(os.environ["RESNET18_WEIGHTS"])
    if not path.is_file():
        raise TrainingError(f"official ResNet18 weights are missing: {path}")
    if file_sha256(path) != config.model.weights_sha256:
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
    return len(filtered)


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


def _build_policy(config: TrainingConfig) -> tuple[Any, Any, Any, Any, int]:
    from lerobot.policies.act.modeling_act import ACTPolicy

    act_config = build_act_config(config)
    policy = ACTPolicy(act_config)
    loaded = _load_resnet_weights(policy, config)
    policy.to(config.device)
    # Processors are built from the training split's meta.stats below.
    return policy, act_config, None, None, loaded


def _build_processors(
    config: TrainingConfig, train_dataset: Any, act_config: Any
) -> tuple[Any, Any]:
    from lerobot.policies.act.processor_act import make_act_pre_post_processors

    if not hasattr(train_dataset, "meta") or not train_dataset.meta.stats:
        raise TrainingError("training dataset has no meta.stats")
    return make_act_pre_post_processors(act_config, train_dataset.meta.stats)


def _dataset(
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


def _sample_keys(config: TrainingConfig) -> tuple[str, ...]:
    keys = ["observation.state"]
    keys.extend(f"observation.images.{camera}" for camera in config.dataset.camera_names)
    keys.extend(["action", "action_is_pad"])
    return tuple(keys)


def _process_sample(pre: Any, sample: dict[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    filtered = {key: sample[key] for key in keys}
    processed = pre(filtered)
    # The upstream default pipeline batches observations but leaves the action
    # trajectory as (chunk, dim).  ACT.forward expects (batch, chunk, dim).
    if processed["action"].dim() == 2:
        processed["action"] = processed["action"].unsqueeze(0)
    if processed["action_is_pad"].dim() == 1:
        processed["action_is_pad"] = processed["action_is_pad"].unsqueeze(0)
    return processed


def _collate(processed: list[dict[str, Any]]) -> dict[str, Any]:
    import torch

    result: dict[str, Any] = {}
    for key in processed[0]:
        values = [item[key] for item in processed]
        if all(torch.is_tensor(value) for value in values):
            result[key] = torch.cat(values, dim=0)
        else:
            result[key] = values
    return result


def _loader(
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
    return DataLoader(
        dataset,
        batch_size=config.batch_size,
        shuffle=training and sampler is None,
        sampler=sampler,
        num_workers=config.num_workers,
        collate_fn=list,
        persistent_workers=config.num_workers > 0,
        pin_memory=False,
        drop_last=False,
    )


def _validation(
    policy: Any,
    pre: Any,
    dataset: Any,
    config: TrainingConfig,
    *,
    max_batches: int,
) -> dict[str, Any]:
    import torch

    loader = _loader(dataset, config, training=False)
    was_training = bool(policy.training)
    total = 0.0
    count = 0
    batches = 0
    policy.eval()
    try:
        with torch.no_grad():
            for samples in loader:
                processed = _collate(
                    [_process_sample(pre, sample, _sample_keys(config)) for sample in samples]
                )
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
    "budget_remaining_s",
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


def _make_logger(config: TrainingConfig, run_id: str) -> tuple[Any, Path]:
    from robot_run_logging import EventLogger, LogSettings

    root = config.logging.root / run_id
    root.mkdir(parents=True, exist_ok=True)
    logger_dir = root / "events"
    if logger_dir.exists():
        raise TrainingError(f"{logger_dir}: event log directory already exists")
    logger = EventLogger(
        logger_dir,
        run_id=run_id,
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


def _emit(logger: Any, event: str, **fields: Any) -> None:
    scalars = {
        key: value
        for key, value in fields.items()
        if value is None or isinstance(value, bool | int | float | str)
    }
    logger.emit(event, fields=scalars)


def _new_run_id(config: TrainingConfig) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return f"{config.mode}-{stamp}-{os.getpid()}"


def _prepare_dir(path: Path) -> None:
    if path.exists():
        raise TrainingError(f"{path}: run directory already exists; use --resume-run")
    path.mkdir(parents=True, exist_ok=False)


def _new_segment_id(run_id: str) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return f"{run_id}.segment.{stamp}.{os.getpid()}.{uuid.uuid4().hex[:8]}"


def _make_logger(config: TrainingConfig, run_id: str, segment_id: str) -> tuple[Any, Path]:
    from robot_run_logging import EventLogger, LogSettings

    root = config.logging.root / run_id
    root.mkdir(parents=True, exist_ok=True)
    logger_dir = root / f"events-{segment_id}"
    if logger_dir.exists():
        raise TrainingError(f"{logger_dir}: event log directory already exists")
    logger = EventLogger(
        logger_dir,
        run_id=run_id,
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


class TrainingInterrupted(TrainingError):
    """A bounded save was attempted after SIGINT/SIGTERM; exit is non-zero."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(f"training interrupted by {reason}")


def _report(
    *,
    config: TrainingConfig,
    audit: DatasetAudit,
    run_id: str,
    segment_id: str,
    instance_id: str,
    reason: str,
    step: int,
    steps_total: int,
    device: str,
    policy: Any,
    loaded_backbone: int,
    started: float,
    best_val: Any,
    save_records: list[dict[str, Any]],
    errors: list[str],
    peak_alloc: float,
    peak_reserved: float,
    sample_filter: SampleFilterMask | None = None,
) -> dict[str, Any]:
    return {
        "format": "dual-arm-act-training-report-v1",
        "run_id": run_id,
        "budget_run_id": run_id,
        "budget_segment_id": segment_id,
        "budget_instance_id": instance_id,
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
        "best_val_l1": best_val,
        "dataset": asdict(audit)
        | {
            "train_root": str(audit.train_root),
            "validation_root": str(audit.validation_root),
        },
        "checkpoints": save_records,
        "errors": errors,
        "sample_filter": None if sample_filter is None else sample_filter.summary(),
        "limits": [
            "model selection uses validation masked L1 only",
            "physical stacking success is measured by the supervised campaign, not this loss",
            "synthetic fixtures never count as the 50 formal episodes",
        ],
    }


def train(
    config: TrainingConfig,
    *,
    allow_synthetic: bool = False,
    resume_run: Path | None = None,
    max_steps: int | None = None,
) -> dict[str, Any]:
    """Run one mode with a stable run id and one unique budget segment."""
    import torch

    if config.mode == "full" and allow_synthetic:
        raise TrainingError("full mode never accepts synthetic source data")
    audit = verify_dataset_manifest(
        config, allow_synthetic=allow_synthetic, require_full=config.mode == "full"
    )
    sample_filter = _sample_filter(config, audit)
    checkpoint: dict[str, Any] | None = None
    if resume_run is not None:
        run_root = resume_run.resolve()
        if not run_root.is_dir():
            raise TrainingError(f"resume run directory is missing: {run_root}")
        run_id = run_root.name
        latest = run_root / "latest.pt"
        if not latest.is_file():
            raise TrainingError(f"resume run has no latest.pt: {run_root}")
        checkpoint = load_checkpoint(latest)
        validate_checkpoint_binding(checkpoint, config)
        if checkpoint.get("config_sha256") != config.sha256:
            raise TrainingError("resume checkpoint was written with a different configuration")
        if checkpoint.get("mode") != config.mode:
            raise TrainingError("resume checkpoint mode differs")
    else:
        run_id = _new_run_id(config)
        run_root = config.checkpoint.root / run_id
        _prepare_dir(run_root)
    segment_id = _new_segment_id(run_id)
    instance_id = segment_id
    logger, _ = _make_logger(config, run_id, segment_id)
    metrics = MetricsWriter(run_root)
    budget = BudgetLedger(config.budget.ledger, config.budget.lock, config.budget.limit_seconds)
    ticket: BudgetTicket | None = None
    started = time.monotonic()
    final_reason = "completed"
    interrupted: TrainingInterrupted | None = None
    original_error: BaseException | None = None
    save_records: list[dict[str, Any]] = []
    errors: list[str] = []
    policy = optimizer = scheduler = pre = post = None
    step = 0
    best_val: float | None = None
    train_dataset = None
    validation_dataset = None
    loaded_backbone = 0
    stop_signal: int | None = None
    steps_total = (
        config.mode_spec.steps if max_steps is None else min(config.mode_spec.steps, max_steps)
    )

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal stop_signal
        if stop_signal is None:
            stop_signal = signum

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        mode = config.mode_spec
        ticket = budget.begin(
            run_id=run_id,
            segment_id=segment_id,
            mode=config.mode,
            reserved_seconds=mode.reserve_seconds,
            config_sha256=config.sha256,
            source=str(config.source),
            instance_id=instance_id,
        )
        _emit(
            logger,
            "training.start",
            mode=config.mode,
            seed=config.seed,
            device=config.device,
            steps=steps_total,
            budget_run_id=run_id,
            budget_segment_id=segment_id,
            budget_instance_id=instance_id,
            budget_reserved_s=mode.reserve_seconds,
            budget_used_before_s=ticket.used_before,
            resumed=checkpoint is not None,
        )
        torch.manual_seed(config.seed)
        np.random.seed(config.seed)
        random.seed(config.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(config.seed)
        if config.deterministic:
            torch.use_deterministic_algorithms(True, warn_only=True)

        train_episodes = mode.episodes
        if train_episodes is not None:
            if max(train_episodes) >= len(audit.train_episodes):
                raise TrainingError("mode episode list is outside the training split")
            train_episodes = tuple(train_episodes)
        validation_episodes = tuple(range(len(audit.validation_episodes)))
        train_dataset = _dataset(config, audit.train_root, train_episodes, training=True)
        if sample_filter is not None:
            if len(train_dataset) != sample_filter.source_frames:
                raise TrainingError("filtered training dataset size differs from the sample filter")
            if max(sample_filter.eligible_indices) >= len(train_dataset):
                raise TrainingError("sample filter contains an out-of-range anchor")
        if mode.validation:
            validation_dataset = _dataset(
                config, audit.validation_root, validation_episodes, training=False
            )
        policy, act_config, _, _, loaded_backbone = _build_policy(config)
        pre, post = _build_processors(config, train_dataset, act_config)
        optimizer = torch.optim.AdamW(
            policy.parameters(), lr=config.lr, weight_decay=config.weight_decay
        )
        scheduler = torch.optim.lr_scheduler.ConstantLR(
            optimizer, factor=1.0, total_iters=max(1, steps_total)
        )
        if checkpoint is not None:
            policy.load_state_dict(checkpoint["model"], strict=True)
            optimizer.load_state_dict(checkpoint["optimizer"])
            if checkpoint.get("scheduler") is not None:
                scheduler.load_state_dict(checkpoint["scheduler"])
            rng = checkpoint.get("rng") or {}
            if "python" in rng:
                random.setstate(rng["python"])
            if "numpy" in rng:
                np.random.set_state(rng["numpy"])
            if "torch" in rng:
                torch.set_rng_state(rng["torch"])
            if torch.cuda.is_available() and "cuda" in rng:
                torch.cuda.set_rng_state_all(rng["cuda"])
            step = int(checkpoint.get("step", 0))
            best_val = checkpoint.get("best_val_l1")
        _emit(
            logger,
            "training.model_ready",
            batch_size=config.batch_size,
            loaded_backbone_tensors=loaded_backbone,
            train_frames=len(train_dataset),
            train_source_frames=len(train_dataset),
            train_eligible_frames=(
                len(train_dataset) if sample_filter is None else len(sample_filter.eligible_indices)
            ),
            train_excluded_frames=(0 if sample_filter is None else sample_filter.excluded_frames),
            sample_filter_digest=("" if sample_filter is None else sample_filter.digest),
            sample_filter_sha256=("" if sample_filter is None else sample_filter.file_sha256),
            validation_frames=0 if validation_dataset is None else len(validation_dataset),
            train_episodes=len(audit.train_episodes)
            if train_episodes is None
            else len(train_episodes),
            validation_episodes=0 if validation_dataset is None else len(audit.validation_episodes),
            parameters=sum(parameter.numel() for parameter in policy.parameters()),
            resumed=checkpoint is not None,
        )
        loader = _loader(
            train_dataset,
            config,
            training=True,
            indices=None if sample_filter is None else sample_filter.eligible_indices,
        )
        iterator: Iterator[Any] = iter(loader)
        policy.train()

        while step < steps_total:
            if stop_signal is not None:
                raise TrainingInterrupted("sigterm" if stop_signal == signal.SIGTERM else "sigint")
            ticket.check(config.budget.stop_margin_seconds)
            try:
                samples = next(iterator)
            except StopIteration:
                iterator = iter(loader)
                samples = next(iterator)
            step_started = time.monotonic()
            processed = _collate(
                [_process_sample(pre, sample, _sample_keys(config)) for sample in samples]
            )
            optimizer.zero_grad(set_to_none=True)
            loss, loss_metrics = policy.forward(processed)
            if not torch.isfinite(loss):
                raise TrainingError(f"non-finite training loss at step {step}")
            loss.backward()
            grad_norm = torch.nn.utils.clip_grad_norm_(policy.parameters(), config.grad_clip_norm)
            if not torch.isfinite(grad_norm):
                raise TrainingError(f"non-finite gradient norm at step {step}")
            optimizer.step()
            scheduler.step()
            step += 1
            step_seconds = time.monotonic() - step_started
            if stop_signal is not None:
                raise TrainingInterrupted("sigterm" if stop_signal == signal.SIGTERM else "sigint")
            if step % config.log_every_steps == 0 or step == 1:
                vram_alloc = vram_reserved = 0.0
                if torch.cuda.is_available():
                    vram_alloc = torch.cuda.max_memory_allocated() / (1024 * 1024)
                    vram_reserved = torch.cuda.max_memory_reserved() / (1024 * 1024)
                row = {
                    "step": step,
                    "phase": "train",
                    "unit": "step",
                    "loss": float(loss.detach().cpu()),
                    "l1_loss": float(loss_metrics.get("l1_loss", 0.0)),
                    "kl_loss": float(loss_metrics.get("kld_loss", 0.0)),
                    "lr": float(optimizer.param_groups[0]["lr"]),
                    "grad_norm": float(grad_norm.detach().cpu()),
                    "step_seconds": step_seconds,
                    "elapsed_seconds": time.monotonic() - started,
                    "vram_allocated_mb": vram_alloc,
                    "vram_reserved_mb": vram_reserved,
                    "budget_remaining_s": ticket.remaining_seconds,
                }
                metrics.write(row)
                _emit(logger, "training.step", **row)
            if (
                step % config.save_every_steps == 0 or step == steps_total
            ) and config.checkpoint.keep_latest:
                record = save_checkpoint(
                    run_root / "latest.pt",
                    config=config,
                    policy=policy,
                    optimizer=optimizer,
                    scheduler=scheduler,
                    step=step,
                    best_val=best_val,
                    reason="periodic" if step % config.save_every_steps == 0 else "final",
                    run_id=run_id,
                    segment_id=segment_id,
                    instance_id=instance_id,
                )
                save_records.append(record)
                _emit(logger, "training.checkpoint", path=record["path"], step=step)
            if validation_dataset is not None and (
                step % config.val_every_steps == 0 or step == steps_total
            ):
                validation = _validation(
                    policy, pre, validation_dataset, config, max_batches=config.val_batches
                )
                validation.update({"step": step, "phase": "validation", "unit": "step"})
                metrics.write(validation)
                _emit(logger, "training.validation", **validation)
                if best_val is None or validation["val_l1"] < best_val:
                    best_val = validation["val_l1"]
                    if config.checkpoint.keep_best:
                        record = save_checkpoint(
                            run_root / "best.pt",
                            config=config,
                            policy=policy,
                            optimizer=optimizer,
                            scheduler=scheduler,
                            step=step,
                            best_val=best_val,
                            reason="best_validation",
                            run_id=run_id,
                            segment_id=segment_id,
                            instance_id=instance_id,
                        )
                        save_records.append(record)
            if ticket.remaining_seconds <= config.budget.stop_margin_seconds:
                final_reason = "budget_near_exhausted"
                break
        if stop_signal is not None and final_reason == "completed":
            raise TrainingInterrupted("sigterm" if stop_signal == signal.SIGTERM else "sigint")
    except TrainingInterrupted as error:
        final_reason = f"{error.reason}_save_requested"
        interrupted = error
    except BudgetExceeded as error:
        final_reason = "budget_exhausted"
        if ticket is None:
            original_error = error
    except BaseException as error:
        final_reason = "failed"
        original_error = error
    finally:
        if policy is not None and optimizer is not None and scheduler is not None:
            if config.checkpoint.keep_latest:
                try:
                    record = save_checkpoint(
                        run_root / "latest.pt",
                        config=config,
                        policy=policy,
                        optimizer=optimizer,
                        scheduler=scheduler,
                        step=step,
                        best_val=best_val,
                        reason=final_reason,
                        run_id=run_id,
                        segment_id=segment_id,
                        instance_id=instance_id,
                    )
                    save_records.append(record)
                except BaseException as error:
                    errors.append(f"latest checkpoint save failed: {type(error).__name__}: {error}")
            if config.checkpoint.keep_final:
                try:
                    record = save_checkpoint(
                        run_root / "final.pt",
                        config=config,
                        policy=policy,
                        optimizer=optimizer,
                        scheduler=scheduler,
                        step=step,
                        best_val=best_val,
                        reason=final_reason,
                        run_id=run_id,
                        segment_id=segment_id,
                        instance_id=instance_id,
                    )
                    save_records.append(record)
                except BaseException as error:
                    errors.append(f"final checkpoint save failed: {type(error).__name__}: {error}")
        event_reason = "failed" if errors else final_reason
        if original_error is not None:
            try:
                logger.emit(
                    "training.exception",
                    level="ERROR",
                    exception=original_error,
                    fields={"reason": final_reason, "step": step},
                )
            except BaseException as error:
                errors.append(f"exception event failed: {type(error).__name__}: {error}")
        else:
            try:
                _emit(
                    logger,
                    "training.end",
                    reason=event_reason,
                    step=step,
                    segment_id=segment_id,
                    instance_id=instance_id,
                )
            except BaseException as error:
                errors.append(f"training.end event failed: {type(error).__name__}: {error}")
        try:
            logger.close()
        except BaseException as error:
            errors.append(f"logger close failed: {type(error).__name__}: {error}")
        reason_for_report = "failed" if errors else final_reason
        peak_alloc = peak_reserved = 0.0
        if torch.cuda.is_available():
            peak_alloc = torch.cuda.max_memory_allocated() / (1024 * 1024)
            peak_reserved = torch.cuda.max_memory_reserved() / (1024 * 1024)
        report = _report(
            config=config,
            audit=audit,
            run_id=run_id,
            segment_id=segment_id,
            instance_id=instance_id,
            reason=reason_for_report,
            step=step,
            steps_total=steps_total,
            device=config.device,
            policy=policy,
            loaded_backbone=loaded_backbone,
            started=started,
            best_val=best_val,
            save_records=save_records,
            errors=errors,
            peak_alloc=peak_alloc,
            peak_reserved=peak_reserved,
            sample_filter=sample_filter,
        )
        try:
            payload = json.dumps(report, indent=2, allow_nan=False, default=str) + "\n"
            (run_root / f"training_report-{segment_id}.json").write_text(payload, encoding="utf-8")
            (run_root / "training_report.json").write_text(payload, encoding="utf-8")
        except BaseException as error:
            errors.append(f"training report write failed: {type(error).__name__}: {error}")
        if ticket is not None:
            try:
                ticket.finish(reason_for_report, reason=final_reason)
            except BaseException as error:
                errors.append(f"budget finish failed: {type(error).__name__}: {error}")
                report["reason"] = "failed"
                report["errors"] = list(errors)
                try:
                    payload = json.dumps(report, indent=2, allow_nan=False, default=str) + "\n"
                    (run_root / f"training_report-{segment_id}.json").write_text(payload)
                    (run_root / "training_report.json").write_text(payload)
                except OSError as report_error:
                    errors.append(f"failure report update failed: {report_error}")
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    if errors:
        message = "; ".join(errors)
        if original_error is not None:
            original_error.add_note(message)
            raise original_error
        raise TrainingError(message)
    if original_error is not None:
        raise original_error
    if interrupted is not None:
        raise interrupted
    return report


__all__ = [
    "DatasetAudit",
    "TrainingError",
    "build_act_config",
    "masked_l1",
    "train",
    "verify_dataset_manifest",
]
