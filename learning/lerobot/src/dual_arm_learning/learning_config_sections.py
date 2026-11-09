"""Pydantic sections and document shaping for learning configuration."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeFloat,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    SafeIdStr,
    Sha256HexStr,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

PINNED_REVISION = "7e241bd630a3719a56157a497ce5d08f244784f1"
PINNED_ARCHIVE = "710397ad753090743c1aadf04b2bd40584fbca8448b1409131138172913c8a0c"


def episodes_before(value: object) -> object:
    if value == "all":
        return None
    return tuple(value) if isinstance(value, list) else value


def unknown(block: object, allowed: set[str], prefix: str) -> None:
    if isinstance(block, dict):
        for key in block:
            if key not in allowed:
                raise contract_error(f"{prefix}.{key}", "unknown key")


def absolute(value: object, root: Path) -> Path:
    candidate = Path(str(value))
    return candidate if candidate.is_absolute() else (root / candidate).resolve()


Episodes = Annotated[tuple[Annotated[int, Field(strict=True, ge=0, le=9999)], ...] | None,
                     BeforeValidator(episodes_before)]
Names3 = Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                   Field(min_length=3, max_length=3)]
VectorN = Annotated[tuple[NonNegativeInt, ...], BeforeValidator(as_tuple),
                    Field(min_length=5, max_length=5)]


class RGBEncoderSpec(StrictMappingModel):
    vcodec: Literal["h264"] = "h264"
    g: Annotated[int, Field(strict=True, ge=2, le=2)] = 2
    crf: Annotated[int, Field(strict=True, ge=18, le=18)] = 18
    preset: Literal["veryfast"] = "veryfast"


class ModelSpec(StrictMappingModel):
    vision_backbone: Literal["resnet18"] = "resnet18"
    dim_model: PositiveInt = 256
    n_heads: PositiveInt = 8
    dim_feedforward: PositiveInt = 1024
    n_encoder_layers: PositiveInt = 4
    n_decoder_layers: PositiveInt = 1
    use_vae: bool = True
    latent_dim: PositiveInt = 4
    n_vae_encoder_layers: PositiveInt = 4
    dropout: Annotated[NonNegativeFloat, Field(le=0.9)] = 0.1
    kl_weight: NonNegativeFloat = 10.0
    pretrained_backbone_weights: str | None = None
    weights_path: Path = Field(validation_alias="weights_path")
    weights_sha256: Sha256HexStr = Field(validation_alias="weights_sha256")

    @model_validator(mode="before")
    @classmethod
    def _weights(cls, data: object) -> object:
        if not isinstance(data, dict):
            return data
        data = dict(data)
        weights = data.pop("weights", None)
        if isinstance(weights, dict):
            unknown(weights, {"path", "sha256"}, "model.weights")
            data["weights_path"] = weights.get("path")
            data["weights_sha256"] = weights.get("sha256")
        return data

    @model_validator(mode="after")
    def _local_weights(self) -> ModelSpec:
        if self.pretrained_backbone_weights is not None:
            raise contract_error("pretrained_backbone_weights", "must be null")
        return self


class ModeSpec(StrictMappingModel):
    steps: Annotated[PositiveInt, Field(le=1_000_000)]
    split: Literal["train", "validation"]
    episodes: Episodes
    validation: bool
    reserve_seconds: Annotated[PositiveFloat, Field(ge=1.0)]

    @model_validator(mode="after")
    def _episodes_unique(self) -> ModeSpec:
        if self.episodes is not None and len(set(self.episodes)) != len(self.episodes):
            raise contract_error("episodes", "must be unique")
        return self


class BudgetSpec(StrictMappingModel):
    limit_seconds: Annotated[PositiveFloat, Field(ge=1.0)]
    ledger: Path
    lock: Path
    stop_margin_seconds: NonNegativeFloat
    min_free_bytes: NonNegativeInt


class CheckpointSpec(StrictMappingModel):
    root: Path
    keep_latest: bool
    keep_best: bool
    keep_final: bool
    min_free_bytes: NonNegativeInt
    save_before_exit: bool


class LoggingSpec(StrictMappingModel):
    root: Path
    component: SafeIdStr
    queue_capacity: Annotated[PositiveInt, Field(le=65536)]
    max_event_bytes: Annotated[PositiveInt, Field(ge=1024, le=2**24)]
    max_file_bytes: Annotated[PositiveInt, Field(ge=1024, le=2**30)]
    max_total_bytes: Annotated[PositiveInt, Field(ge=1024, le=2**34)]
    close_timeout_s: Annotated[PositiveFloat, Field(ge=0.1)]
    flush_interval_s: Annotated[PositiveFloat, Field(ge=0.01)]
    console: bool


def dataset_before(data: object) -> object:
    if not isinstance(data, dict):
        return data
    data = dict(data)
    root = data.get("project_root", Path("."))
    for name in ("profile", "select", "output_root", "cache_root"):
        if name in data and not isinstance(data[name], Path):
            data[name] = root / Path(data[name])
    unknown(data.get("expected"), {"formal_episodes", "training_episodes",
                                   "validation_episodes", "validation_positions"}, "expected")
    lerobot = data.get("lerobot")
    unknown(lerobot, {"source_revision", "source_archive_sha256", "rgb_encoder",
                      "video_backend", "video_files_size_mb", "data_files_size_mb",
                      "encoder_threads", "image_writer_threads"}, "lerobot")
    if isinstance(lerobot, dict):
        unknown(lerobot.get("rgb_encoder"), {"vcodec", "g", "crf", "preset"},
                "lerobot.rgb_encoder")
    return data


def dataset_after(config) -> None:
    if config.camera_names != ("global", "left_wrist", "right_wrist"):
        raise contract_error("camera_names", "must be global/left_wrist/right_wrist")
    if config.validation_positions != (9, 19, 29, 39, 49):
        raise contract_error("validation_positions",
                             "must be the 10th/20th/30th/40th/50th")
    if config.lerobot_source_revision != PINNED_REVISION:
        raise contract_error("lerobot.source_revision", "must be the pinned revision")
    if config.lerobot_source_archive_sha256 != PINNED_ARCHIVE:
        raise contract_error("lerobot.source_archive_sha256", "must match the frozen export")
    if (config.attempt_export_glob.count("*") > 1
            or not config.attempt_export_glob.endswith("export")):
        raise contract_error("attempt_export_glob",
                             "must be a bounded glob ending in export")


def training_before(data: object) -> object:
    if not isinstance(data, dict):
        return data
    data = dict(data)
    root = data.get("project_root", Path("."))
    block = data.get("data")
    if isinstance(block, dict):
        block = dict(block)
        for key in ("config", "root"):
            block[key] = absolute(block[key], root)
        if block.get("sample_filter") is not None:
            block["sample_filter"] = absolute(block["sample_filter"], root)
        unknown(block, {"config", "root", "manifest", "train_dir", "validation_dir",
                        "verify_hashes", "sample_filter"}, "data")
        data["data"] = block
    unknown(data.get("training"), {"batch_size", "num_workers", "lr", "weight_decay",
                                   "grad_clip_norm", "log_every_steps", "val_every_steps",
                                   "save_every_steps", "val_batches", "dtype", "amp",
                                   "deterministic"}, "training")
    model = data.get("model")
    if isinstance(model, dict):
        model = dict(model)
        weights = model.get("weights")
        if isinstance(weights, dict):
            model["weights"] = {**weights, "path": absolute(weights["path"], root)}
        data["model"] = model
    data["cache_root"] = absolute(
        data.get("cache_root", ".build/cache/huggingface-learning"), root)
    for section, keys in (("budget", ("ledger", "lock")),
                          ("checkpoint", ("root",)), ("logging", ("root",))):
        block = data.get(section)
        if isinstance(block, dict):
            block = dict(block)
            for key in keys:
                block[key] = absolute(block[key], root)
            data[section] = block
    return data


def training_after(config) -> None:
    if set(config.modes) != {"smoke", "overfit", "full"}:
        raise contract_error("modes", "must define smoke, overfit and full")
    if config.mode not in config.modes:
        raise contract_error("mode", "must be one of the configured modes")
    full = config.modes["full"]
    if full.episodes is not None:
        raise contract_error("modes.full.episodes", "must use all 45 training episodes")
    if not full.validation or full.split != "train":
        raise contract_error("modes.full",
                             "must validate on the independent 5-episode split")
    if config.budget.limit_seconds > 14400.0:
        raise contract_error("budget.limit_seconds",
                             "task training budget is capped at 14400 s")


def feature_spec(config) -> dict[str, dict[str, object]]:
    features: dict[str, dict[str, object]] = {
        "observation.state": {"dtype": "float32", "shape": (config.state_dim,)},
        "observation.velocity": {"dtype": "float32", "shape": (config.state_dim,)},
        "action": {"dtype": "float32", "shape": (config.state_dim,)},
        "raw_action_chunk": {"dtype": "float32",
                             "shape": (config.chunk_size, config.state_dim)},
        "observation.sim_time_ns": {"dtype": "int64", "shape": (1,)},
        "action.sim_time_ns": {"dtype": "int64", "shape": (1,)},
        "source.episode_index": {"dtype": "int64", "shape": (1,)},
        "source.seed": {"dtype": "int64", "shape": (1,)},
    }
    for camera in config.camera_names:
        features[f"observation.images.{camera}"] = {
            "dtype": "video", "shape": (config.camera_height, config.camera_width, 3)}
    return features
