"""Strict, ROS-free YAML configuration for conversion and ACT training."""

from __future__ import annotations

import copy
import hashlib
import json
import re
from pathlib import Path
from typing import Annotated, Literal

from pydantic import AliasPath, BeforeValidator, Field, model_validator
from robot_contracts.overrides import apply_overrides
from robot_contracts.project_paths import project_path
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeFloat,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    SafeIdStr,
    StrictMappingModel,
    as_tuple,
    contract_error,
    validate_config_model,
)

from .config_yaml import ConfigError, load_yaml

__all__ = [
    "CheckpointSpec",
    "ConfigError",
    "DatasetConfig",
    "LoggingSpec",
    "ModeSpec",
    "ModelSpec",
    "RGBEncoderSpec",
    "TrainingConfig",
    "load_dataset_config",
    "load_training_config",
    "parse_dataset_config",
    "parse_training_config",
]

PNG_SHA = re.compile(r"^[0-9a-f]{64}$")
OptionalSha = Annotated[str, Field(strict=True, pattern=r"^(?:[0-9a-f]{64})?$")]
PINNED_REVISION = "7e241bd630a3719a56157a497ce5d08f244784f1"


def episodes_before(value: object) -> object:
    if value == "all":
        return None
    return tuple(value) if isinstance(value, list) else value


def unknown(block: object, allowed: set[str], prefix: str) -> None:
    if isinstance(block, dict):
        for key in block:
            if key not in allowed:
                raise contract_error(f"{prefix}.{key}", "unknown key")


Episodes = Annotated[
    tuple[Annotated[int, Field(strict=True, ge=0, le=9999)], ...] | None,
    BeforeValidator(episodes_before),
]
Names3 = Annotated[
    tuple[NonEmptyStr, ...], BeforeValidator(as_tuple), Field(min_length=3, max_length=3)
]
VectorN = Annotated[
    tuple[NonNegativeInt, ...], BeforeValidator(as_tuple), Field(min_length=1, max_length=50)
]
FORMAL_SPLIT = (50, 45, 5)
FORMAL_VALIDATION_POSITIONS = (9, 19, 29, 39, 49)


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
    weights_sha256: OptionalSha = Field(default="", validation_alias="weights_sha256")

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
            data["weights_sha256"] = weights.get("sha256", "")
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

    @model_validator(mode="after")
    def _episodes_unique(self) -> ModeSpec:
        if self.episodes is not None and len(set(self.episodes)) != len(self.episodes):
            raise contract_error("episodes", "must be unique")
        return self


class CheckpointSpec(StrictMappingModel):
    root: Path


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


class DatasetConfig(StrictMappingModel):
    """Conversion contract and feature source for one learning data root."""

    project_root: Path = Path(".")
    schema_version: Annotated[int, Field(strict=True, ge=1, le=1)] = 1
    fps: Annotated[int, Field(strict=True, ge=20, le=20)]
    chunk_size: Annotated[int, Field(strict=True, ge=60, le=60)]
    action_lead_ns: Annotated[int, Field(strict=True, ge=50_000_000, le=50_000_000)]
    state_dim: Annotated[int, Field(strict=True, ge=16, le=16)]
    camera_names: Names3
    camera_width: Annotated[int, Field(strict=True, ge=320, le=320)]
    camera_height: Annotated[int, Field(strict=True, ge=240, le=240)]
    task_text: NonEmptyStr
    profile_path: Path = Field(validation_alias="profile")
    select_path: Path = Field(validation_alias="select")
    output_root: Path
    cache_root: Path
    attempt_export_glob: NonEmptyStr
    verify_png_pixels: bool
    require_quality: bool
    diagnostic: bool = False
    expected_formal_episodes: Annotated[int, Field(strict=True, ge=1, le=9999)] = Field(
        validation_alias=AliasPath("expected", "formal_episodes")
    )
    expected_training_episodes: Annotated[int, Field(strict=True, ge=1, le=9999)] = Field(
        validation_alias=AliasPath("expected", "training_episodes")
    )
    expected_validation_episodes: Annotated[int, Field(strict=True, ge=1, le=9999)] = Field(
        validation_alias=AliasPath("expected", "validation_episodes")
    )
    validation_positions: VectorN = Field(
        validation_alias=AliasPath("expected", "validation_positions")
    )
    lerobot_source_revision: NonEmptyStr = Field(
        validation_alias=AliasPath("lerobot", "source_revision")
    )
    lerobot_source_archive_sha256: OptionalSha = Field(
        default="", validation_alias=AliasPath("lerobot", "source_archive_sha256")
    )
    video_backend: Literal["pyav", "torchcodec"] = Field(
        default="pyav", validation_alias=AliasPath("lerobot", "video_backend")
    )
    video_files_size_mb: Annotated[int, Field(strict=True, ge=16, le=16384)] = Field(
        validation_alias=AliasPath("lerobot", "video_files_size_mb")
    )
    data_files_size_mb: Annotated[int, Field(strict=True, ge=16, le=16384)] = Field(
        validation_alias=AliasPath("lerobot", "data_files_size_mb")
    )
    encoder_threads: Annotated[int, Field(strict=True, ge=0, le=64)] = Field(
        validation_alias=AliasPath("lerobot", "encoder_threads")
    )
    image_writer_threads: Annotated[int, Field(strict=True, ge=0, le=64)] = Field(
        validation_alias=AliasPath("lerobot", "image_writer_threads")
    )
    rgb_vcodec: Literal["h264"] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "vcodec")
    )
    rgb_g: Annotated[int, Field(strict=True, ge=2, le=2)] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "g")
    )
    rgb_crf: Annotated[int, Field(strict=True, ge=18, le=18)] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "crf")
    )
    rgb_preset: Literal["veryfast"] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "preset")
    )

    @model_validator(mode="before")
    @classmethod
    def _paths(cls, data: object) -> object:
        return dataset_before(data)

    @model_validator(mode="after")
    def _contract(self) -> DatasetConfig:
        dataset_after(self)
        return self

    rgb_encoder = property(
        lambda self: RGBEncoderSpec(
            vcodec=self.rgb_vcodec, g=self.rgb_g, crf=self.rgb_crf, preset=self.rgb_preset
        )
    )
    profile_sha256 = property(
        lambda self: hashlib.sha256(self.profile_path.read_bytes()).hexdigest()
    )

    def feature_spec(self) -> dict[str, dict[str, object]]:
        return feature_spec(self)


class TrainingConfig(StrictMappingModel):
    project_root: Path
    source: Path
    schema_version: Annotated[int, Field(strict=True, ge=1, le=1)] = 1
    mode: NonEmptyStr
    seed: Annotated[NonNegativeInt, Field(le=2**31 - 1)]
    device: NonEmptyStr
    cache_root: Path
    dataset_config_path: Path = Field(validation_alias=AliasPath("data", "config"))
    data_root: Path = Field(validation_alias=AliasPath("data", "root"))
    manifest_name: NonEmptyStr = Field(validation_alias=AliasPath("data", "manifest"))
    train_dir: NonEmptyStr = Field(validation_alias=AliasPath("data", "train_dir"))
    validation_dir: NonEmptyStr = Field(validation_alias=AliasPath("data", "validation_dir"))
    verify_hashes: bool = Field(validation_alias=AliasPath("data", "verify_hashes"))
    sample_filter: Path | None = Field(
        default=None, validation_alias=AliasPath("data", "sample_filter")
    )
    batch_size: Annotated[PositiveInt, Field(le=256)] = Field(
        validation_alias=AliasPath("training", "batch_size")
    )
    num_workers: Annotated[NonNegativeInt, Field(le=32)] = Field(
        default=4, validation_alias=AliasPath("training", "num_workers")
    )
    pin_memory: bool = Field(default=True, validation_alias=AliasPath("training", "pin_memory"))
    prefetch_factor: Annotated[PositiveInt, Field(le=32)] = Field(
        default=2, validation_alias=AliasPath("training", "prefetch_factor")
    )
    use_amp: bool = Field(default=True, validation_alias=AliasPath("training", "use_amp"))
    amp_dtype: Literal["bfloat16", "float16"] = Field(
        default="bfloat16", validation_alias=AliasPath("training", "amp_dtype")
    )
    lr: Annotated[PositiveFloat, Field(ge=1e-12)] = Field(
        validation_alias=AliasPath("training", "lr")
    )
    lr_backbone: Annotated[PositiveFloat, Field(ge=1e-12)] | None = Field(
        default=None, validation_alias=AliasPath("training", "lr_backbone")
    )
    weight_decay: NonNegativeFloat = Field(validation_alias=AliasPath("training", "weight_decay"))
    grad_clip_norm: NonNegativeFloat = Field(
        validation_alias=AliasPath("training", "grad_clip_norm")
    )
    log_every_steps: Annotated[PositiveInt, Field(le=1_000_000)] = Field(
        validation_alias=AliasPath("training", "log_every_steps")
    )
    val_every_steps: Annotated[PositiveInt, Field(le=1_000_000)] = Field(
        validation_alias=AliasPath("training", "val_every_steps")
    )
    val_batches: Annotated[PositiveInt, Field(le=10000)] = Field(
        validation_alias=AliasPath("training", "val_batches")
    )
    image_shift_px: Annotated[NonNegativeInt, Field(le=32)] = Field(
        default=0, validation_alias=AliasPath("training", "image_shift_px")
    )
    deterministic: bool = Field(validation_alias=AliasPath("training", "deterministic"))
    modes: dict[str, ModeSpec]
    model: ModelSpec
    checkpoint: CheckpointSpec
    logging: LoggingSpec
    dataset: DatasetConfig

    @model_validator(mode="before")
    @classmethod
    def _paths(cls, data: object) -> object:
        return training_before(data)

    @model_validator(mode="after")
    def _training(self) -> TrainingConfig:
        training_after(self)
        return self

    mode_spec = property(lambda self: self.modes[self.mode])

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.source.read_bytes()).hexdigest()

    def to_document(self) -> dict[str, object]:
        return self.model_dump(mode="json")


def dataset_before(data: object) -> object:
    if not isinstance(data, dict):
        return data
    data = dict(data)
    root = data.get("project_root", Path("."))
    for name in ("profile", "select", "output_root", "cache_root"):
        if name in data and not isinstance(data[name], Path):
            data[name] = project_path(data[name], root)
    unknown(
        data.get("expected"),
        {"formal_episodes", "training_episodes", "validation_episodes", "validation_positions"},
        "expected",
    )
    lerobot = data.get("lerobot")
    unknown(
        lerobot,
        {
            "source_revision",
            "source_archive_sha256",
            "rgb_encoder",
            "video_backend",
            "video_files_size_mb",
            "data_files_size_mb",
            "encoder_threads",
            "image_writer_threads",
        },
        "lerobot",
    )
    if isinstance(lerobot, dict):
        unknown(lerobot.get("rgb_encoder"), {"vcodec", "g", "crf", "preset"}, "lerobot.rgb_encoder")
    return data


def dataset_after(config) -> None:
    if config.camera_names != ("global", "left_wrist", "right_wrist"):
        raise contract_error("camera_names", "must be global/left_wrist/right_wrist")
    if len(set(config.validation_positions)) != len(config.validation_positions):
        raise contract_error("validation_positions", "must be unique")
    if config.diagnostic:
        if config.expected_formal_episodes != (
            config.expected_training_episodes + config.expected_validation_episodes
        ):
            raise contract_error(
                "expected.formal_episodes", "must equal the training and validation episode counts"
            )
        if len(config.validation_positions) != config.expected_validation_episodes:
            raise contract_error("validation_positions", "must name every validation episode")
        if any(
            position >= config.expected_formal_episodes for position in config.validation_positions
        ):
            raise contract_error(
                "validation_positions", "must stay inside the configured episode range"
            )
    else:
        if (
            config.expected_formal_episodes,
            config.expected_training_episodes,
            config.expected_validation_episodes,
        ) != FORMAL_SPLIT:
            raise contract_error("expected", "formal data must be exactly 50/45/5")
        if config.validation_positions != FORMAL_VALIDATION_POSITIONS:
            raise contract_error("validation_positions", "must be the 10th/20th/30th/40th/50th")
    if config.lerobot_source_revision != PINNED_REVISION:
        raise contract_error("lerobot.source_revision", "must be the pinned revision")
    if config.lerobot_source_archive_sha256:
        binding = project_path("learning/lerobot/docker/learning-source.json", config.project_root)
        if binding.is_file():
            declared = json.loads(binding.read_text(encoding="utf-8")).get("archive_sha256")
            if declared and config.lerobot_source_archive_sha256 != declared:
                raise contract_error(
                    "lerobot.source_archive_sha256", "differs from the source binding"
                )
    if config.attempt_export_glob.count("*") > 1 or not config.attempt_export_glob.endswith(
        "export"
    ):
        raise contract_error("attempt_export_glob", "must be a bounded glob ending in export")


def training_before(data: object) -> object:
    if not isinstance(data, dict):
        return data
    data = dict(data)
    root = data.get("project_root", Path("."))
    block = data.get("data")
    if isinstance(block, dict):
        block = dict(block)
        for key in ("config", "root"):
            block[key] = project_path(block[key], root)
        if block.get("sample_filter") is not None:
            block["sample_filter"] = project_path(block["sample_filter"], root)
        unknown(
            block,
            {
                "config",
                "root",
                "manifest",
                "train_dir",
                "validation_dir",
                "verify_hashes",
                "sample_filter",
            },
            "data",
        )
        data["data"] = block
    unknown(
        data.get("training"),
        {
            "batch_size",
            "num_workers",
            "pin_memory",
            "prefetch_factor",
            "use_amp",
            "amp_dtype",
            "lr",
            "lr_backbone",
            "weight_decay",
            "grad_clip_norm",
            "log_every_steps",
            "val_every_steps",
            "val_batches",
            "image_shift_px",
            "deterministic",
        },
        "training",
    )
    model = data.get("model")
    if isinstance(model, dict):
        model = dict(model)
        weights = model.get("weights")
        if isinstance(weights, dict):
            model["weights"] = {**weights, "path": project_path(weights["path"], root)}
        data["model"] = model
    if "cache_root" in data:
        data["cache_root"] = project_path(data["cache_root"], root)
    for section, keys in (
        ("checkpoint", ("root",)),
        ("logging", ("root",)),
    ):
        block = data.get(section)
        if isinstance(block, dict):
            block = dict(block)
            for key in keys:
                block[key] = project_path(block[key], root)
            data[section] = block
    return data


def training_after(config) -> None:
    if set(config.modes) != {"smoke", "overfit", "full"}:
        raise contract_error("modes", "must define smoke, overfit and full")
    if config.mode not in config.modes:
        raise contract_error("mode", "must be one of the configured modes")
    full = config.modes["full"]
    if full.episodes is not None:
        raise contract_error("modes.full.episodes", "must use every configured training episode")
    if not full.validation or full.split != "train":
        raise contract_error("modes.full", "must validate on the independent validation split")
    if config.lr_backbone is not None and config.lr_backbone > config.lr:
        raise contract_error("training.lr_backbone", "must not exceed training.lr")


def feature_spec(config) -> dict[str, dict[str, object]]:
    features: dict[str, dict[str, object]] = {
        "observation.state": {"dtype": "float32", "shape": (config.state_dim,)},
        "observation.velocity": {"dtype": "float32", "shape": (config.state_dim,)},
        "action": {"dtype": "float32", "shape": (config.state_dim,)},
        "raw_action_chunk": {"dtype": "float32", "shape": (config.chunk_size, config.state_dim)},
        "observation.sim_time_ns": {"dtype": "int64", "shape": (1,)},
        "action.sim_time_ns": {"dtype": "int64", "shape": (1,)},
        "source.episode_index": {"dtype": "int64", "shape": (1,)},
        "source.seed": {"dtype": "int64", "shape": (1,)},
    }
    for camera in config.camera_names:
        features[f"observation.images.{camera}"] = {
            "dtype": "video",
            "shape": (config.camera_height, config.camera_width, 3),
        }
    return features


def parse_dataset_config(document: object, project_root: str | Path) -> DatasetConfig:
    root = Path(project_root).resolve()
    if not isinstance(document, dict):
        raise ConfigError("<dataset>", "<root>", "must be a mapping")
    return validate_config_model(
        DatasetConfig, {**document, "project_root": root}, "<dataset>", ConfigError
    )


def load_dataset_config(
    path: str | Path,
    project_root: str | Path,
    *,
    overrides: tuple[str, ...] | list[str] = (),
) -> DatasetConfig:
    document = load_yaml(path)
    if overrides:
        document = apply_overrides(document, list(overrides))
    return parse_dataset_config(document, project_root)


def _dataset_config_path(document: dict) -> object:
    block = document.get("data")
    if isinstance(block, dict) and "config" in block:
        return block["config"]
    return document["dataset_config_path"]


def parse_training_config(
    document: object, *, project_root: str | Path, source: str | Path
) -> TrainingConfig:
    root = Path(project_root).resolve()
    if not isinstance(document, dict):
        raise ConfigError(str(source), "<root>", "must be a mapping")
    dataset = parse_dataset_config(
        load_yaml(project_path(_dataset_config_path(document), root)), root
    )
    value = copy.deepcopy(document)
    value.update({"project_root": root, "source": Path(source), "dataset": dataset})
    return validate_config_model(TrainingConfig, value, source, ConfigError)


def load_training_config(
    path: str | Path,
    project_root: str | Path,
    *,
    overrides: tuple[str, ...] | list[str] = (),
) -> TrainingConfig:
    document = load_yaml(path)
    if overrides:
        document = apply_overrides(document, list(overrides))
    return parse_training_config(document, project_root=project_root, source=path)
