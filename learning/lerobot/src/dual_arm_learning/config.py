"""Strict, ROS-free YAML configuration for conversion and ACT training."""

from __future__ import annotations

import copy
import hashlib
import re
from pathlib import Path
from typing import Annotated, Literal

from pydantic import AliasPath, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeFloat,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    Sha256HexStr,
    StrictMappingModel,
    validate_config_model,
)

from .config_yaml import ConfigError, load_yaml
from .learning_config_sections import (
    BudgetSpec,
    CheckpointSpec,
    LoggingSpec,
    ModelSpec,
    ModeSpec,
    Names3,
    RGBEncoderSpec,
    VectorN,
    absolute,
    dataset_after,
    dataset_before,
    feature_spec,
    training_after,
    training_before,
)

__all__ = ["BudgetSpec", "CheckpointSpec", "ConfigError", "DatasetConfig", "LoggingSpec",
           "ModeSpec", "ModelSpec", "RGBEncoderSpec", "TrainingConfig",
           "load_dataset_config", "load_training_config", "parse_dataset_config",
           "parse_training_config"]

PNG_SHA = re.compile(r"^[0-9a-f]{64}$")
_TASK_TEXT = "Pick up the red source cube and place it on the blue target cube."


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
    task_text: NonEmptyStr = _TASK_TEXT
    profile_path: Path = Field(validation_alias="profile")
    select_path: Path = Field(validation_alias="select")
    output_root: Path
    cache_root: Path
    attempt_export_glob: NonEmptyStr
    verify_png_pixels: bool
    require_quality: bool
    expected_formal_episodes: Annotated[int, Field(strict=True, ge=50, le=50)] = Field(
        validation_alias=AliasPath("expected", "formal_episodes"))
    expected_training_episodes: Annotated[int, Field(strict=True, ge=45, le=45)] = Field(
        validation_alias=AliasPath("expected", "training_episodes"))
    expected_validation_episodes: Annotated[int, Field(strict=True, ge=5, le=5)] = Field(
        validation_alias=AliasPath("expected", "validation_episodes"))
    validation_positions: VectorN = Field(
        validation_alias=AliasPath("expected", "validation_positions"))
    lerobot_source_revision: NonEmptyStr = Field(
        validation_alias=AliasPath("lerobot", "source_revision"))
    lerobot_source_archive_sha256: Sha256HexStr = Field(
        validation_alias=AliasPath("lerobot", "source_archive_sha256"))
    video_backend: Literal["torchcodec"] = Field(
        validation_alias=AliasPath("lerobot", "video_backend"))
    video_files_size_mb: Annotated[int, Field(strict=True, ge=16, le=16384)] = Field(
        validation_alias=AliasPath("lerobot", "video_files_size_mb"))
    data_files_size_mb: Annotated[int, Field(strict=True, ge=16, le=16384)] = Field(
        validation_alias=AliasPath("lerobot", "data_files_size_mb"))
    encoder_threads: Annotated[int, Field(strict=True, ge=0, le=64)] = Field(
        validation_alias=AliasPath("lerobot", "encoder_threads"))
    image_writer_threads: Annotated[int, Field(strict=True, ge=0, le=64)] = Field(
        validation_alias=AliasPath("lerobot", "image_writer_threads"))
    rgb_vcodec: Literal["h264"] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "vcodec"))
    rgb_g: Annotated[int, Field(strict=True, ge=2, le=2)] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "g"))
    rgb_crf: Annotated[int, Field(strict=True, ge=18, le=18)] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "crf"))
    rgb_preset: Literal["veryfast"] = Field(
        validation_alias=AliasPath("lerobot", "rgb_encoder", "preset"))

    @model_validator(mode="before")
    @classmethod
    def _paths(cls, data: object) -> object:
        return dataset_before(data)

    @model_validator(mode="after")
    def _contract(self) -> DatasetConfig:
        dataset_after(self)
        return self

    rgb_encoder = property(lambda self: RGBEncoderSpec(
        vcodec=self.rgb_vcodec, g=self.rgb_g, crf=self.rgb_crf, preset=self.rgb_preset))
    profile_sha256 = property(
        lambda self: hashlib.sha256(self.profile_path.read_bytes()).hexdigest())

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
        default=None, validation_alias=AliasPath("data", "sample_filter"))
    batch_size: Annotated[PositiveInt, Field(le=256)] = Field(
        validation_alias=AliasPath("training", "batch_size"))
    num_workers: Annotated[NonNegativeInt, Field(le=32)] = Field(
        validation_alias=AliasPath("training", "num_workers"))
    lr: Annotated[PositiveFloat, Field(ge=1e-12)] = Field(
        validation_alias=AliasPath("training", "lr"))
    weight_decay: NonNegativeFloat = Field(
        validation_alias=AliasPath("training", "weight_decay"))
    grad_clip_norm: NonNegativeFloat = Field(
        validation_alias=AliasPath("training", "grad_clip_norm"))
    log_every_steps: Annotated[PositiveInt, Field(le=1_000_000)] = Field(
        validation_alias=AliasPath("training", "log_every_steps"))
    val_every_steps: Annotated[PositiveInt, Field(le=1_000_000)] = Field(
        validation_alias=AliasPath("training", "val_every_steps"))
    save_every_steps: Annotated[PositiveInt, Field(le=1_000_000)] = Field(
        validation_alias=AliasPath("training", "save_every_steps"))
    val_batches: Annotated[PositiveInt, Field(le=10000)] = Field(
        validation_alias=AliasPath("training", "val_batches"))
    dtype: Literal["float32"] = Field(validation_alias=AliasPath("training", "dtype"))
    amp: Literal[False] = Field(validation_alias=AliasPath("training", "amp"))
    deterministic: bool = Field(validation_alias=AliasPath("training", "deterministic"))
    modes: dict[str, ModeSpec]
    model: ModelSpec
    budget: BudgetSpec
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


def parse_dataset_config(document: object, project_root: str | Path) -> DatasetConfig:
    root = Path(project_root).resolve()
    if not isinstance(document, dict):
        raise ConfigError("<dataset>", "<root>", "must be a mapping")
    return validate_config_model(DatasetConfig, {**document, "project_root": root},
                                 "<dataset>", ConfigError)


def load_dataset_config(path: str | Path, project_root: str | Path) -> DatasetConfig:
    return parse_dataset_config(load_yaml(path), project_root)


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
        load_yaml(absolute(_dataset_config_path(document), root)), root)
    value = copy.deepcopy(document)
    value.update({"project_root": root, "source": Path(source), "dataset": dataset})
    return validate_config_model(TrainingConfig, value, source, ConfigError)


def load_training_config(path: str | Path, project_root: str | Path) -> TrainingConfig:
    return parse_training_config(load_yaml(path), project_root=project_root, source=path)
