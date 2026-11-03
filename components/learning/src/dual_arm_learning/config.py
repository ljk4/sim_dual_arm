"""Strict, ROS-free YAML configuration for conversion and ACT training.

The learning process must never import ``rclpy``.  This module deliberately
uses only the standard library and PyYAML so configuration rejection is
testable in the small NumPy-only environment.
"""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
PNG_SHA = re.compile(r"^[0-9a-f]{64}$")


class ConfigError(ValueError):
    """One actionable configuration rejection with its dotted field."""

    def __init__(self, field_name: str, message: str) -> None:
        self.field = field_name
        super().__init__(f"{field_name}: {message}")


def load_yaml(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
    except OSError as error:
        raise ConfigError("<file>", f"cannot read {source}: {error}") from error
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ConfigError("<yaml>", str(error)) from error
    return mapping(document, "<root>")


def mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(location, "must be an object")
    result: dict[str, Any] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise ConfigError(location, "keys must be strings")
        result[key] = item
    return result


def reject_unknown(value: dict[str, Any], known: set[str], location: str) -> None:
    unknown = sorted(set(value) - known)
    if unknown:
        raise ConfigError(location, f"unknown field(s): {', '.join(unknown)}")


def require(value: dict[str, Any], name: str, location: str) -> Any:
    if name not in value:
        raise ConfigError(f"{location}.{name}", "field is required")
    return value[name]


def text(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise ConfigError(location, "must be a non-empty string")
    return value


def boolean(value: object, location: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(location, "must be a boolean")
    return value


def integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(location, "must be an integer")
    if not minimum <= value <= maximum:
        raise ConfigError(location, f"must be within [{minimum}, {maximum}]")
    return value


def number(
    value: object, location: str, *, minimum: float = 0.0, maximum: float = float("inf")
) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(location, "must be a finite number")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ConfigError(location, f"must be a finite number within [{minimum}, {maximum}]")
    return result


def path_value(value: object, location: str) -> Path:
    return Path(text(value, location))


def name(value: object, location: str) -> str:
    result = text(value, location)
    if _NAME.fullmatch(result) is None:
        raise ConfigError(location, "contains unsupported characters")
    return result


def sha256(value: object, location: str) -> str:
    result = text(value, location)
    if _SHA256.fullmatch(result) is None:
        raise ConfigError(location, "must be a lowercase SHA-256 digest")
    return result


def names(value: object, location: str, *, count: int | None = None) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise ConfigError(location, "must be an array of names")
    result = tuple(name(item, f"{location}[{index}]") for index, item in enumerate(value))
    if len(set(result)) != len(result):
        raise ConfigError(location, "must be unique")
    if count is not None and len(result) != count:
        raise ConfigError(location, f"must contain exactly {count} names")
    return result


@dataclass(frozen=True, slots=True)
class RGBEncoderSpec:
    vcodec: str = "h264"
    g: int = 2
    crf: int = 18
    preset: str = "veryfast"

    def __post_init__(self) -> None:
        if self.vcodec != "h264":
            raise ConfigError("dataset.lerobot.rgb_encoder.vcodec", "must be h264")
        if self.g != 2:
            raise ConfigError("dataset.lerobot.rgb_encoder.g", "must be 2")
        if self.crf != 18:
            raise ConfigError("dataset.lerobot.rgb_encoder.crf", "must be 18")
        if self.preset != "veryfast":
            raise ConfigError("dataset.lerobot.rgb_encoder.preset", "must be veryfast")


@dataclass(frozen=True, slots=True)
class DatasetConfig:
    """Conversion contract and feature source for one learning data root."""

    project_root: Path
    fps: int
    chunk_size: int
    action_lead_ns: int
    state_dim: int
    camera_names: tuple[str, ...]
    camera_width: int
    camera_height: int
    profile_path: Path
    select_path: Path
    output_root: Path
    cache_root: Path
    attempt_export_glob: str
    expected_formal_episodes: int
    expected_training_episodes: int
    expected_validation_episodes: int
    validation_positions: tuple[int, ...]
    lerobot_source_revision: str
    lerobot_source_archive_sha256: str
    rgb_encoder: RGBEncoderSpec
    video_backend: str
    video_files_size_mb: int
    data_files_size_mb: int
    encoder_threads: int
    image_writer_threads: int
    verify_png_pixels: bool
    require_quality: bool
    task_text: str = "Pick up the red source cube and place it on the blue target cube."
    schema_version: int = 1

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ConfigError("dataset.schema_version", "must be 1")
        if self.fps != 20:
            raise ConfigError("dataset.fps", "this contract requires 20 Hz")
        if self.chunk_size != 60:
            raise ConfigError("dataset.chunk_size", "this contract requires 60 future actions")
        if self.action_lead_ns != 50_000_000:
            raise ConfigError("dataset.action_lead_ns", "labels are the reference at +50 ms")
        if self.state_dim != 16:
            raise ConfigError("dataset.state_dim", "the panda learning profile has 16 joints")
        if self.camera_names != ("global", "left_wrist", "right_wrist"):
            raise ConfigError("dataset.camera_names", "must be global/left_wrist/right_wrist")
        if (self.camera_width, self.camera_height) != (320, 240):
            raise ConfigError("dataset.camera_width", "three RGB views must be 320x240")
        if self.expected_formal_episodes != 50:
            raise ConfigError("dataset.expected_formal_episodes", "formal set is exactly 50")
        if self.expected_training_episodes != 45:
            raise ConfigError("dataset.expected_training_episodes", "training split is exactly 45")
        if self.expected_validation_episodes != 5:
            raise ConfigError("dataset.expected_validation_episodes", "validation split is 5")
        if self.validation_positions != (9, 19, 29, 39, 49):
            raise ConfigError(
                "dataset.validation_positions", "must be the 10th/20th/30th/40th/50th successes"
            )
        if self.lerobot_source_revision != "7e241bd630a3719a56157a497ce5d08f244784f1":
            raise ConfigError("dataset.lerobot.source_revision", "must be the pinned revision")
        if self.lerobot_source_archive_sha256 != (
            "710397ad753090743c1aadf04b2bd40584fbca8448b1409131138172913c8a0c"
        ):
            raise ConfigError(
                "dataset.lerobot.source_archive_sha256", "must match the frozen export"
            )
        if self.video_backend != "torchcodec":
            raise ConfigError("dataset.lerobot.video_backend", "must be torchcodec")
        if self.attempt_export_glob.count("*") > 1 or not self.attempt_export_glob.endswith(
            "export"
        ):
            raise ConfigError(
                "dataset.attempt_export_glob", "must be a single bounded glob ending in 'export'"
            )

    @property
    def profile_sha256(self) -> str:
        return hashlib.sha256(self.profile_path.read_bytes()).hexdigest()

    def feature_spec(self) -> dict[str, dict[str, object]]:
        features: dict[str, dict[str, object]] = {
            "observation.state": {
                "dtype": "float32",
                "shape": (self.state_dim,),
            },
            "observation.velocity": {
                "dtype": "float32",
                "shape": (self.state_dim,),
            },
            "action": {"dtype": "float32", "shape": (self.state_dim,)},
            "raw_action_chunk": {"dtype": "float32", "shape": (self.chunk_size, self.state_dim)},
            "observation.sim_time_ns": {"dtype": "int64", "shape": (1,)},
            "action.sim_time_ns": {"dtype": "int64", "shape": (1,)},
            "source.episode_index": {"dtype": "int64", "shape": (1,)},
            "source.seed": {"dtype": "int64", "shape": (1,)},
        }
        for camera in self.camera_names:
            features[f"observation.images.{camera}"] = {
                "dtype": "video",
                "shape": (self.camera_height, self.camera_width, 3),
            }
        return features


def parse_dataset_config(document: object, project_root: str | Path) -> DatasetConfig:
    root = Path(project_root).resolve()
    value = mapping(document, "<dataset>")
    reject_unknown(
        value,
        {
            "schema_version",
            "fps",
            "chunk_size",
            "action_lead_ns",
            "state_dim",
            "camera_names",
            "camera_width",
            "camera_height",
            "profile",
            "select",
            "output_root",
            "cache_root",
            "attempt_export_glob",
            "expected",
            "lerobot",
            "verify_png_pixels",
            "require_quality",
            "task_text",
        },
        "<dataset>",
    )
    expected = mapping(require(value, "expected", "<dataset>"), "dataset.expected")
    reject_unknown(
        expected,
        {
            "formal_episodes",
            "training_episodes",
            "validation_episodes",
            "validation_positions",
        },
        "dataset.expected",
    )
    positions = require(expected, "validation_positions", "dataset.expected")
    if not isinstance(positions, list):
        raise ConfigError("dataset.expected.validation_positions", "must be an array")
    position_values = tuple(
        integer(item, f"dataset.expected.validation_positions[{index}]", 0, 49)
        for index, item in enumerate(positions)
    )
    lerobot = mapping(require(value, "lerobot", "<dataset>"), "dataset.lerobot")
    reject_unknown(
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
        "dataset.lerobot",
    )
    encoder = mapping(
        require(lerobot, "rgb_encoder", "dataset.lerobot"), "dataset.lerobot.rgb_encoder"
    )
    reject_unknown(encoder, {"vcodec", "g", "crf", "preset"}, "dataset.lerobot.rgb_encoder")
    return DatasetConfig(
        project_root=root,
        schema_version=integer(
            require(value, "schema_version", "<dataset>"), "dataset.schema_version", 1, 1
        ),
        fps=integer(require(value, "fps", "<dataset>"), "dataset.fps", 1, 240),
        chunk_size=integer(require(value, "chunk_size", "<dataset>"), "dataset.chunk_size", 1, 600),
        action_lead_ns=integer(
            require(value, "action_lead_ns", "<dataset>"),
            "dataset.action_lead_ns",
            1,
            10_000_000_000,
        ),
        state_dim=integer(require(value, "state_dim", "<dataset>"), "dataset.state_dim", 1, 256),
        camera_names=names(
            require(value, "camera_names", "<dataset>"), "dataset.camera_names", count=3
        ),
        camera_width=integer(
            require(value, "camera_width", "<dataset>"), "dataset.camera_width", 1, 4096
        ),
        camera_height=integer(
            require(value, "camera_height", "<dataset>"), "dataset.camera_height", 1, 4096
        ),
        profile_path=root / path_value(require(value, "profile", "<dataset>"), "dataset.profile"),
        select_path=root / path_value(require(value, "select", "<dataset>"), "dataset.select"),
        output_root=root
        / path_value(require(value, "output_root", "<dataset>"), "dataset.output_root"),
        cache_root=root
        / path_value(require(value, "cache_root", "<dataset>"), "dataset.cache_root"),
        attempt_export_glob=text(
            require(value, "attempt_export_glob", "<dataset>"), "dataset.attempt_export_glob"
        ),
        expected_formal_episodes=integer(
            require(expected, "formal_episodes", "dataset.expected"),
            "dataset.expected.formal_episodes",
            1,
            10000,
        ),
        expected_training_episodes=integer(
            require(expected, "training_episodes", "dataset.expected"),
            "dataset.expected.training_episodes",
            1,
            10000,
        ),
        expected_validation_episodes=integer(
            require(expected, "validation_episodes", "dataset.expected"),
            "dataset.expected.validation_episodes",
            1,
            10000,
        ),
        validation_positions=position_values,
        lerobot_source_revision=text(
            require(lerobot, "source_revision", "dataset.lerobot"),
            "dataset.lerobot.source_revision",
        ),
        lerobot_source_archive_sha256=sha256(
            require(lerobot, "source_archive_sha256", "dataset.lerobot"),
            "dataset.lerobot.source_archive_sha256",
        ),
        rgb_encoder=RGBEncoderSpec(
            vcodec=text(require(encoder, "vcodec", "dataset.lerobot.rgb_encoder"), "vcodec"),
            g=integer(require(encoder, "g", "dataset.lerobot.rgb_encoder"), "g", 1, 600),
            crf=integer(require(encoder, "crf", "dataset.lerobot.rgb_encoder"), "crf", 0, 63),
            preset=text(require(encoder, "preset", "dataset.lerobot.rgb_encoder"), "preset"),
        ),
        video_backend=text(
            require(lerobot, "video_backend", "dataset.lerobot"), "dataset.lerobot.video_backend"
        ),
        video_files_size_mb=integer(
            require(lerobot, "video_files_size_mb", "dataset.lerobot"),
            "dataset.lerobot.video_files_size_mb",
            16,
            16384,
        ),
        data_files_size_mb=integer(
            require(lerobot, "data_files_size_mb", "dataset.lerobot"),
            "dataset.lerobot.data_files_size_mb",
            16,
            16384,
        ),
        encoder_threads=integer(
            require(lerobot, "encoder_threads", "dataset.lerobot"),
            "dataset.lerobot.encoder_threads",
            0,
            64,
        ),
        image_writer_threads=integer(
            require(lerobot, "image_writer_threads", "dataset.lerobot"),
            "dataset.lerobot.image_writer_threads",
            0,
            64,
        ),
        verify_png_pixels=boolean(
            require(value, "verify_png_pixels", "<dataset>"), "dataset.verify_png_pixels"
        ),
        require_quality=boolean(
            require(value, "require_quality", "<dataset>"), "dataset.require_quality"
        ),
        task_text=(
            "Pick up the red source cube and place it on the blue target cube."
            if value.get("task_text") is None
            else text(value["task_text"], "dataset.task_text")
        ),
    )


def load_dataset_config(path: str | Path, project_root: str | Path) -> DatasetConfig:
    return parse_dataset_config(load_yaml(path), project_root)


@dataclass(frozen=True, slots=True)
class ModelSpec:
    vision_backbone: str = "resnet18"
    dim_model: int = 256
    n_heads: int = 8
    dim_feedforward: int = 1024
    n_encoder_layers: int = 4
    n_decoder_layers: int = 1
    use_vae: bool = True
    latent_dim: int = 4
    n_vae_encoder_layers: int = 4
    dropout: float = 0.1
    kl_weight: float = 10.0
    pretrained_backbone_weights: str | None = None
    weights_path: Path = Path("")
    weights_sha256: str = ""

    def __post_init__(self) -> None:
        if self.pretrained_backbone_weights is not None:
            raise ConfigError(
                "model.pretrained_backbone_weights",
                "must be null; the local official file is loaded explicitly after construction",
            )
        if self.vision_backbone != "resnet18":
            raise ConfigError("model.vision_backbone", "must be resnet18")
        if not self.weights_sha256:
            raise ConfigError("model.weights.sha256", "field is required")


@dataclass(frozen=True, slots=True)
class ModeSpec:
    steps: int
    split: str
    episodes: tuple[int, ...] | None
    validation: bool
    reserve_seconds: float


@dataclass(frozen=True, slots=True)
class BudgetSpec:
    limit_seconds: float
    ledger: Path
    lock: Path
    stop_margin_seconds: float
    min_free_bytes: int


@dataclass(frozen=True, slots=True)
class CheckpointSpec:
    root: Path
    keep_latest: bool
    keep_best: bool
    keep_final: bool
    min_free_bytes: int
    save_before_exit: bool


@dataclass(frozen=True, slots=True)
class LoggingSpec:
    root: Path
    component: str
    queue_capacity: int
    max_event_bytes: int
    max_file_bytes: int
    max_total_bytes: int
    close_timeout_s: float
    flush_interval_s: float
    console: bool


@dataclass(frozen=True, slots=True)
class TrainingConfig:
    project_root: Path
    source: Path
    mode: str
    seed: int
    device: str
    dataset_config_path: Path
    data_root: Path
    cache_root: Path
    manifest_name: str
    train_dir: str
    validation_dir: str
    verify_hashes: bool
    batch_size: int
    num_workers: int
    lr: float
    weight_decay: float
    grad_clip_norm: float
    log_every_steps: int
    val_every_steps: int
    save_every_steps: int
    val_batches: int
    dtype: str
    amp: bool
    deterministic: bool
    modes: dict[str, ModeSpec]
    model: ModelSpec
    budget: BudgetSpec
    checkpoint: CheckpointSpec
    logging: LoggingSpec
    dataset: DatasetConfig = field(default=None)  # type: ignore[assignment]
    sample_filter: Path | None = None
    schema_version: int = 1

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ConfigError("schema_version", "must be 1")
        if self.mode not in self.modes:
            raise ConfigError("mode", f"must be one of {sorted(self.modes)}")
        if self.dtype != "float32":
            raise ConfigError("dtype", "the 6 GiB target requires float32")
        if self.amp:
            raise ConfigError("amp", "mixed precision is not part of this contract")

    @property
    def mode_spec(self) -> ModeSpec:
        return self.modes[self.mode]

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.source.read_bytes()).hexdigest()


def _load_model(value: object, project_root: Path) -> ModelSpec:
    model = mapping(value, "model")
    reject_unknown(
        model,
        {
            "vision_backbone",
            "dim_model",
            "n_heads",
            "dim_feedforward",
            "n_encoder_layers",
            "n_decoder_layers",
            "use_vae",
            "latent_dim",
            "n_vae_encoder_layers",
            "dropout",
            "kl_weight",
            "pretrained_backbone_weights",
            "weights",
        },
        "model",
    )
    weights = mapping(require(model, "weights", "model"), "model.weights")
    reject_unknown(weights, {"path", "sha256"}, "model.weights")
    return ModelSpec(
        vision_backbone=text(require(model, "vision_backbone", "model"), "model.vision_backbone"),
        dim_model=integer(require(model, "dim_model", "model"), "model.dim_model", 1, 4096),
        n_heads=integer(require(model, "n_heads", "model"), "model.n_heads", 1, 128),
        dim_feedforward=integer(
            require(model, "dim_feedforward", "model"), "model.dim_feedforward", 1, 16384
        ),
        n_encoder_layers=integer(
            require(model, "n_encoder_layers", "model"), "model.n_encoder_layers", 1, 64
        ),
        n_decoder_layers=integer(
            require(model, "n_decoder_layers", "model"), "model.n_decoder_layers", 1, 64
        ),
        use_vae=boolean(require(model, "use_vae", "model"), "model.use_vae"),
        latent_dim=integer(require(model, "latent_dim", "model"), "model.latent_dim", 1, 1024),
        n_vae_encoder_layers=integer(
            require(model, "n_vae_encoder_layers", "model"), "model.n_vae_encoder_layers", 1, 64
        ),
        dropout=number(
            require(model, "dropout", "model"), "model.dropout", minimum=0.0, maximum=0.9
        ),
        kl_weight=number(require(model, "kl_weight", "model"), "model.kl_weight", minimum=0.0),
        pretrained_backbone_weights=(
            None
            if require(model, "pretrained_backbone_weights", "model") is None
            else text(model["pretrained_backbone_weights"], "model.pretrained_backbone_weights")
        ),
        weights_path=project_root
        / path_value(require(weights, "path", "model.weights"), "model.weights.path"),
        weights_sha256=sha256(require(weights, "sha256", "model.weights"), "model.weights.sha256"),
    )


def parse_training_config(
    document: object, *, project_root: str | Path, source: str | Path
) -> TrainingConfig:
    root = Path(project_root).resolve()
    value = mapping(document, "<training>")
    reject_unknown(
        value,
        {
            "schema_version",
            "mode",
            "seed",
            "device",
            "data",
            "cache_root",
            "model",
            "training",
            "modes",
            "budget",
            "checkpoint",
            "logging",
        },
        "<training>",
    )
    data = mapping(require(value, "data", "<training>"), "data")
    reject_unknown(
        data,
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
    dataset_path = root / path_value(require(data, "config", "data"), "data.config")
    dataset = load_dataset_config(dataset_path, root)
    training = mapping(require(value, "training", "<training>"), "training")
    reject_unknown(
        training,
        {
            "batch_size",
            "num_workers",
            "lr",
            "weight_decay",
            "grad_clip_norm",
            "log_every_steps",
            "val_every_steps",
            "save_every_steps",
            "val_batches",
            "dtype",
            "amp",
            "deterministic",
        },
        "training",
    )
    modes_value = mapping(require(value, "modes", "<training>"), "modes")
    if set(modes_value) != {"smoke", "overfit", "full"}:
        raise ConfigError("modes", "must define smoke, overfit and full")
    modes: dict[str, ModeSpec] = {}
    for mode_name, raw in modes_value.items():
        entry = mapping(raw, f"modes.{mode_name}")
        reject_unknown(
            entry,
            {"steps", "split", "episodes", "validation", "reserve_seconds"},
            f"modes.{mode_name}",
        )
        episodes_raw = require(entry, "episodes", f"modes.{mode_name}")
        episodes: tuple[int, ...] | None
        if episodes_raw == "all":
            episodes = None
        elif isinstance(episodes_raw, list):
            episodes = tuple(
                integer(item, f"modes.{mode_name}.episodes[{index}]", 0, 9999)
                for index, item in enumerate(episodes_raw)
            )
            if len(set(episodes)) != len(episodes):
                raise ConfigError(f"modes.{mode_name}.episodes", "must be unique")
        else:
            raise ConfigError(f"modes.{mode_name}.episodes", "must be 'all' or an array")
        split = text(require(entry, "split", f"modes.{mode_name}"), f"modes.{mode_name}.split")
        if split not in {"train", "validation"}:
            raise ConfigError(f"modes.{mode_name}.split", "must be train or validation")
        modes[mode_name] = ModeSpec(
            steps=integer(
                require(entry, "steps", f"modes.{mode_name}"),
                f"modes.{mode_name}.steps",
                1,
                1_000_000,
            ),
            split=split,
            episodes=episodes,
            validation=boolean(
                require(entry, "validation", f"modes.{mode_name}"), f"modes.{mode_name}.validation"
            ),
            reserve_seconds=number(
                require(entry, "reserve_seconds", f"modes.{mode_name}"),
                f"modes.{mode_name}.reserve_seconds",
                minimum=1.0,
            ),
        )
    if modes["full"].episodes is not None:
        raise ConfigError("modes.full.episodes", "full mode must use all 45 training episodes")
    if not modes["full"].validation or modes["full"].split != "train":
        raise ConfigError(
            "modes.full", "full mode must validate on the independent 5-episode split"
        )
    budget_value = mapping(require(value, "budget", "<training>"), "budget")
    reject_unknown(
        budget_value,
        {"limit_seconds", "ledger", "lock", "stop_margin_seconds", "min_free_bytes"},
        "budget",
    )
    checkpoint_value = mapping(require(value, "checkpoint", "<training>"), "checkpoint")
    reject_unknown(
        checkpoint_value,
        {"root", "keep_latest", "keep_best", "keep_final", "min_free_bytes", "save_before_exit"},
        "checkpoint",
    )
    logging_value = mapping(require(value, "logging", "<training>"), "logging")
    reject_unknown(
        logging_value,
        {
            "root",
            "component",
            "queue_capacity",
            "max_event_bytes",
            "max_file_bytes",
            "max_total_bytes",
            "close_timeout_s",
            "flush_interval_s",
            "console",
        },
        "logging",
    )
    mode = text(require(value, "mode", "<training>"), "mode")
    sample_filter_value = data.get("sample_filter")
    sample_filter = (
        None
        if sample_filter_value is None
        else root / path_value(sample_filter_value, "data.sample_filter")
    )
    config = TrainingConfig(
        project_root=root,
        source=Path(source),
        mode=mode,
        seed=integer(require(value, "seed", "<training>"), "seed", 0, 2**31 - 1),
        device=text(require(value, "device", "<training>"), "device"),
        dataset_config_path=dataset_path,
        data_root=root / path_value(require(data, "root", "data"), "data.root"),
        cache_root=root
        / path_value(
            value.get("cache_root", ".build/cache/huggingface-learning"),
            "cache_root",
        ),
        manifest_name=text(require(data, "manifest", "data"), "data.manifest"),
        train_dir=text(require(data, "train_dir", "data"), "data.train_dir"),
        validation_dir=text(require(data, "validation_dir", "data"), "data.validation_dir"),
        verify_hashes=boolean(require(data, "verify_hashes", "data"), "data.verify_hashes"),
        batch_size=integer(
            require(training, "batch_size", "training"), "training.batch_size", 1, 256
        ),
        num_workers=integer(
            require(training, "num_workers", "training"), "training.num_workers", 0, 32
        ),
        lr=number(require(training, "lr", "training"), "training.lr", minimum=1e-12),
        weight_decay=number(
            require(training, "weight_decay", "training"), "training.weight_decay", minimum=0.0
        ),
        grad_clip_norm=number(
            require(training, "grad_clip_norm", "training"), "training.grad_clip_norm", minimum=0.0
        ),
        log_every_steps=integer(
            require(training, "log_every_steps", "training"),
            "training.log_every_steps",
            1,
            1_000_000,
        ),
        val_every_steps=integer(
            require(training, "val_every_steps", "training"),
            "training.val_every_steps",
            1,
            1_000_000,
        ),
        save_every_steps=integer(
            require(training, "save_every_steps", "training"),
            "training.save_every_steps",
            1,
            1_000_000,
        ),
        val_batches=integer(
            require(training, "val_batches", "training"), "training.val_batches", 1, 10000
        ),
        dtype=text(require(training, "dtype", "training"), "training.dtype"),
        amp=boolean(require(training, "amp", "training"), "training.amp"),
        deterministic=boolean(
            require(training, "deterministic", "training"), "training.deterministic"
        ),
        modes=modes,
        model=_load_model(require(value, "model", "<training>"), root),
        budget=BudgetSpec(
            limit_seconds=number(
                require(budget_value, "limit_seconds", "budget"),
                "budget.limit_seconds",
                minimum=1.0,
            ),
            ledger=root / path_value(require(budget_value, "ledger", "budget"), "budget.ledger"),
            lock=root / path_value(require(budget_value, "lock", "budget"), "budget.lock"),
            stop_margin_seconds=number(
                require(budget_value, "stop_margin_seconds", "budget"),
                "budget.stop_margin_seconds",
                minimum=0.0,
            ),
            min_free_bytes=integer(
                require(budget_value, "min_free_bytes", "budget"),
                "budget.min_free_bytes",
                0,
                2**63 - 1,
            ),
        ),
        checkpoint=CheckpointSpec(
            root=root
            / path_value(require(checkpoint_value, "root", "checkpoint"), "checkpoint.root"),
            keep_latest=boolean(
                require(checkpoint_value, "keep_latest", "checkpoint"), "checkpoint.keep_latest"
            ),
            keep_best=boolean(
                require(checkpoint_value, "keep_best", "checkpoint"), "checkpoint.keep_best"
            ),
            keep_final=boolean(
                require(checkpoint_value, "keep_final", "checkpoint"), "checkpoint.keep_final"
            ),
            min_free_bytes=integer(
                require(checkpoint_value, "min_free_bytes", "checkpoint"),
                "checkpoint.min_free_bytes",
                0,
                2**63 - 1,
            ),
            save_before_exit=boolean(
                require(checkpoint_value, "save_before_exit", "checkpoint"),
                "checkpoint.save_before_exit",
            ),
        ),
        logging=LoggingSpec(
            root=root / path_value(require(logging_value, "root", "logging"), "logging.root"),
            component=name(require(logging_value, "component", "logging"), "logging.component"),
            queue_capacity=integer(
                require(logging_value, "queue_capacity", "logging"),
                "logging.queue_capacity",
                1,
                65536,
            ),
            max_event_bytes=integer(
                require(logging_value, "max_event_bytes", "logging"),
                "logging.max_event_bytes",
                1024,
                2**24,
            ),
            max_file_bytes=integer(
                require(logging_value, "max_file_bytes", "logging"),
                "logging.max_file_bytes",
                1024,
                2**30,
            ),
            max_total_bytes=integer(
                require(logging_value, "max_total_bytes", "logging"),
                "logging.max_total_bytes",
                1024,
                2**34,
            ),
            close_timeout_s=number(
                require(logging_value, "close_timeout_s", "logging"),
                "logging.close_timeout_s",
                minimum=0.1,
            ),
            flush_interval_s=number(
                require(logging_value, "flush_interval_s", "logging"),
                "logging.flush_interval_s",
                minimum=0.01,
            ),
            console=boolean(require(logging_value, "console", "logging"), "logging.console"),
        ),
        dataset=dataset,
        sample_filter=sample_filter,
    )
    if config.budget.limit_seconds > 14400.0:
        raise ConfigError("budget.limit_seconds", "task training budget is capped at 14400 s")
    return config


def load_training_config(path: str | Path, project_root: str | Path) -> TrainingConfig:
    return parse_training_config(load_yaml(path), project_root=project_root, source=path)
