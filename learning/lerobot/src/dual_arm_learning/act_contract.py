"""Offline, framework-free verification of one trained ACT deployment bundle.

This module is intentionally small and Torch-free.  It is used by the deployment
launcher to read the real bundle digests before Compose starts, and by the ACT
provider entry to refuse a bundle whose profile, observation/action schemas,
processor statistics or 45/5 dataset facts do not match the shipped files.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

BUNDLE_FORMAT = "dual-arm-act-bundle-v1"
ACT_CAMERAS = ("global", "left_wrist", "right_wrist")
ACT_STATE_DIM = 16
ACT_CHUNK_SIZE = 60
ACT_ACTION_PERIOD_NS = 50_000_000
ACT_FPS = 20
ACT_IMAGE_HEIGHT = 240
ACT_IMAGE_WIDTH = 320
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")


class ActContractError(ValueError):
    """A bundle or generated deployment configuration is not deployable."""


@dataclass(frozen=True)
class ActSettings:
    """Deployment inputs; every identity is derived from the bundle/files."""

    schema_version: int
    bundle: Path
    profile: Path
    observation_schema: Path
    action_schema: Path
    device: str
    require_formal: bool
    bind_address: str
    port: int
    max_workers: int
    max_message_bytes: int
    max_sessions: int
    max_open_requests: int
    max_request_cache: int
    predict_deadline_s: float
    open_deadline_s: float
    close_deadline_s: float
    test_only: bool
    protocol_major: int
    stateful: bool
    task_id: str
    task_text: str
    logging: dict[str, Any]
    gripper_max_correction_m: float = 0.0
    temporal_ensemble_coeff: float | None = None


@dataclass(frozen=True)
class ActContract:
    """All verified identities and limits needed to configure the service."""

    bundle_root: Path
    bundle_digest: str
    model_sha256: str
    state_dict_sha256: str
    config_sha256: str
    processor_digest: str
    stats_sha256: str
    profile_file_sha256: str
    profile_semantic_sha256: str
    observation_schema_hash: str
    action_schema_hash: str
    joint_names: tuple[str, ...]
    joint_lower_bounds: tuple[float, ...]
    joint_upper_bounds: tuple[float, ...]
    camera_names: tuple[str, ...]
    camera_width: int
    camera_height: int
    chunk_size: int
    state_dim: int
    action_period_ns: int
    action_lead_ns: int
    fps: int
    checkpoint_step: int
    checkpoint_mode: str
    dataset_seed: int
    train_episodes: int
    validation_episodes: int
    train_global_indices: tuple[int, ...]
    validation_global_indices: tuple[int, ...]
    synthetic: bool
    profile: dict[str, Any]
    observation: dict[str, Any]
    action: dict[str, Any]
    lerobot_revision: str = ""
    lerobot_archive_sha256: str = ""
    training_config_sha256: str = ""

    def summary(self) -> dict[str, Any]:
        """JSON-safe summary for logs and the attempt snapshot, never tensors."""
        return {
            "format": BUNDLE_FORMAT,
            "bundle_digest": self.bundle_digest,
            "model_sha256": self.model_sha256,
            "config_sha256": self.config_sha256,
            "processor_digest": self.processor_digest,
            "stats_sha256": self.stats_sha256,
            "profile_semantic_sha256": self.profile_semantic_sha256,
            "observation_schema_hash": self.observation_schema_hash,
            "action_schema_hash": self.action_schema_hash,
            "checkpoint_step": self.checkpoint_step,
            "checkpoint_mode": self.checkpoint_mode,
            "dataset_seed": self.dataset_seed,
            "train_episodes": self.train_episodes,
            "validation_episodes": self.validation_episodes,
            "synthetic": self.synthetic,
            "lerobot_revision": self.lerobot_revision,
            "lerobot_archive_sha256": self.lerobot_archive_sha256,
            "training_config_sha256": self.training_config_sha256,
            "chunk_size": self.chunk_size,
            "state_dim": self.state_dim,
            "action_period_ns": self.action_period_ns,
            "action_lead_ns": self.action_lead_ns,
            "fps": self.fps,
            "cameras": list(self.camera_names),
            "camera_width": self.camera_width,
            "camera_height": self.camera_height,
        }


def _read_json(path: Path) -> dict[str, Any]:
    def reject_constant(value: str) -> Any:
        raise ActContractError(f"{path}: non-finite JSON constant {value!r}")

    try:
        value = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)
    except OSError as error:
        raise ActContractError(f"{path}: cannot read: {error}") from error
    except json.JSONDecodeError as error:
        raise ActContractError(f"{path}: invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ActContractError(f"{path}: expected a JSON object")
    return value


def _canonical_hash(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _tree_hashes(root: Path) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if path.is_symlink():
            raise ActContractError(f"{path}: symlinks are not allowed in a deployment bundle")
        result[path.relative_to(root).as_posix()] = {
            "sha256": _file_sha256(path),
            "bytes": path.stat().st_size,
        }
    return result


def _processor_digest(files: dict[str, dict[str, Any]]) -> str:
    """Match bundle._processor_digest: names are relative to processors/."""
    digest = hashlib.sha256()
    records = {
        name.removeprefix("processors/"): record
        for name, record in files.items()
        if name.startswith("processors/")
    }
    for name, record in sorted(records.items()):
        digest.update(name.encode())
        digest.update(str(record["sha256"]).encode())
    return digest.hexdigest()


def _text(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise ActContractError(f"{location}: must be a non-empty string")
    return value


def _digest(value: object, location: str) -> str:
    result = _text(value, location)
    if _SHA256.fullmatch(result) is None:
        raise ActContractError(f"{location}: must be a lowercase SHA-256 digest")
    return result


def _integer(value: object, location: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ActContractError(f"{location}: must be an integer")
    if value < minimum:
        raise ActContractError(f"{location}: must be >= {minimum}")
    return value


def _number(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ActContractError(f"{location}: must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ActContractError(f"{location}: must be finite")
    return number


def _boolean(value: object, location: str) -> bool:
    if not isinstance(value, bool):
        raise ActContractError(f"{location}: must be a boolean")
    return value


def _mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ActContractError(f"{location}: must be an object")
    if any(not isinstance(key, str) for key in value):
        raise ActContractError(f"{location}: keys must be strings")
    return value


def _names(value: object, location: str, *, count: int) -> tuple[str, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ActContractError(f"{location}: must contain exactly {count} names")
    result: list[str] = []
    for index, item in enumerate(value):
        if not isinstance(item, str) or _NAME.fullmatch(item) is None:
            raise ActContractError(f"{location}[{index}]: invalid name")
        if item in result:
            raise ActContractError(f"{location}[{index}]: duplicate name")
        result.append(item)
    return tuple(result)


def _reject_non_finite(value: object, location: str) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite(item, f"{location}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_non_finite(item, f"{location}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise ActContractError(f"{location}: must be finite")


def _safe_relative(root: Path, name: object, location: str) -> Path:
    text = _text(name, location)
    candidate = (root / text).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError as error:
        raise ActContractError(f"{location}: must stay inside the bundle") from error
    if not candidate.is_file():
        raise ActContractError(f"{location}: missing file {candidate}")
    return candidate


def _profile_contract(
    profile: dict[str, Any], external: dict[str, Any], location: str
) -> tuple[tuple[str, ...], tuple[float, ...], tuple[float, ...], tuple[str, ...], int, int]:
    if profile != external:
        raise ActContractError(f"{location}: bundle profile differs from the shipped profile")
    if profile.get("schema_version") != 2:
        raise ActContractError(f"{location}.schema_version: must be 2")
    if profile.get("test_only") is not False:
        raise ActContractError(f"{location}.test_only: formal deployment needs a real profile")
    joints = profile.get("joints")
    if not isinstance(joints, list) or len(joints) != ACT_STATE_DIM:
        raise ActContractError(f"{location}.joints: must contain 16 joints")
    names: list[str] = []
    lower: list[float] = []
    upper: list[float] = []
    for index, joint in enumerate(joints):
        item = _mapping(joint, f"{location}.joints[{index}]")
        name = _text(item.get("name"), f"{location}.joints[{index}].name")
        if _NAME.fullmatch(name) is None:
            raise ActContractError(f"{location}.joints[{index}].name: invalid")
        low = _number(item.get("min"), f"{location}.joints[{index}].min")
        high = _number(item.get("max"), f"{location}.joints[{index}].max")
        if not low < high:
            raise ActContractError(f"{location}.joints[{index}]: min must be below max")
        names.append(name)
        lower.append(low)
        upper.append(high)
    if len(set(names)) != len(names):
        raise ActContractError(f"{location}.joints: names must be unique")
    cameras = profile.get("cameras")
    if not isinstance(cameras, list) or len(cameras) != 3:
        raise ActContractError(f"{location}.cameras: must contain exactly three cameras")
    camera_names: list[str] = []
    for index, camera in enumerate(cameras):
        item = _mapping(camera, f"{location}.cameras[{index}]")
        name = _text(item.get("name"), f"{location}.cameras[{index}].name")
        if _NAME.fullmatch(name) is None:
            raise ActContractError(f"{location}.cameras[{index}].name: invalid")
        if (item.get("width"), item.get("height")) != (ACT_IMAGE_WIDTH, ACT_IMAGE_HEIGHT):
            raise ActContractError(f"{location}.cameras[{index}]: must be 320x240")
        _digest(item.get("calibration_digest"), f"{location}.cameras[{index}].calibration_digest")
        _text(item.get("frame_id"), f"{location}.cameras[{index}].frame_id")
        camera_names.append(name)
    if tuple(camera_names) != ACT_CAMERAS:
        raise ActContractError(f"{location}.cameras: order must be {list(ACT_CAMERAS)}")
    return (
        tuple(names),
        tuple(lower),
        tuple(upper),
        tuple(camera_names),
        ACT_IMAGE_WIDTH,
        ACT_IMAGE_HEIGHT,
    )


def _schema_contract(
    document: dict[str, Any],
    location: str,
    *,
    joint_names: tuple[str, ...],
    camera_names: tuple[str, ...],
) -> None:
    if document.get("joint_names") != list(joint_names):
        raise ActContractError(f"{location}.joint_names: order differs")
    if "camera_names" in document and document.get("camera_names") != list(camera_names):
        raise ActContractError(f"{location}.camera_names: order differs")


def _stats_contract(stats: dict[str, Any], location: str) -> str:
    _reject_non_finite(stats, location)
    if stats.get("source") != "training_split_only":
        raise ActContractError(f"{location}.source: must prove training-split-only statistics")
    features = _mapping(stats.get("features"), f"{location}.features")
    for name in ("observation.state", "action"):
        feature = _mapping(features.get(name), f"{location}.features.{name}")
        for field in ("mean", "std", "min", "max"):
            values = feature.get(field)
            if not isinstance(values, list) or len(values) != ACT_STATE_DIM:
                raise ActContractError(
                    f"{location}.features.{name}.{field}: must contain 16 finite numbers"
                )
            for index, value in enumerate(values):
                _number(value, f"{location}.features.{name}.{field}[{index}]")
    return _canonical_hash(stats)


def _dataset_contract(
    manifest: dict[str, Any],
    document: dict[str, Any],
    *,
    profile_semantic_sha256: str,
    require_formal: bool,
) -> tuple[int, int, tuple[int, ...], tuple[int, ...], bool]:
    if document.get("status") != "complete":
        raise ActContractError("metadata/dataset_manifest.json: status must be complete")
    if document.get("schema_version") != 1:
        raise ActContractError("metadata/dataset_manifest.json: schema_version must be 1")
    synthetic = bool(document.get("synthetic"))
    if require_formal and synthetic:
        raise ActContractError("synthetic dataset evidence cannot deploy a formal ACT model")
    profile = _mapping(document.get("profile"), "dataset_manifest.profile")
    if profile.get("semantic_sha256") != profile_semantic_sha256:
        raise ActContractError("dataset_manifest.profile: semantic hash differs")
    lerobot = _mapping(document.get("lerobot"), "dataset_manifest.lerobot")
    if _integer(lerobot.get("chunk_size"), "dataset_manifest.lerobot.chunk_size") != ACT_CHUNK_SIZE:
        raise ActContractError("dataset_manifest.lerobot.chunk_size: must be 60")
    if (
        _integer(lerobot.get("action_lead_ns"), "dataset_manifest.lerobot.action_lead_ns")
        != ACT_ACTION_PERIOD_NS
    ):
        raise ActContractError("dataset_manifest.lerobot.action_lead_ns: must be 50 ms")
    if _integer(lerobot.get("fps"), "dataset_manifest.lerobot.fps") != ACT_FPS:
        raise ActContractError("dataset_manifest.lerobot.fps: must be 20")
    splits = _mapping(document.get("splits"), "dataset_manifest.splits")
    training = _mapping(splits.get("training"), "dataset_manifest.splits.training")
    validation = _mapping(splits.get("validation"), "dataset_manifest.splits.validation")
    train_count = _integer(training.get("episode_count"), "training.episode_count", minimum=1)
    validation_count = _integer(
        validation.get("episode_count"), "validation.episode_count", minimum=1
    )
    if (train_count, validation_count) != (45, 5):
        raise ActContractError("dataset_manifest: formal split must be exactly 45/5")
    if training.get("stats_source") != "train/meta/stats.json":
        raise ActContractError("dataset_manifest: training statistics source differs")
    processors = _mapping(document.get("processors"), "dataset_manifest.processors")
    if processors.get("training_stats_only") is not True:
        raise ActContractError("dataset_manifest: processors must use training-only statistics")

    def indices(split: dict[str, Any], label: str) -> tuple[int, ...]:
        episodes = split.get("episodes")
        count = _integer(
            split.get("episode_count"), f"dataset_manifest.{label}.episode_count", minimum=1
        )
        if not isinstance(episodes, list) or len(episodes) != count:
            raise ActContractError(f"dataset_manifest.{label}.episodes: count differs")
        result = []
        for index, episode in enumerate(episodes):
            location = f"dataset_manifest.{label}.episodes[{index}]"
            record = _mapping(episode, location)
            facts = _mapping(record.get("source_facts"), f"{location}.source_facts")
            task_facts = _mapping(facts.get("task"), f"{location}.task")
            runtime_facts = _mapping(facts.get("runtime"), f"{location}.runtime")
            if task_facts.get("outcome") != "succeeded":
                raise ActContractError(f"{location}: task outcome is not succeeded")
            if task_facts.get("reason") != "TASK_SUCCEEDED":
                raise ActContractError(f"{location}: task reason is not TASK_SUCCEEDED")
            if runtime_facts.get("outcome") not in {"stopped", "completed"}:
                raise ActContractError(f"{location}: runtime is not a successful stop")
            if runtime_facts.get("reason") != "TASK_SUCCEEDED":
                raise ActContractError(f"{location}: runtime reason is not TASK_SUCCEEDED")
            recording = _mapping(facts.get("recording"), f"{location}.recording")
            if recording.get("complete") is not True:
                raise ActContractError(f"{location}: recording is not complete")
            if recording.get("failure_reason") not in {"", None}:
                raise ActContractError(f"{location}: recording has a failure reason")
            if facts.get("quality_qualified") is not True:
                raise ActContractError(f"{location}: quality is not qualified")
            declared = _digest(facts.get("facts_sha256"), f"{location}.facts_sha256")
            payload = {key: value for key, value in facts.items() if key != "facts_sha256"}
            if _canonical_hash(payload) != declared:
                raise ActContractError(f"{location}: source facts digest differs")
            result.append(
                _integer(
                    record.get("global_success_index"),
                    f"{location}.global_success_index",
                )
            )
        if len(set(result)) != len(result):
            raise ActContractError(f"dataset_manifest.{label}.episodes: duplicate index")
        return tuple(sorted(result))

    train_indices = indices(training, "training")
    validation_indices = indices(validation, "validation")
    if set(train_indices) & set(validation_indices):
        raise ActContractError("dataset_manifest: training/validation indices overlap")
    if sorted((*train_indices, *validation_indices)) != list(range(50)):
        raise ActContractError("dataset_manifest: formal indices must cover exactly 0..49")
    if manifest.get("split") != "45train/5validation":
        raise ActContractError("bundle manifest dataset split must be 45train/5validation")
    if manifest.get("train_episodes") != 45 or manifest.get("validation_episodes") != 5:
        raise ActContractError("bundle manifest dataset counts must be 45/5")
    if sorted(manifest.get("training_global_indices", [])) != list(train_indices):
        raise ActContractError("bundle manifest training global indices differ")
    if sorted(manifest.get("validation_global_indices", [])) != list(validation_indices):
        raise ActContractError("bundle manifest validation global indices differ")
    return train_count, validation_count, train_indices, validation_indices, synthetic


def resolve_act_contract(
    *,
    bundle: Path,
    profile_path: Path,
    observation_schema_path: Path,
    action_schema_path: Path,
    require_formal: bool = True,
) -> ActContract:
    """Verify a 035 bundle and the exact profile/schema files it will serve."""
    root = Path(bundle).resolve()
    if not root.is_dir():
        raise ActContractError(f"{root}: bundle directory is missing")
    manifest = _read_json(root / "manifest.json")
    if manifest.get("format") != BUNDLE_FORMAT:
        raise ActContractError("bundle manifest format is not a deployable ACT bundle")
    if manifest.get("policy") != "act":
        raise ActContractError("bundle policy must be act")
    stored_digest = _digest(manifest.get("bundle_digest"), "manifest.bundle_digest")
    payload = {key: value for key, value in manifest.items() if key != "bundle_digest"}
    if stored_digest != _canonical_hash(payload):
        raise ActContractError("bundle manifest digest does not match its contents")
    declared = _mapping(manifest.get("files"), "manifest.files")
    actual = _tree_hashes(root)
    actual.pop("manifest.json", None)
    if actual != declared:
        missing = sorted(set(declared) - set(actual))
        extra = sorted(set(actual) - set(declared))
        changed = sorted(
            name for name in set(actual) & set(declared) if actual[name] != declared[name]
        )
        raise ActContractError(
            "bundle file hash mismatch: "
            f"missing={missing[:4]} extra={extra[:4]} changed={changed[:4]}"
        )
    if any(name.endswith((".pt", ".pth", ".pkl", ".pickle")) for name in declared):
        raise ActContractError("bundle contains a pickle-based model artefact")
    if _integer(manifest.get("chunk_size"), "manifest.chunk_size") != ACT_CHUNK_SIZE:
        raise ActContractError("bundle chunk_size must be 60")
    if _integer(manifest.get("state_dim"), "manifest.state_dim") != ACT_STATE_DIM:
        raise ActContractError("bundle state_dim must be 16")
    model = _mapping(manifest.get("model"), "manifest.model")
    model_sha = _digest(model.get("sha256"), "manifest.model.sha256")
    state_sha = _digest(manifest.get("state_dict_sha256"), "manifest.state_dict_sha256")
    if model_sha != state_sha:
        raise ActContractError("bundle model/state_dict hashes disagree")
    if actual.get("model.safetensors", {}).get("sha256") != model_sha:
        raise ActContractError("bundle model.safetensors hash differs")
    config = _mapping(manifest.get("config"), "manifest.config")
    config_sha = _digest(config.get("sha256"), "manifest.config.sha256")
    if (
        _safe_relative(root, config.get("path"), "manifest.config.path")
        != (root / "config.json").resolve()
    ):
        raise ActContractError("manifest.config.path must be config.json")
    if actual.get("config.json", {}).get("sha256") != config_sha:
        raise ActContractError("bundle config.json hash differs")
    checkpoint = _mapping(manifest.get("checkpoint"), "manifest.checkpoint")
    training_sha = _digest(
        _mapping(manifest.get("source"), "manifest.source").get("training_config_sha256"),
        "manifest.source.training_config_sha256",
    )
    if (
        checkpoint.get("config_sha256") != training_sha
        or actual.get("metadata/training_config.yaml", {}).get("sha256") != training_sha
    ):
        raise ActContractError("checkpoint training configuration hash differs from the bundle")
    checkpoint_mode = _text(checkpoint.get("mode"), "manifest.checkpoint.mode")
    if require_formal and checkpoint_mode != "full":
        raise ActContractError("formal ACT deployment requires a full-mode checkpoint")
    checkpoint_step = _integer(checkpoint.get("step"), "manifest.checkpoint.step", minimum=1)
    dataset = _mapping(manifest.get("dataset"), "manifest.dataset")
    dataset_seed = _integer(dataset.get("seed"), "manifest.dataset.seed")
    if bool(manifest.get("synthetic")) or bool(dataset.get("synthetic")):
        if require_formal:
            raise ActContractError("synthetic bundle evidence cannot deploy a formal ACT model")
    deployment = _mapping(manifest.get("deployment"), "manifest.deployment")
    if (
        deployment.get("format") != "safetensors_only"
        or deployment.get("api") != "predict_action_chunk"
        or deployment.get("fresh_chunk_each_call") is not True
        or deployment.get("select_action_queue") is not False
        or deployment.get("crop") is not None
        or deployment.get("privileged_inputs") != []
    ):
        raise ActContractError("bundle deployment contract is not the accepted ACT contract")

    profile_declared = _mapping(manifest.get("profile"), "manifest.profile")
    bundle_profile_path = _safe_relative(
        root, profile_declared.get("path"), "manifest.profile.path"
    )
    if bundle_profile_path != (root / "metadata/profile.json").resolve():
        raise ActContractError("manifest.profile.path must be metadata/profile.json")
    bundle_profile = _read_json(bundle_profile_path)
    profile_semantic = _canonical_hash(bundle_profile)
    if profile_declared.get("semantic_sha256") != profile_semantic:
        raise ActContractError("bundle profile semantic hash differs")
    if actual.get("metadata/profile.json", {}).get("sha256") != (profile_declared.get("sha256")):
        raise ActContractError("bundle profile file hash differs")
    external_profile = _read_json(Path(profile_path))
    if _canonical_hash(external_profile) != profile_semantic:
        raise ActContractError("shipped profile differs from the trained bundle profile")
    joint_names, lower, upper, camera_names, width, height = _profile_contract(
        bundle_profile, external_profile, "profile"
    )
    joint_units = tuple(str(joint.get("unit")) for joint in bundle_profile["joints"])

    observation_document = _read_json(Path(observation_schema_path))
    action_document = _read_json(Path(action_schema_path))
    observation_hash = _canonical_hash(observation_document)
    action_hash = _canonical_hash(action_document)
    _schema_contract(
        observation_document,
        "observation_schema",
        joint_names=joint_names,
        camera_names=camera_names,
    )
    _schema_contract(
        action_document,
        "action_schema",
        joint_names=joint_names,
        camera_names=camera_names,
    )
    if action_document.get("command_space") != "joint_position":
        raise ActContractError("action schema command_space must be joint_position")
    if observation_document.get("max_skew_ns", 0) < 0:
        raise ActContractError("observation schema max_skew_ns must be non-negative")

    observation = _mapping(manifest.get("observation"), "manifest.observation")
    action = _mapping(manifest.get("action"), "manifest.action")
    if _integer(observation.get("state_dim"), "manifest.observation.state_dim") != ACT_STATE_DIM:
        raise ActContractError("bundle observation state_dim must be 16")
    if observation.get("state_names") != list(joint_names):
        raise ActContractError("bundle observation state names differ")
    if observation.get("state_units") != list(joint_units):
        raise ActContractError("bundle observation state units differ")
    if observation.get("cameras") != list(camera_names):
        raise ActContractError("bundle observation cameras differ")
    if (observation.get("image_height"), observation.get("image_width")) != (height, width):
        raise ActContractError("bundle observation image dimensions differ")
    if observation.get("channels") != 3:
        raise ActContractError("bundle observation channels must be 3")
    if _integer(action.get("dim"), "manifest.action.dim") != ACT_STATE_DIM:
        raise ActContractError("bundle action dim must be 16")
    if action.get("names") != list(joint_names):
        raise ActContractError("bundle action names differ")
    if action.get("units") != list(joint_units):
        raise ActContractError("bundle action units differ")
    if _integer(action.get("chunk_size"), "manifest.action.chunk_size") != ACT_CHUNK_SIZE:
        raise ActContractError("bundle action chunk_size must be 60")
    if (
        _integer(action.get("action_lead_ns"), "manifest.action.action_lead_ns")
        != ACT_ACTION_PERIOD_NS
    ):
        raise ActContractError("bundle action lead must be 50 ms")
    if _integer(action.get("fps"), "manifest.action.fps") != ACT_FPS:
        raise ActContractError("bundle action fps must be 20")
    schema_path = root / "metadata/schema.json"
    schema = _read_json(schema_path)
    if schema.get("observation") != observation or schema.get("action") != action:
        raise ActContractError("metadata/schema.json differs from the bundle manifest")

    processors = _mapping(manifest.get("processors"), "manifest.processors")
    processor_digest = _digest(processors.get("digest"), "manifest.processors.digest")
    if _processor_digest(actual) != processor_digest:
        raise ActContractError("bundle processor digest differs from the shipped files")
    processors_root = root / "processors"
    if not (processors_root / "policy_preprocessor.json").is_file():
        raise ActContractError("bundle is missing the training preprocessor")
    if not (processors_root / "policy_postprocessor.json").is_file():
        raise ActContractError("bundle is missing the training postprocessor")
    processor_stats = _read_json(processors_root / "train_stats_summary.json")
    metadata_stats = _read_json(root / "metadata/train_stats_summary.json")
    if _canonical_hash(processor_stats) != _canonical_hash(metadata_stats):
        raise ActContractError("metadata and processor training statistics differ")
    if dataset.get("stats_source") != "processors/train_stats_summary.json":
        raise ActContractError("bundle dataset stats_source differs")
    stats_sha = _stats_contract(processor_stats, "train_stats_summary")

    dataset_manifest_path = _safe_relative(
        root, dataset.get("manifest"), "manifest.dataset.manifest"
    )
    if dataset_manifest_path != (root / "metadata/dataset_manifest.json").resolve():
        raise ActContractError("manifest.dataset.manifest must be metadata/dataset_manifest.json")
    if actual.get("metadata/dataset_manifest.json", {}).get("sha256") != (
        dataset.get("manifest_sha256")
    ):
        raise ActContractError("bundle dataset manifest file hash differs")
    dataset_document = _read_json(dataset_manifest_path)
    source = _mapping(manifest.get("source"), "manifest.source")
    dataset_lerobot = _mapping(dataset_document.get("lerobot"), "dataset_manifest.lerobot")
    if dataset_lerobot.get("source_revision") != source.get("lerobot_revision"):
        raise ActContractError("dataset manifest source revision differs from the bundle")
    if dataset_lerobot.get("source_archive_sha256") != source.get("lerobot_archive_sha256"):
        raise ActContractError("dataset manifest source archive differs from the bundle")
    train_count, validation_count, train_indices, validation_indices, synthetic = _dataset_contract(
        dataset,
        dataset_document,
        profile_semantic_sha256=profile_semantic,
        require_formal=require_formal,
    )

    act_config = _read_json(root / "config.json")
    if act_config.get("type") not in (None, "act"):
        raise ActContractError("config.json is not an ACT policy configuration")
    if _integer(act_config.get("chunk_size"), "config.chunk_size") != ACT_CHUNK_SIZE:
        raise ActContractError("ACT config chunk_size must be 60")
    if act_config.get("n_obs_steps", 1) != 1:
        raise ActContractError("ACT config n_obs_steps must be 1")
    if act_config.get("temporal_ensemble_coeff") is not None:
        raise ActContractError("ACT config must disable temporal ensembling")
    inputs = _mapping(act_config.get("input_features"), "config.input_features")
    state_feature = _mapping(inputs.get("observation.state"), "config.input_features.state")
    if state_feature.get("shape") != [ACT_STATE_DIM]:
        raise ActContractError("ACT config state shape must be [16]")
    for camera in camera_names:
        name = f"observation.images.{camera}"
        feature = _mapping(inputs.get(name), f"config.input_features.{camera}")
        if feature.get("shape") != [3, height, width]:
            raise ActContractError(f"ACT config camera {camera} shape differs")
    outputs = _mapping(act_config.get("output_features"), "config.output_features")
    action_feature = _mapping(outputs.get("action"), "config.output_features.action")
    if action_feature.get("shape") != [ACT_STATE_DIM]:
        raise ActContractError("ACT config action shape must be [16]")
    _text(source.get("lerobot_revision"), "manifest.source.lerobot_revision")
    _digest(source.get("lerobot_archive_sha256"), "manifest.source.lerobot_archive_sha256")
    _digest(source.get("training_config_sha256"), "manifest.source.training_config_sha256")

    return ActContract(
        bundle_root=root,
        bundle_digest=stored_digest,
        model_sha256=model_sha,
        state_dict_sha256=state_sha,
        config_sha256=config_sha,
        processor_digest=processor_digest,
        stats_sha256=stats_sha,
        profile_file_sha256=_file_sha256(Path(profile_path)),
        profile_semantic_sha256=profile_semantic,
        observation_schema_hash=observation_hash,
        action_schema_hash=action_hash,
        joint_names=joint_names,
        joint_lower_bounds=lower,
        joint_upper_bounds=upper,
        camera_names=camera_names,
        camera_width=width,
        camera_height=height,
        chunk_size=ACT_CHUNK_SIZE,
        state_dim=ACT_STATE_DIM,
        action_period_ns=ACT_ACTION_PERIOD_NS,
        action_lead_ns=ACT_ACTION_PERIOD_NS,
        fps=ACT_FPS,
        checkpoint_step=checkpoint_step,
        checkpoint_mode=checkpoint_mode,
        dataset_seed=dataset_seed,
        train_episodes=train_count,
        validation_episodes=validation_count,
        train_global_indices=train_indices,
        validation_global_indices=validation_indices,
        synthetic=synthetic,
        profile=bundle_profile,
        observation=observation,
        action=action,
        lerobot_revision=str(source.get("lerobot_revision")),
        lerobot_archive_sha256=str(source.get("lerobot_archive_sha256")),
        training_config_sha256=str(source.get("training_config_sha256")),
    )


# --------------------------------------------------------------------- settings
_SETTINGS_FIELDS = {"schema_version", "model", "service", "logging"}
_MODEL_FIELDS = {
    "bundle",
    "profile",
    "observation_schema",
    "action_schema",
    "device",
    "require_formal",
    "gripper_max_correction_m",
    "temporal_ensemble_coeff",
}
_SERVICE_FIELDS = {
    "bind_address",
    "port",
    "max_workers",
    "max_message_bytes",
    "max_sessions",
    "max_open_requests",
    "max_request_cache",
    "predict_deadline_s",
    "open_deadline_s",
    "close_deadline_s",
    "test_only",
    "protocol_major",
    "stateful",
    "task_id",
    "task_text",
}


def _unknown(value: dict[str, Any], known: set[str], location: str) -> None:
    unknown = sorted(set(value) - known)
    if unknown:
        raise ActContractError(f"{location}: unknown field(s): {', '.join(unknown)}")


def _required(value: dict[str, Any], name: str, location: str) -> Any:
    if name not in value:
        raise ActContractError(f"{location}.{name}: field is required")
    return value[name]


def load_act_settings(path: str | Path) -> ActSettings:
    """Parse the small deployment YAML; all model identities are still derived."""
    source = Path(path)
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ActContractError(f"{source}: cannot read: {error}") from error
    except yaml.YAMLError as error:
        raise ActContractError(f"{source}: invalid YAML: {error}") from error
    root = _mapping(document, "<act config>")
    _reject_non_finite(root, "<act config>")
    _unknown(root, _SETTINGS_FIELDS, "<act config>")
    if _integer(_required(root, "schema_version", "<act config>"), "schema_version") != 1:
        raise ActContractError("schema_version: must be 1")
    model = _mapping(_required(root, "model", "<act config>"), "model")
    _unknown(model, _MODEL_FIELDS, "model")
    service = _mapping(_required(root, "service", "<act config>"), "service")
    _unknown(service, _SERVICE_FIELDS, "service")
    base = source.parent

    def model_path(name: str) -> Path:
        raw = Path(_text(_required(model, name, "model"), f"model.{name}"))
        return (base / raw).resolve() if not raw.is_absolute() else raw.resolve()

    device = _text(model.get("device", "cuda"), "model.device")
    if not (device == "cpu" or device == "cuda" or device.startswith("cuda:")):
        raise ActContractError("model.device: must be cpu, cuda or cuda:N")
    require_formal = _boolean(model.get("require_formal", True), "model.require_formal")
    gripper_limit = _number(
        model.get("gripper_max_correction_m", 0.0), "model.gripper_max_correction_m"
    )
    if not 0.0 <= gripper_limit <= 0.03:
        raise ActContractError("model.gripper_max_correction_m: must be in [0, 0.03] m")
    temporal_coeff = model.get("temporal_ensemble_coeff")
    if temporal_coeff is not None:
        temporal_coeff = _number(temporal_coeff, "model.temporal_ensemble_coeff")
        if not 0.0 <= temporal_coeff <= 1.0:
            raise ActContractError("model.temporal_ensemble_coeff: must be in [0, 1]")
    logging = _mapping(root.get("logging", {}), "logging")
    _reject_non_finite(logging, "logging")

    def number(name: str) -> float:
        value = _number(_required(service, name, "service"), f"service.{name}")
        if value <= 0.0:
            raise ActContractError(f"service.{name}: must be positive")
        return value

    settings = ActSettings(
        schema_version=1,
        bundle=model_path("bundle"),
        profile=model_path("profile"),
        observation_schema=model_path("observation_schema"),
        action_schema=model_path("action_schema"),
        device=device,
        require_formal=require_formal,
        bind_address=_text(_required(service, "bind_address", "service"), "service.bind_address"),
        port=_integer(_required(service, "port", "service"), "service.port"),
        max_workers=_integer(_required(service, "max_workers", "service"), "service.max_workers"),
        max_message_bytes=_integer(
            _required(service, "max_message_bytes", "service"), "service.max_message_bytes"
        ),
        max_sessions=_integer(
            _required(service, "max_sessions", "service"), "service.max_sessions"
        ),
        max_open_requests=_integer(
            _required(service, "max_open_requests", "service"), "service.max_open_requests"
        ),
        max_request_cache=_integer(
            _required(service, "max_request_cache", "service"), "service.max_request_cache"
        ),
        predict_deadline_s=number("predict_deadline_s"),
        open_deadline_s=number("open_deadline_s"),
        close_deadline_s=number("close_deadline_s"),
        test_only=_boolean(_required(service, "test_only", "service"), "service.test_only"),
        protocol_major=_integer(
            _required(service, "protocol_major", "service"), "service.protocol_major"
        ),
        stateful=_boolean(_required(service, "stateful", "service"), "service.stateful"),
        task_id=_text(_required(service, "task_id", "service"), "service.task_id"),
        task_text=_text(_required(service, "task_text", "service"), "service.task_text"),
        logging=dict(logging),
        gripper_max_correction_m=gripper_limit,
        temporal_ensemble_coeff=temporal_coeff,
    )
    if settings.test_only:
        raise ActContractError("service.test_only: the ACT deployment cannot be a test-only policy")
    if settings.protocol_major != 1:
        raise ActContractError("service.protocol_major: must be 1")
    if settings.stateful is not True:
        raise ActContractError("service.stateful: the ACT runtime requires a stateful session")
    if not 0 <= settings.port <= 65535:
        raise ActContractError("service.port: must be within [0, 65535]")
    if settings.max_message_bytes < 1024:
        raise ActContractError("service.max_message_bytes: must be at least 1024")
    if not settings.max_workers or settings.max_workers > 256:
        raise ActContractError("service.max_workers: must be within [1, 256]")
    if settings.max_sessions < 1:
        raise ActContractError("service.max_sessions: must be positive")
    if settings.max_open_requests < 1 or settings.max_request_cache < 1:
        raise ActContractError("service request caches must be positive")
    return settings


__all__ = [
    "ACT_ACTION_PERIOD_NS",
    "ACT_CAMERAS",
    "ACT_CHUNK_SIZE",
    "ACT_STATE_DIM",
    "ActContract",
    "ActContractError",
    "ActSettings",
    "load_act_settings",
    "resolve_act_contract",
]
