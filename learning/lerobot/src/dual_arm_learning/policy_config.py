"""Single strict Policy service configuration and deployment contract."""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import asdict
from pathlib import Path
from typing import Annotated, Any, ClassVar

from pydantic import BeforeValidator, Field, field_validator, model_validator
from robot_contracts import validation as rv
from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.overrides import apply_overrides
from robot_run_logging import LogSettings

from .config_yaml import load_yaml

BUNDLE_FORMAT = "dual-arm-act-bundle-v1"
CAMERAS = ("global", "left_wrist", "right_wrist")
STATE_DIM, CHUNK_SIZE, PERIOD_NS, FPS = 16, 60, 50_000_000, 20
WIDTH, HEIGHT = 320, 240
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_MODEL_KEYS = (
    "bundle_dir", "robot_profile_path", "observation_schema_path", "action_schema_path",
    "device", "require_formal", "max_prediction_corrections", "temporal_ensemble_coeff",
)


class PolicyConfigError(rv.StrictMappingError):
    """One actionable configuration or contract rejection."""


OptionalSha = Annotated[str, Field(strict=True, pattern=r"^(?:[0-9a-f]{64})?$")]
Vector16 = Annotated[
    tuple[rv.StrictFloat, ...], BeforeValidator(rv.as_tuple), Field(min_length=16, max_length=16)
]


class PolicyServiceConfig(rv.StrictMappingModel):
    _config_error: ClassVar[type[ValueError]] = PolicyConfigError

    schema_version: Annotated[int, Field(strict=True, ge=1, le=1)] = 1
    bind_address: rv.NonEmptyStr
    port: Annotated[int, Field(strict=True, ge=0, le=65535)]
    max_workers: Annotated[rv.PositiveInt, Field(le=256)]
    max_message_bytes: Annotated[rv.PositiveInt, Field(ge=1024, le=256 * 1024 * 1024)]
    max_sessions: rv.PositiveInt
    max_open_requests: rv.PositiveInt
    max_request_cache: rv.PositiveInt
    predict_deadline_s: rv.PositiveFloat
    open_deadline_s: rv.PositiveFloat
    close_deadline_s: rv.PositiveFloat
    test_only: bool
    protocol_major: Annotated[int, Field(strict=True, ge=1, le=1)]
    stateful: bool
    model_bundle_digest: OptionalSha = ""
    processor_digest: OptionalSha = ""
    observation_schema_hash: OptionalSha = ""
    action_schema_hash: OptionalSha = ""
    robot_profile_hash: OptionalSha = ""
    joint_names: Annotated[
        tuple[rv.SafeIdStr, ...], BeforeValidator(rv.as_tuple), Field(min_length=16, max_length=16)
    ]
    joint_lower_bounds: Vector16
    joint_upper_bounds: Vector16
    action_joint_names: Annotated[
        tuple[rv.SafeIdStr, ...], BeforeValidator(rv.as_tuple), Field(min_length=16, max_length=16)
    ]
    camera_names: Annotated[
        tuple[rv.SafeIdStr, ...], BeforeValidator(rv.as_tuple), Field(min_length=3, max_length=3)
    ]
    camera_width: Annotated[rv.PositiveInt, Field(le=4096)]
    camera_height: Annotated[rv.PositiveInt, Field(le=4096)]
    action_period_ns: Annotated[rv.PositiveInt, Field(le=10_000_000_000)]
    chunk_size: Annotated[rv.PositiveInt, Field(le=4096)]
    task_id: rv.NonEmptyStr
    task_text: rv.NonEmptyStr
    command_space: rv.NonEmptyStr
    logging: LogSettings = Field(default_factory=LogSettings)
    bundle_dir: Path | None = None
    robot_profile_path: Path | None = None
    observation_schema_path: Path | None = None
    action_schema_path: Path | None = None
    device: rv.NonEmptyStr = "cpu"
    require_formal: bool = True
    max_prediction_corrections: dict[str, rv.NonNegativeFloat] = Field(default_factory=dict)
    temporal_ensemble_coeff: rv.StrictFloat | None = None

    @field_validator("logging", mode="before")
    @classmethod
    def _log_settings(cls, value: object) -> object:
        return value if isinstance(value, LogSettings) else LogSettings(**(value or {}))

    @model_validator(mode="after")
    def _service_contract(self) -> PolicyServiceConfig:
        if self.chunk_size != CHUNK_SIZE:
            raise rv.contract_error("chunk_size", "v1 requires exactly 60 targets")
        if self.action_period_ns != PERIOD_NS:
            raise rv.contract_error("action_period_ns", "v1 requires a uniform 50 ms period")
        if self.action_joint_names != self.joint_names:
            raise rv.contract_error("action_joint_names", "v1 requires the complete 16-joint order")
        if self.command_space != "joint_position":
            raise rv.contract_error("command_space", "v1 accepts only joint_position")
        unknown = set(self.max_prediction_corrections) - set(self.joint_names)
        if unknown:
            raise rv.contract_error(
                "max_prediction_corrections", f"unknown axes: {sorted(unknown)}"
            )
        if self.temporal_ensemble_coeff is not None and not (
            0.0 <= self.temporal_ensemble_coeff <= 1.0
        ):
            raise rv.contract_error("temporal_ensemble_coeff", "must be in [0, 1]")
        return self

    @property
    def chunk_duration_ns(self) -> int:
        return self.chunk_size * self.action_period_ns

    def to_document(self) -> dict[str, Any]:
        document = self.model_dump(mode="json", exclude={"logging"})
        document["logging"] = asdict(self.logging)
        return document


def parse_policy_config(document: object) -> PolicyServiceConfig:
    return rv.validate_config_model(PolicyServiceConfig, document, "<config>", PolicyConfigError)


def load_policy_config(
    path: str | Path, *, set_overrides: tuple[str, ...] | list[str] = (), **overrides: object
) -> PolicyServiceConfig:
    document = load_yaml(path)
    if set_overrides:
        document = apply_overrides(document, list(set_overrides))
    if not isinstance(document, dict):
        raise PolicyConfigError(str(path), "<root>", "must be a mapping")
    service, model = document.get("service"), document.get("model", {})
    if not isinstance(service, dict) or not isinstance(model, dict):
        raise PolicyConfigError(str(path), "service/model", "must be mappings")
    values = {key: model[key] for key in _MODEL_KEYS if key in model} | dict(service)
    values |= {"logging": document.get("logging", {}), "command_space": "joint_position"}
    values.update({key: value for key, value in overrides.items() if value is not None})
    base = Path(path).resolve().parent
    for key in (name for name in _MODEL_KEYS if name == "bundle_dir" or name.endswith("path")):
        if values.get(key) is not None:
            values[key] = Path(base, values[key]).resolve()
    if values.get("bundle_dir") is not None:
        if values.get("robot_profile_path") is None or values.get("test_only"):
            raise PolicyConfigError(str(path), "robot_profile_path", "required")
        if not values.get("stateful"):
            raise PolicyConfigError(str(path), "stateful", "stateful session required")
        values.update(resolve_policy_contract(values))
    return rv.validate_config_model(PolicyServiceConfig, values, str(path), PolicyConfigError)


def resolve_policy_contract(values: dict[str, Any]) -> dict[str, Any]:
    from .bundle import BundleError, verify_bundle

    root = Path(values["bundle_dir"]).resolve()
    formal = bool(values.get("require_formal", True))
    if not root.is_dir():
        raise PolicyConfigError("<bundle>", f"{root}: bundle directory is missing")
    try:
        manifest = verify_bundle(root)
    except BundleError as error:
        raise PolicyConfigError("<bundle>", str(error)) from error
    checkpoint = _mapping(manifest.get("checkpoint"), "manifest.checkpoint")
    _check(not formal or checkpoint.get("mode") == "full",
           "formal ACT deployment requires a full-mode checkpoint")
    dataset = _mapping(manifest.get("dataset"), "manifest.dataset")
    _check(not (formal and (manifest.get("synthetic") or dataset.get("synthetic"))),
           "synthetic bundle cannot deploy a formal model")
    bundle_profile = _read_json(root / "metadata/profile.json")
    semantic = _canonical(bundle_profile)
    _check(_read_json(Path(values["robot_profile_path"])) == bundle_profile,
           "shipped profile differs from the trained bundle profile")
    _check(bundle_profile.get("schema_version") == 2 and bundle_profile.get("test_only") is False,
           "profile must be a formal schema-2 RobotProfile")
    try:
        profile = RobotProfile.validate_dict(bundle_profile)
    except ContractError as error:
        raise PolicyConfigError("<profile>", str(error)) from error
    names = profile.joint_names
    _check(len(names) == STATE_DIM and profile.camera_names == CAMERAS,
           "profile joints/cameras differ")
    _check(all((camera.width, camera.height) == (WIDTH, HEIGHT) for camera in profile.cameras),
           "profile cameras must be 320x240")
    observation = _read_json(Path(values["observation_schema_path"]))
    action = _read_json(Path(values["action_schema_path"]))
    _check(tuple(action.get("joint_names", ())) == names
           and action.get("command_space") == "joint_position",
           "action_schema: joint order/command space differs")
    _check(tuple(observation.get("joint_names", ())) == names,
           "observation_schema: joint order differs")
    stats = _read_json(root / "processors/train_stats_summary.json")
    _check(stats.get("source") == "training_split_only",
           "train_stats_summary.source must prove training-split-only statistics")
    document = _read_json(root / "metadata/dataset_manifest.json")
    _check(document.get("status") == "complete" and document.get("schema_version") == 1,
           "dataset_manifest.json: status must be complete")
    _check(not (formal and document.get("synthetic")),
           "synthetic dataset evidence cannot deploy a formal ACT model")
    splits = document.get("splits", {})
    train_episodes = splits.get("training", {}).get("episode_count")
    validation_episodes = splits.get("validation", {}).get("episode_count")
    _check(dataset.get("train_episodes") == train_episodes
           and dataset.get("validation_episodes") == validation_episodes,
           "bundle dataset counts disagree with dataset_manifest")
    _check(not (formal and document.get("diagnostic")),
           "diagnostic dataset evidence cannot deploy a formal ACT model")
    _check(not formal or (train_episodes, validation_episodes) == (45, 5),
           "dataset_manifest: formal split must be exactly 45/5")
    _check(document.get("processors", {}).get("training_stats_only") is True,
           "dataset_manifest: processors must use training-only statistics")
    act_config = _read_json(root / "config.json")
    _check(act_config.get("chunk_size") == CHUNK_SIZE
           and act_config.get("n_obs_steps", 1) == 1
           and act_config.get("temporal_ensemble_coeff") is None,
           "ACT config must use chunk_size 60 and disable temporal ensembling")
    return {
        "model_bundle_digest": _digest(manifest.get("bundle_digest"), "manifest.bundle_digest"),
        "processor_digest": _digest(
            _mapping(manifest.get("processors"), "manifest.processors").get("digest"),
            "manifest.processors.digest",
        ),
        "observation_schema_hash": _canonical(observation),
        "action_schema_hash": _canonical(action),
        "robot_profile_hash": semantic,
        "joint_names": names,
        "joint_lower_bounds": tuple(joint.minimum for joint in profile.joints),
        "joint_upper_bounds": tuple(joint.maximum for joint in profile.joints),
        "action_joint_names": names,
        "camera_names": profile.camera_names,
        "camera_width": WIDTH,
        "camera_height": HEIGHT,
        "action_period_ns": PERIOD_NS,
        "chunk_size": CHUNK_SIZE,
    }


def _check(condition: object, message: str) -> None:
    if not condition:
        raise PolicyConfigError("<bundle>", message)


def _canonical(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PolicyConfigError(str(path), "cannot read JSON", str(error)) from error
    if not isinstance(value, dict):
        raise PolicyConfigError(str(path), "expected a JSON object")
    return value


def _mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PolicyConfigError("<bundle>", f"{location}: must be an object")
    return value


def _digest(value: object, location: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise PolicyConfigError("<bundle>", f"{location}: must be a SHA-256 digest")
    return value
