"""Single strict Policy service configuration and deployment contract."""

from __future__ import annotations

import json
import re
from dataclasses import asdict
from pathlib import Path
from typing import Annotated, Any, ClassVar

from pydantic import BeforeValidator, Field, field_validator, model_validator
from robot_contracts import validation as rv
from robot_contracts.io import canonical_sha256 as _canonical
from robot_contracts.io import sha256_file
from robot_contracts.models import ContractError, RobotProfile
from robot_run_logging import LogSettings

from ..config import ConfigError, load_yaml_mapping

CHUNK_SIZE, PERIOD_NS, FPS = 60, 50_000_000, 20
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_MODEL_KEYS = (
    "model_path", "robot_profile_path", "observation_schema_path", "action_schema_path",
    "device", "require_formal", "max_prediction_corrections", "temporal_ensemble_coeff",
)


class PolicyConfigError(rv.StrictMappingError):
    """One actionable configuration or contract rejection."""


OptionalSha = Annotated[str, Field(strict=True, pattern=r"^(?:[0-9a-f]{64})?$")]
FloatVector = Annotated[
    tuple[rv.StrictFloat, ...], BeforeValidator(rv.as_tuple), Field(min_length=1)
]
NameTuple = Annotated[
    tuple[rv.SafeIdStr, ...], BeforeValidator(rv.as_tuple), Field(min_length=1)
]


class PolicyServiceConfig(rv.StrictMappingModel):
    _config_error: ClassVar[type[ValueError]] = PolicyConfigError

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
    stateful: bool
    model_bundle_digest: OptionalSha = ""
    processor_digest: OptionalSha = ""
    observation_schema_hash: OptionalSha = ""
    action_schema_hash: OptionalSha = ""
    robot_profile_hash: OptionalSha = ""
    joint_names: NameTuple
    joint_lower_bounds: FloatVector
    joint_upper_bounds: FloatVector
    action_joint_names: NameTuple
    camera_names: NameTuple
    camera_width: Annotated[rv.PositiveInt, Field(le=4096)]
    camera_height: Annotated[rv.PositiveInt, Field(le=4096)]
    action_period_ns: Annotated[rv.PositiveInt, Field(le=10_000_000_000)]
    chunk_size: Annotated[rv.PositiveInt, Field(le=4096)]
    task_id: rv.NonEmptyStr
    task_text: rv.NonEmptyStr
    command_space: rv.NonEmptyStr
    logging: LogSettings = Field(default_factory=LogSettings)
    model_path: Path | None = None
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
            raise rv.contract_error("chunk_size", f"must contain exactly {CHUNK_SIZE} targets")
        if self.action_period_ns != PERIOD_NS:
            raise rv.contract_error("action_period_ns", "must use a uniform 50 ms period")
        state_dim = len(self.joint_names)
        if len(self.joint_lower_bounds) != state_dim or len(self.joint_upper_bounds) != state_dim:
            raise rv.contract_error("joint_lower_bounds", "must match joint_names length")
        if self.action_joint_names != self.joint_names:
            raise rv.contract_error("action_joint_names", "must match the complete joint order")
        if self.command_space != "joint_position":
            raise rv.contract_error("command_space", "must be joint_position")
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


def load_policy_config(path: str | Path, **overrides: object) -> PolicyServiceConfig:
    try:
        document = load_yaml_mapping(path)
    except ConfigError as error:
        raise PolicyConfigError(error.source, error.field, error.message) from error
    service, model = document.get("service"), document.get("model", {})
    if not isinstance(service, dict) or not isinstance(model, dict):
        raise PolicyConfigError(str(path), "service/model", "must be mappings")
    values = {key: model[key] for key in _MODEL_KEYS if key in model} | dict(service)
    values |= {"logging": document.get("logging", {}), "command_space": "joint_position"}
    values.update({key: value for key, value in overrides.items() if value is not None})
    base = Path(path).resolve().parent
    for key in (name for name in _MODEL_KEYS if name == "model_path" or name.endswith("path")):
        if values.get(key) is not None:
            values[key] = Path(base, values[key]).resolve()
    if values.get("model_path") is not None:
        if values.get("robot_profile_path") is None or values.get("test_only"):
            raise PolicyConfigError(str(path), "robot_profile_path", "required")
        if not values.get("stateful"):
            raise PolicyConfigError(str(path), "stateful", "stateful session required")
        values.update(resolve_policy_contract(values))
    return rv.validate_config_model(PolicyServiceConfig, values, str(path), PolicyConfigError)


def resolve_policy_contract(values: dict[str, Any]) -> dict[str, Any]:
    from ..training.bundle import ModelError, read_model_metadata, resolve_model_path

    formal = bool(values.get("require_formal", True))
    try:
        model_file = resolve_model_path(values["model_path"])
        model_file, metadata = read_model_metadata(model_file)
    except ModelError as error:
        raise PolicyConfigError("<model>", str(error)) from error
    _check(not (formal and metadata.get("synthetic") == "true"),
           "synthetic model cannot deploy a formal ACT policy")
    try:
        act_config = json.loads(metadata["act_config_json"])
        observation_schema = json.loads(metadata["observation_schema_json"])
        action_schema = json.loads(metadata["action_schema_json"])
    except (KeyError, json.JSONDecodeError) as error:
        raise PolicyConfigError("<model>", f"model metadata is incomplete: {error}") from error

    profile_document = _read_json(Path(values["robot_profile_path"]))
    semantic = _canonical(profile_document)
    _check(profile_document.get("test_only") is False, "profile must be a formal RobotProfile")
    try:
        profile = RobotProfile.validate_dict(profile_document)
    except ContractError as error:
        raise PolicyConfigError("<profile>", str(error)) from error
    names = profile.joint_names
    cameras = profile.camera_names
    state_dim = len(names)
    _check(bool(profile.cameras), "profile must declare at least one camera")
    width, height = profile.cameras[0].width, profile.cameras[0].height
    _check(
        all((camera.width, camera.height) == (width, height) for camera in profile.cameras),
        "profile cameras must share identical RGB dimensions",
    )
    chunk_size = int(action_schema.get("chunk_size", CHUNK_SIZE))
    period_ns = int(action_schema.get("action_lead_ns", PERIOD_NS))
    _check(
        act_config.get("chunk_size") == chunk_size
        and act_config.get("n_action_steps") == chunk_size
        and act_config.get("temporal_ensemble_coeff") is None,
        f"ACT config must use chunk_size {chunk_size} and disable internal temporal ensembling",
    )
    _check(
        observation_schema.get("state_dim") == state_dim
        and tuple(observation_schema.get("cameras", ())) == cameras
        and (observation_schema.get("image_width"), observation_schema.get("image_height"))
        == (width, height),
        "model observation schema differs from the RobotProfile joints/cameras/resolution",
    )
    _check(
        action_schema.get("dim") == state_dim and action_schema.get("chunk_size") == chunk_size,
        "model action schema differs from the RobotProfile joint count or chunk size",
    )
    observation = _read_json(Path(values["observation_schema_path"]))
    action = _read_json(Path(values["action_schema_path"]))
    _check(
        tuple(action.get("joint_names", ())) == names
        and action.get("command_space") == "joint_position",
        "action_schema: joint order/command space differs",
    )
    _check(
        tuple(observation.get("joint_names", ())) == names,
        "observation_schema: joint order differs",
    )

    model_digest = sha256_file(model_file)
    processor_digest = _canonical(
        {
            "preprocessor": json.loads(metadata["preprocessor_config_json"]),
            "postprocessor": json.loads(metadata["postprocessor_config_json"]),
        }
    )
    return {
        "model_bundle_digest": _digest(model_digest, "model.safetensors"),
        "processor_digest": _digest(processor_digest, "processors"),
        "observation_schema_hash": _canonical(observation),
        "action_schema_hash": _canonical(action),
        "robot_profile_hash": semantic,
        "joint_names": names,
        "joint_lower_bounds": tuple(joint.minimum for joint in profile.joints),
        "joint_upper_bounds": tuple(joint.maximum for joint in profile.joints),
        "action_joint_names": names,
        "camera_names": cameras,
        "camera_width": width,
        "camera_height": height,
        "action_period_ns": period_ns,
        "chunk_size": chunk_size,
    }


def _check(condition: object, message: str) -> None:
    if not condition:
        raise PolicyConfigError("<model>", message)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PolicyConfigError(str(path), "cannot read JSON", str(error)) from error
    if not isinstance(value, dict):
        raise PolicyConfigError(str(path), "expected a JSON object")
    return value


def _digest(value: object, location: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise PolicyConfigError("<model>", f"{location}: must be a SHA-256 digest")
    return value
