"""Validated runtime recipes, distinct from robot semantics and deployment paths."""

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.profiles import (
    load_document,
    load_profile,
    semantic_hash,
    validate_bindings,
    validate_document,
)
from robot_run_logging import LogSettings

from dual_arm_runtime.recording_queue import RecordingSettings

# BackendEngine.acquire/renew currently support this inclusive lease range. The
# runtime rejects unsupported values before an episode starts instead of waiting
# for the first lease request to fail.
MIN_LEASE_DURATION_NS = 100_000_000
MAX_LEASE_DURATION_NS = 10_000_000_000

_SCHEMA_DIRECTORY = "contracts/schemas"


@dataclass(frozen=True)
class ScriptTarget:
    duration_ns: int
    positions: tuple[float, ...]


@dataclass(frozen=True)
class ObservationSettings:
    schema_version: int
    schema_id: str
    joint_names: tuple[str, ...]
    camera_names: tuple[str, ...]
    max_skew_ns: int
    max_age_ns: int


@dataclass(frozen=True)
class ActionSettings:
    schema_version: int
    schema_id: str
    command_space: str
    joint_names: tuple[str, ...]


@dataclass(frozen=True)
class RuntimeRecipe:
    schema_version: int
    robot_profile: str
    observation_schema: str
    action_schema: str
    robot_namespace: str
    lease_duration_ns: int
    renew_interval_ns: int
    command_acceptance_ns: int
    startup_timeout_s: float
    call_timeout_s: float
    command_timeout_s: float
    script_targets: tuple[ScriptTarget, ...]
    logging: LogSettings = field(default_factory=LogSettings)
    recording: RecordingSettings = field(default_factory=RecordingSettings)


@dataclass(frozen=True)
class Configuration:
    recipe: RuntimeRecipe
    profile: RobotProfile
    observation: ObservationSettings
    action: ActionSettings
    schema_root: Path
    # The snapshot is canonical JSON so callers can only obtain fresh, detached
    # dictionaries and cannot mutate runtime configuration through it.
    _snapshot_json: str = field(repr=False)

    @property
    def snapshot(self) -> dict[str, Any]:
        """Return a fresh JSON-compatible audit snapshot."""
        return json.loads(self._snapshot_json)


def _raise_with_source(source: Path | str, error: Exception) -> None:
    detail = str(error)
    if isinstance(error, ContractError):
        prefix = f"{error.reason}: "
        if detail.startswith(prefix):
            detail = detail[len(prefix) :]
    source_text = str(source)
    if source_text and not detail.startswith(f"{source_text}:"):
        detail = f"{source_text}: {detail}"
    raise ContractError("SCHEMA_MISMATCH", detail) from error


def _load(path: Path) -> dict[str, Any]:
    try:
        document = load_document(path)
    except ContractError as error:
        _raise_with_source(path, error)
    except (OSError, yaml.YAMLError) as error:
        _raise_with_source(path, error)
    _reject_non_finite(document, str(path))
    return document


def _validate(document: dict[str, Any], schema_path: Path, source: Path) -> None:
    try:
        validate_document(document, schema_path)
    except (ContractError, OSError) as error:
        _raise_with_source(source, error)


def _reject_non_finite(value: Any, location: str) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite(item, f"{location}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_non_finite(item, f"{location}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise ContractError("SCHEMA_MISMATCH", f"{location} must be a finite number")


def _integer_value(value: Any, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError("SCHEMA_MISMATCH", f"{location} must be an integer")
    return value


def _number_value(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ContractError("SCHEMA_MISMATCH", f"{location} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ContractError("SCHEMA_MISMATCH", f"{location} must be a finite number")
    return result


def _load_profile(path: Path, schemas: Path) -> RobotProfile:
    try:
        return load_profile(path, schemas)
    except (ContractError, OSError) as error:
        _raise_with_source(path, error)


def _validate_bindings(
    profile: RobotProfile,
    observation: dict[str, Any],
    observation_path: Path,
    action: dict[str, Any],
    action_path: Path,
    schemas: Path,
) -> None:
    try:
        validate_bindings(profile, observation, action, schemas)
    except ContractError as error:
        _raise_with_source(
            f"{observation_path}, {action_path} (joint_names/camera_names binding)",
            error,
        )


def _script_targets(
    document: dict[str, Any],
    source: Path,
    profile: RobotProfile,
) -> tuple[ScriptTarget, ...]:
    targets = []
    previous = profile.initial_positions
    for index, raw in enumerate(document["script_targets"]):
        location = f"{source}: script_targets[{index}]"
        duration_ns = _integer_value(raw["duration_ns"], f"{location}.duration_ns")
        raw_positions = raw["positions"]
        if len(raw_positions) != len(profile.joints):
            raise ContractError(
                "SCHEMA_MISMATCH",
                f"{location}.positions: expected {len(profile.joints)} values for profile "
                f"joints, got {len(raw_positions)}",
            )
        positions = []
        for joint_index, (raw_position, joint) in enumerate(
            zip(raw_positions, profile.joints, strict=True)
        ):
            position_location = f"{location}.positions[{joint_index}]"
            position = _number_value(raw_position, position_location)
            if not joint.minimum <= position <= joint.maximum:
                raise ContractError(
                    "SCHEMA_MISMATCH",
                    f"{position_location} ({joint.name}): {position} outside joint limits "
                    f"[{joint.minimum}, {joint.maximum}]",
                )
            positions.append(position)
        seconds = duration_ns / 1e9
        for joint_index, (position, start, joint) in enumerate(
            zip(positions, previous, profile.joints, strict=True)
        ):
            speed = abs(position - start) / seconds
            if speed > joint.max_velocity + 1e-9:
                raise ContractError(
                    "SCHEMA_MISMATCH",
                    f"{location}.positions[{joint_index}] ({joint.name}): average speed "
                    f"{speed:.6g} exceeds max_velocity {joint.max_velocity}",
                )
        targets.append(ScriptTarget(duration_ns, tuple(positions)))
        previous = tuple(positions)
    return tuple(targets)


def _build_logging(document: dict[str, Any], source: Path) -> LogSettings:
    """Build the process logging limits, filling every omitted default explicitly."""
    raw = document.get("logging", {})
    if not isinstance(raw, dict):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: logging must be an object")
    location = f"{source}: logging"
    integer_fields = (
        "queue_capacity",
        "max_event_bytes",
        "max_file_bytes",
        "max_total_bytes",
    )
    number_fields = ("close_timeout_s", "flush_interval_s")
    values: dict[str, Any] = {}
    for name in integer_fields:
        if name in raw:
            values[name] = _integer_value(raw[name], f"{location}.{name}")
    for name in number_fields:
        if name in raw:
            values[name] = _number_value(raw[name], f"{location}.{name}")
    if "console" in raw:
        if not isinstance(raw["console"], bool):
            raise ContractError("SCHEMA_MISMATCH", f"{location}.console must be a boolean")
        values["console"] = raw["console"]
    try:
        return LogSettings(**values)
    except (TypeError, ValueError) as error:
        raise ContractError("SCHEMA_MISMATCH", f"{location}: {error}") from error


def _build_recording(document: dict[str, Any], source: Path) -> RecordingSettings:
    """Build the episode recording limits, filling omitted defaults explicitly."""
    raw = document.get("recording", {})
    if not isinstance(raw, dict):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: recording must be an object")
    location = f"{source}: recording"
    values: dict[str, Any] = {}
    for name in ("queue_capacity", "max_pending_bytes"):
        if name in raw:
            values[name] = _integer_value(raw[name], f"{location}.{name}")
    for name in ("startup_timeout_s", "close_timeout_s"):
        if name in raw:
            values[name] = _number_value(raw[name], f"{location}.{name}")
    try:
        return RecordingSettings(**values)
    except (TypeError, ValueError) as error:
        raise ContractError("SCHEMA_MISMATCH", f"{location}: {error}") from error


def _effective_recipe_document(
    document: dict[str, Any],
    logging_settings: LogSettings,
    recording_settings: RecordingSettings,
) -> dict[str, Any]:
    """Return the recipe with effective logging/recording defaults visible."""
    effective = dict(document)
    effective["logging"] = asdict(logging_settings)
    effective["recording"] = asdict(recording_settings)
    return effective


def _build_recipe(
    document: dict[str, Any],
    source: Path,
    profile: RobotProfile,
) -> RuntimeRecipe:
    lease_duration_ns = _integer_value(
        document["lease_duration_ns"], f"{source}: lease_duration_ns"
    )
    if not MIN_LEASE_DURATION_NS <= lease_duration_ns <= MAX_LEASE_DURATION_NS:
        raise ContractError(
            "SCHEMA_MISMATCH",
            f"{source}: lease_duration_ns ({lease_duration_ns}) must be within the backend "
            f"limit [{MIN_LEASE_DURATION_NS}, {MAX_LEASE_DURATION_NS}] ns",
        )
    renew_interval_ns = _integer_value(
        document["renew_interval_ns"], f"{source}: renew_interval_ns"
    )
    if renew_interval_ns <= 0 or renew_interval_ns * 2 >= lease_duration_ns:
        raise ContractError(
            "SCHEMA_MISMATCH",
            f"{source}: renew_interval_ns ({renew_interval_ns}) must be positive and less "
            f"than half of lease_duration_ns ({lease_duration_ns})",
        )
    command_acceptance_ns = _integer_value(
        document["command_acceptance_ns"], f"{source}: command_acceptance_ns"
    )
    if command_acceptance_ns <= 0:
        raise ContractError(
            "SCHEMA_MISMATCH",
            f"{source}: command_acceptance_ns must be positive",
        )
    timeouts = {}
    for name in ("startup_timeout_s", "call_timeout_s", "command_timeout_s"):
        value = _number_value(document[name], f"{source}: {name}")
        if value <= 0:
            raise ContractError("SCHEMA_MISMATCH", f"{source}: {name} must be positive")
        timeouts[name] = value
    script_targets = _script_targets(document, source, profile)
    logging_settings = _build_logging(document, source)
    recording_settings = _build_recording(document, source)
    longest_segment_s = max(target.duration_ns for target in script_targets) / 1e9
    if timeouts["command_timeout_s"] <= longest_segment_s:
        raise ContractError(
            "SCHEMA_MISMATCH",
            f"{source}: command_timeout_s ({timeouts['command_timeout_s']}) must exceed the "
            f"longest script segment ({longest_segment_s} s)",
        )
    return RuntimeRecipe(
        schema_version=_integer_value(document["schema_version"], f"{source}: schema_version"),
        robot_profile=document["robot_profile"],
        observation_schema=document["observation_schema"],
        action_schema=document["action_schema"],
        robot_namespace=document["robot_namespace"],
        lease_duration_ns=lease_duration_ns,
        renew_interval_ns=renew_interval_ns,
        command_acceptance_ns=command_acceptance_ns,
        startup_timeout_s=timeouts["startup_timeout_s"],
        call_timeout_s=timeouts["call_timeout_s"],
        command_timeout_s=timeouts["command_timeout_s"],
        script_targets=script_targets,
        logging=logging_settings,
        recording=recording_settings,
    )


def _build_observation(document: dict[str, Any], source: Path) -> ObservationSettings:
    max_skew_ns = _integer_value(document["max_skew_ns"], f"{source}: max_skew_ns")
    if max_skew_ns < 0:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: max_skew_ns must not be negative")
    max_age_ns = _integer_value(document["max_age_ns"], f"{source}: max_age_ns")
    if max_age_ns <= 0:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: max_age_ns must be positive")
    return ObservationSettings(
        schema_version=_integer_value(document["schema_version"], f"{source}: schema_version"),
        schema_id=document["schema_id"],
        joint_names=tuple(document["joint_names"]),
        camera_names=tuple(document["camera_names"]),
        max_skew_ns=max_skew_ns,
        max_age_ns=max_age_ns,
    )


def _build_action(document: dict[str, Any], source: Path) -> ActionSettings:
    return ActionSettings(
        schema_version=_integer_value(document["schema_version"], f"{source}: schema_version"),
        schema_id=document["schema_id"],
        command_space=document["command_space"],
        joint_names=tuple(document["joint_names"]),
    )


def _canonical_json(document: dict[str, Any]) -> str:
    return json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)


def load_configuration(root: Path, path: Path) -> Configuration:
    schemas = root / _SCHEMA_DIRECTORY
    recipe_schemas = schemas / "v1"
    recipe_document = _load(path)
    _validate(recipe_document, recipe_schemas / "runtime_config.schema.json", path)
    profile_path = root / recipe_document["robot_profile"]
    profile_document = _load(profile_path)
    profile = _load_profile(profile_path, schemas)
    observation_path = root / recipe_document["observation_schema"]
    observation_document = _load(observation_path)
    _validate(observation_document, recipe_schemas / "observation.schema.json", observation_path)
    action_path = root / recipe_document["action_schema"]
    action_document = _load(action_path)
    _validate(action_document, recipe_schemas / "action.schema.json", action_path)
    _validate_bindings(
        profile,
        observation_document,
        observation_path,
        action_document,
        action_path,
        schemas,
    )
    recipe = _build_recipe(recipe_document, path, profile)
    snapshot_document = {
        "profile": profile_document,
        "profile_hash": profile.digest,
        "observation": observation_document,
        "observation_hash": semantic_hash(observation_document),
        "action": action_document,
        "action_hash": semantic_hash(action_document),
        "recipe": _effective_recipe_document(recipe_document, recipe.logging, recipe.recording),
    }
    return Configuration(
        recipe=recipe,
        profile=profile,
        observation=_build_observation(observation_document, observation_path),
        action=_build_action(action_document, action_path),
        schema_root=schemas,
        _snapshot_json=_canonical_json(snapshot_document),
    )
